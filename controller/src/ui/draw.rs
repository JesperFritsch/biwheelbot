//! Rendering. Owns the layout arithmetic and nothing else -- all state lives in
//! `App`, except `App::scroll`, which is written here because this is the only
//! place the viewport height is known.

use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph};
use ratatui::Frame;

use super::App;
use crate::input::{DeviceInfo, Kind};

const TELEM_COLS: usize = 4;
/// Half-width of the drive bars, in cells.
const BAR_W: i32 = 24;

const SPINNER: [&str; 10] = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"];

/// Device menu, shown before anything is opened or connected.
pub fn picker(f: &mut Frame, devices: &[DeviceInfo], sel: usize) {
    let chunks = Layout::vertical([Constraint::Min(3), Constraint::Length(3)]).split(f.area());

    let name_w = devices.iter().map(|d| d.name.len()).max().unwrap_or(10);
    let lines: Vec<Line> = devices
        .iter()
        .enumerate()
        .map(|(i, d)| {
            let selected = i == sel;
            let style = if selected {
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::Cyan)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default()
            };
            Span::styled(
                format!(
                    "{} {:<5} {:<nw$}  {}",
                    if selected { ">" } else { " " },
                    kind_of(d),
                    d.name,
                    d.path.display(),
                    nw = name_w
                ),
                style,
            )
            .into()
        })
        .collect();

    f.render_widget(
        Paragraph::new(lines).block(
            Block::default()
                .borders(Borders::ALL)
                .title(" input device "),
        ),
        chunks[0],
    );
    f.render_widget(
        Paragraph::new(" ↑↓/jk select   enter open   q quit")
            .block(Block::default().borders(Borders::ALL)),
        chunks[1],
    );
}

/// `Kind` lives in `input` and has no Display; keep the mapping here rather
/// than leaking a UI concern into the device layer.
fn kind_of(d: &DeviceInfo) -> &'static str {
    match d.kind {
        Kind::Abs => "pad",
        Kind::Rel => "mouse",
        Kind::Key => "keys",
    }
}

/// Held while a slow step runs. The spinner is the only moving part, so a 15
/// second scan reads as working rather than hung.
pub fn working(f: &mut Frame, label: &str, frame: usize) {
    let msg = vec![
        Line::from(""),
        Line::from(Span::styled(
            format!("  {}  {label}", SPINNER[frame % SPINNER.len()]),
            Style::default().fg(Color::Cyan),
        )),
    ];
    f.render_widget(
        Paragraph::new(msg).block(Block::default().borders(Borders::ALL).title(" working ")),
        f.area(),
    );
}

pub fn ui(f: &mut Frame, app: &mut App) {
    // The telemetry pane sizes itself to the discovered field count: one row
    // per TELEM_COLS fields, plus a header line and the block borders.
    let telem_h = if app.telem_fields.is_empty() {
        0
    } else {
        (app.telem_fields.len().div_ceil(TELEM_COLS) + 3) as u16
    };

    let chunks = Layout::vertical([
        Constraint::Length(3),       // status
        Constraint::Length(4),       // drive
        Constraint::Min(5),          // gains
        Constraint::Length(telem_h), // telemetry
        Constraint::Length(3),       // footer
    ])
    .split(f.area());

    status(f, app, chunks[0]);
    drive(f, app, chunks[1]);
    gains(f, app, chunks[2]);
    if !app.telem_fields.is_empty() {
        f.render_widget(
            Paragraph::new(telem_lines(app))
                .block(Block::default().borders(Borders::ALL).title(" telemetry ")),
            chunks[3],
        );
    }
    footer(f, app, chunks[4]);
}

fn status(f: &mut Frame, app: &App, area: ratatui::layout::Rect) {
    let line = if app.status.is_empty() {
        let (link, link_style) = if app.state.connected {
            ("connected", Style::default().fg(Color::Green))
        } else {
            ("link down", Style::default().fg(Color::Red))
        };
        Line::from(vec![
            Span::styled(link, link_style),
            Span::raw(format!("   input: {}", app.device)),
            if app.state.input_dead {
                Span::styled("  (input gone -- holding zero)", Style::default().fg(Color::Red))
            } else {
                Span::raw("")
            },
        ])
    } else {
        Line::from(Span::styled(
            app.status.as_str(),
            Style::default().fg(Color::Red),
        ))
    };
    f.render_widget(
        Paragraph::new(line)
            .block(Block::default().borders(Borders::ALL).title(" BiWheelBot ")),
        area,
    );
}

fn drive(f: &mut Frame, app: &App, area: ratatui::layout::Rect) {
    let rows = vec![
        axis("linear ", app.state.drive.linear),
        axis("angular", app.state.drive.angular),
    ];
    f.render_widget(
        Paragraph::new(rows).block(Block::default().borders(Borders::ALL).title(" drive ")),
        area,
    );
}

fn axis(name: &str, v: f32) -> Line<'static> {
    let n = (v.clamp(-1.0, 1.0) * BAR_W as f32).round() as i32;
    let bar: String = (-BAR_W..=BAR_W)
        .map(|i| match i {
            0 => '|',
            i if i > 0 && i <= n => '#',
            i if i < 0 && i >= n => '#',
            _ => ' ',
        })
        .collect();
    let colour = if v.abs() < 0.001 {
        Color::DarkGray
    } else {
        Color::Cyan
    };
    Line::from(vec![
        Span::raw(format!(" {name} {v:+.2}  [")),
        Span::styled(bar, Style::default().fg(colour)),
        Span::raw("]"),
    ])
}

fn gains(f: &mut Frame, app: &mut App, area: ratatui::layout::Rect) {
    let mut lines: Vec<Line> = Vec::new();
    let mut prev_block: Option<usize> = None;
    let mut sel_line = 0usize;

    for i in 0..app.rows.len() {
        let (bi, _) = app.rows[i];
        if prev_block.is_some_and(|p| p != bi) {
            lines.push(Line::from(""));
        }
        prev_block = Some(bi);

        let selected = i == app.sel;
        if selected {
            sel_line = lines.len();
        }
        let marker = if selected { " > " } else { "   " };
        let value = format!("{:>14.6}", app.get(i));
        // A block with every gain at zero is disabled; dim it so the active
        // loops stand out at a glance.
        let inactive = app.blocks[bi].values.iter().all(|v| *v == 0.0);

        let style = if selected {
            Style::default()
                .fg(Color::Black)
                .bg(Color::Cyan)
                .add_modifier(Modifier::BOLD)
        } else if inactive {
            Style::default().fg(Color::DarkGray)
        } else {
            Style::default()
        };

        let dirty = if app.blocks[bi].dirty { "*" } else { " " };
        lines.push(Line::from(Span::styled(
            format!("{marker}{}{value}  {dirty}", app.label(i)),
            style,
        )));
    }

    // The pane is a fixed viewport onto a list that grows with whatever the
    // firmware advertises, so scroll only as far as it takes to keep the
    // selection visible -- surrounding rows stay put between keypresses.
    let view_h = (area.height.saturating_sub(2)).max(1) as usize;
    let max_scroll = lines.len().saturating_sub(view_h);
    let mut scroll = (app.scroll as usize).min(max_scroll);
    if sel_line < scroll {
        scroll = sel_line;
    } else if sel_line >= scroll + view_h {
        scroll = sel_line + 1 - view_h;
    }
    app.scroll = scroll as u16;

    let title = match (scroll > 0, scroll + view_h < lines.len()) {
        (false, false) => " gains ".to_string(),
        (above, below) => format!(
            " gains {}{} ",
            if above { "^" } else { " " },
            if below { "v" } else { " " }
        ),
    };

    f.render_widget(
        Paragraph::new(lines)
            .scroll((app.scroll, 0))
            .block(Block::default().borders(Borders::ALL).title(title)),
        area,
    );
}

fn telem_lines(app: &App) -> Vec<Line<'static>> {
    let mut lines = vec![Line::from(Span::styled(
        format!(" {} packets   {:.1} Hz", app.state.telem_packets, app.telem_hz()),
        Style::default().fg(Color::DarkGray),
    ))];

    for (r, names) in app.telem_fields.chunks(TELEM_COLS).enumerate() {
        let mut s = String::from(" ");
        for (c, name) in names.iter().enumerate() {
            // A field the firmware has no reading for arrives as NaN.
            match app.state.telem.get(r * TELEM_COLS + c) {
                Some(v) if !v.is_nan() => s.push_str(&format!("{name:>6} {v:>10.3}   ")),
                _ => s.push_str(&format!("{name:>6}       --   ")),
            }
        }
        lines.push(Line::from(s));
    }
    lines
}

fn footer(f: &mut Frame, app: &App, area: ratatui::layout::Rect) {
    let line = match &app.edit {
        Some(buf) => Line::from(vec![
            Span::styled("value: ", Style::default().fg(Color::Yellow)),
            Span::styled(
                format!("{buf}_"),
                Style::default().add_modifier(Modifier::BOLD),
            ),
            Span::raw("   enter=apply  esc=cancel"),
        ]),
        None => Line::from(
            " tab/hl select   jk ±1%   JK ±10%   enter edit   0 zero   r reload   q quit",
        ),
    };
    f.render_widget(
        Paragraph::new(line).block(Block::default().borders(Borders::ALL)),
        area,
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::State;
    use crate::ui::{App, GainRow};
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;

    fn names(s: &str) -> Vec<String> {
        s.split(',').map(str::to_string).collect()
    }

    /// Mirrors the real firmware: four 3-field gain blocks, 16 telemetry fields.
    fn app() -> App {
        let blocks: Vec<GainRow> = ["balance", "speed", "position", "turn"]
            .iter()
            .map(|n| GainRow {
                name: n.to_string(),
                fields: names("kp,ki,kd"),
                values: vec![1.0, 0.0, 0.25],
                dirty: false,
            })
            .collect();
        let telem_fields =
            names("ang,rate,kfa,cmp,tpos,pos,tang,tspd,eff,spd,d_a,d_b,t_d,bat,en,ovr");
        let mut app = App::for_test(blocks, "Microsoft X-Box 360 pad".into());
        app.state = State {
            drive: Default::default(),
            telem: vec![f32::NAN; telem_fields.len()],
            telem_packets: 0,
            connected: true,
            input_dead: false,
        };
        app.telem_fields = telem_fields;
        app
    }

    /// The layout arithmetic subtracts border widths and slices a scroll
    /// viewport, both of which underflow on a small enough terminal.
    #[test]
    fn renders_without_panicking_at_any_size() {
        for (w, h) in [(200, 60), (120, 40), (80, 24), (60, 20), (40, 10), (20, 5), (4, 2)] {
            let mut term = Terminal::new(TestBackend::new(w, h)).unwrap();
            let mut app = app();
            term.draw(|f| ui(f, &mut app)).unwrap();
        }
    }

    #[test]
    fn selection_scrolls_the_gains_viewport() {
        let mut term = Terminal::new(TestBackend::new(80, 16)).unwrap();
        let mut app = app();
        app.sel = app.rows.len() - 1;
        term.draw(|f| ui(f, &mut app)).unwrap();
        assert!(app.scroll > 0, "bottom selection should have scrolled");
    }

    #[test]
    fn missing_telemetry_collapses_its_pane() {
        let mut term = Terminal::new(TestBackend::new(80, 24)).unwrap();
        let mut app = app();
        app.telem_fields.clear();
        term.draw(|f| ui(f, &mut app)).unwrap();
    }

    /// A dead input and a dropped link both have to be visible; without them a
    /// stopped robot looks the same as a working one nobody is driving.
    #[test]
    fn failure_states_render() {
        let mut term = Terminal::new(TestBackend::new(90, 30)).unwrap();
        let mut app = app();
        app.state.connected = false;
        app.state.input_dead = true;
        term.draw(|f| ui(f, &mut app)).unwrap();
    }

    #[test]
    fn picker_and_spinner_render() {
        let devices = crate::input::enumerate();
        let mut term = Terminal::new(TestBackend::new(80, 10)).unwrap();
        term.draw(|f| picker(f, &devices, 0)).unwrap();
        term.draw(|f| working(f, "scanning…", 3)).unwrap();
    }
}
