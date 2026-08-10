//! Live PID gain tuning for BiWheelBot over BLE.
//!
//! Gain blocks are discovered at runtime rather than hardcoded. Every
//! characteristic in the config service that carries a Characteristic User
//! Description descriptor (0x2901) is treated as a block, and the descriptor
//! string is its schema:
//!
//!     "balance:kp,ki,kd"
//!      ^name   ^fields, in value order
//!
//! The value is that many little-endian f32 in that order -- the same layout
//! `decode_gains()` expects in src/com.cpp. Adding a fourth gain term, or a
//! whole new PID, is a firmware-only change: this tool grows a row on its own.
//!
//!     cargo run --release
//!
//! Arrow up/down step the selected gain by 1% of its own magnitude, so one
//! keypress means the same thing whether the gain is 0.0005 or 20. A gain
//! sitting at exactly zero has no magnitude to scale, so it steps by FLOOR_STEP
//! instead -- otherwise a zeroed gain could never be raised.

use std::io;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};
use btleplug::api::{Central, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType};
use btleplug::platform::{Manager, Peripheral};
use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use crossterm::execute;
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph};
use ratatui::{Frame, Terminal};
use uuid::{uuid, Uuid};

const DEVICE_NAME: &str = "BiWheelBot";

/// The one UUID still hardcoded, and deliberately: it is the anchor that says
/// which characteristics are gains. Discovering everything instead would sweep
/// up cmdChar and try to render it as a PID.
const CONFIG_SVC: Uuid = uuid!("19b10002-e8f2-537e-4f6c-d104768a1214");

/// 16-bit SIG-assigned UUIDs are an alias for a slot in the Bluetooth Base
/// UUID -- only bits 96..111 vary, everything else is fixed. The host stack
/// always hands them back expanded, so match on the long form.
const fn sig_uuid(short: u16) -> Uuid {
    Uuid::from_u128(0x00000000_0000_1000_8000_00805f9b34fb | ((short as u128) << 96))
}

/// Characteristic User Description.
const USER_DESC: Uuid = sig_uuid(0x2901);

const FINE: f32 = 0.01; // arrow keys: 1%
const COARSE: f32 = 0.10; // shift+arrow: 10%
const FLOOR_STEP: f32 = 1e-4; // absolute step when the gain is exactly zero
const SCAN_TIMEOUT: Duration = Duration::from_secs(15);
const WRITE_COALESCE: Duration = Duration::from_millis(60);

/// One characteristic's worth of gains, as described by its 0x2901 schema.
struct Group {
    name: String,
    fields: Vec<String>,
    values: Vec<f32>,
    ch: Characteristic,
    /// Set when a value changed locally and hasn't reached the device yet.
    dirty: bool,
}

struct App {
    groups: Vec<Group>,
    /// Flattened (group, field) index, in display order.
    rows: Vec<(usize, usize)>,
    sel: usize,
    edit: Option<String>,
    status: String,
    label_w: usize,
}

impl App {
    fn new(groups: Vec<Group>) -> Self {
        let mut rows = Vec::new();
        for (gi, g) in groups.iter().enumerate() {
            for fi in 0..g.fields.len() {
                rows.push((gi, fi));
            }
        }
        let label_w = groups.iter().map(|g| g.name.len()).max().unwrap_or(8);
        App {
            groups,
            rows,
            sel: 0,
            edit: None,
            status: String::new(),
            label_w,
        }
    }

    fn get(&self, i: usize) -> f32 {
        let (g, f) = self.rows[i];
        self.groups[g].values[f]
    }

    fn set(&mut self, i: usize, v: f32) {
        let (g, f) = self.rows[i];
        self.groups[g].values[f] = v;
        self.groups[g].dirty = true;
    }

    fn label(&self, i: usize) -> String {
        let (g, f) = self.rows[i];
        format!(
            "{:<w$}  {}",
            self.groups[g].name,
            self.groups[g].fields[f],
            w = self.label_w
        )
    }

    /// Step by a fraction of the value's own magnitude, so one keypress means
    /// the same thing across four orders of magnitude of gain.
    fn nudge(&mut self, i: usize, dir: f32, frac: f32) {
        let v = self.get(i);
        let step = if v.abs() < 1e-9 {
            FLOOR_STEP
        } else {
            v.abs() * frac
        };
        let next = v + dir * step;
        // Snap through zero rather than leaving a denormal behind.
        self.set(i, if next.abs() < FLOOR_STEP / 2.0 { 0.0 } else { next });
    }
}

fn encode(v: &[f32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(v.len() * 4);
    for x in v {
        out.extend_from_slice(&x.to_le_bytes());
    }
    out
}

fn decode(b: &[u8], n: usize) -> Option<Vec<f32>> {
    if b.len() < n * 4 {
        return None;
    }
    Some(
        (0..n)
            .map(|i| f32::from_le_bytes(b[i * 4..i * 4 + 4].try_into().unwrap()))
            .collect(),
    )
}

/// `"balance:kp,ki,kd"` -> `("balance", ["kp", "ki", "kd"])`.
fn parse_schema(s: &str) -> Option<(String, Vec<String>)> {
    let s = s.trim_end_matches('\0').trim();
    let (name, rest) = s.split_once(':')?;
    let fields: Vec<String> = rest
        .split(',')
        .map(str::trim)
        .filter(|f| !f.is_empty())
        .map(str::to_string)
        .collect();
    if name.is_empty() || fields.is_empty() {
        return None;
    }
    Some((name.trim().to_string(), fields))
}

/// The 16-bit slice of a UUID, for terse diagnostics.
fn short(u: &Uuid) -> String {
    u.to_string()[4..8].to_string()
}

async fn connect() -> Result<Peripheral> {
    let manager = Manager::new().await?;
    let central = manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or_else(|| anyhow!("no Bluetooth adapter found"))?;

    central.start_scan(ScanFilter::default()).await?;
    let deadline = Instant::now() + SCAN_TIMEOUT;

    loop {
        for p in central.peripherals().await? {
            let name = p.properties().await?.and_then(|pr| pr.local_name);
            if name.as_deref() == Some(DEVICE_NAME) {
                central.stop_scan().await.ok();
                p.connect().await?;
                p.discover_services().await?;
                return Ok(p);
            }
        }
        if Instant::now() > deadline {
            central.stop_scan().await.ok();
            return Err(anyhow!(
                "{DEVICE_NAME} not found in {}s -- is it powered and not already connected?",
                SCAN_TIMEOUT.as_secs()
            ));
        }
        tokio::time::sleep(Duration::from_millis(300)).await;
    }
}

/// Walk the config service and build a group per self-describing characteristic.
/// Returns the groups plus a note for anything skipped, so a firmware that is
/// missing a descriptor says so instead of silently vanishing from the list.
async fn discover(p: &Peripheral) -> Result<(Vec<Group>, Vec<String>)> {
    let mut chars: Vec<Characteristic> = p
        .characteristics()
        .into_iter()
        .filter(|c| c.service_uuid == CONFIG_SVC)
        .collect();
    // BTreeSet order is already by UUID, but the display order is user-facing.
    chars.sort_by_key(|c| c.uuid);

    let mut groups = Vec::new();
    let mut skipped = Vec::new();

    for ch in chars {
        let Some(desc) = ch.descriptors.iter().find(|d| d.uuid == USER_DESC).cloned() else {
            skipped.push(format!("{} has no 0x2901", short(&ch.uuid)));
            continue;
        };
        let raw = match p.read_descriptor(&desc).await {
            Ok(v) => v,
            Err(e) => {
                skipped.push(format!("{} descriptor unreadable: {e}", short(&ch.uuid)));
                continue;
            }
        };
        let text = String::from_utf8_lossy(&raw).into_owned();
        let Some((name, fields)) = parse_schema(&text) else {
            skipped.push(format!("{} bad schema {text:?}", short(&ch.uuid)));
            continue;
        };
        let n = fields.len();
        groups.push(Group {
            name,
            fields,
            values: vec![0.0; n],
            ch,
            dirty: false,
        });
    }

    Ok((groups, skipped))
}

fn ui(f: &mut Frame, app: &App) {
    let chunks = Layout::vertical([
        Constraint::Length(3),
        Constraint::Min(5),
        Constraint::Length(3),
    ])
    .split(f.area());

    f.render_widget(
        Paragraph::new(app.status.as_str())
            .block(Block::default().borders(Borders::ALL).title(" BiWheelBot ")),
        chunks[0],
    );

    let mut lines: Vec<Line> = Vec::new();
    let mut prev_group: Option<usize> = None;

    for i in 0..app.rows.len() {
        let (gi, _) = app.rows[i];
        if prev_group.is_some_and(|p| p != gi) {
            lines.push(Line::from(""));
        }
        prev_group = Some(gi);

        let selected = i == app.sel;
        let marker = if selected { " > " } else { "   " };
        let value = format!("{:>14.6}", app.get(i));
        // A block with every gain at zero is disabled; dim it so the active
        // loops stand out at a glance.
        let inactive = app.groups[gi].values.iter().all(|v| *v == 0.0);

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

        let dirty = if app.groups[gi].dirty { "*" } else { " " };
        lines.push(Line::from(vec![Span::styled(
            format!("{marker}{}{value}  {dirty}", app.label(i)),
            style,
        )]));
    }

    f.render_widget(
        Paragraph::new(lines).block(Block::default().borders(Borders::ALL).title(" gains ")),
        chunks[1],
    );

    let footer = match &app.edit {
        Some(buf) => Line::from(vec![
            Span::styled("value: ", Style::default().fg(Color::Yellow)),
            Span::styled(
                format!("{buf}_"),
                Style::default().add_modifier(Modifier::BOLD),
            ),
            Span::raw("   enter=apply  esc=cancel"),
        ]),
        None => Line::from(
            "tab/←→ select   ↑↓ ±1%   shift+↑↓ ±10%   enter edit   0 zero   r reload   q quit",
        ),
    };
    f.render_widget(
        Paragraph::new(footer).block(Block::default().borders(Borders::ALL)),
        chunks[2],
    );
}

#[tokio::main]
async fn main() -> Result<()> {
    println!("scanning for {DEVICE_NAME}...");
    let peripheral = connect().await?;

    let (mut groups, skipped) = discover(&peripheral).await?;
    if groups.is_empty() {
        peripheral.disconnect().await.ok();
        return Err(anyhow!(
            "no gain characteristics found in service {CONFIG_SVC}{}",
            if skipped.is_empty() {
                String::new()
            } else {
                format!(" ({})", skipped.join("; "))
            }
        ));
    }

    // Seed from the device. A group whose read fails keeps its zeros, but stays
    // clean, so nothing is written back over good gains unless you edit it.
    let mut notes: Vec<String> = skipped;
    for g in &mut groups {
        match peripheral.read(&g.ch).await {
            Ok(v) => match decode(&v, g.fields.len()) {
                Some(vals) => g.values = vals,
                None => notes.push(format!(
                    "{}: expected {} bytes, got {}",
                    g.name,
                    g.fields.len() * 4,
                    v.len()
                )),
            },
            Err(e) => notes.push(format!("{}: read failed ({e})", g.name)),
        }
    }

    let found = groups
        .iter()
        .map(|g| g.name.as_str())
        .collect::<Vec<_>>()
        .join(", ");
    let mut app = App::new(groups);
    app.status = if notes.is_empty() {
        format!("connected to {DEVICE_NAME} -- {found}")
    } else {
        format!("connected to {DEVICE_NAME} -- {found}  [{}]", notes.join("; "))
    };

    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let mut terminal = Terminal::new(CrosstermBackend::new(stdout))?;

    let result = run(&mut terminal, &mut app, &peripheral).await;

    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;
    peripheral.disconnect().await.ok();

    result
}

async fn run(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    app: &mut App,
    p: &Peripheral,
) -> Result<()> {
    let mut last_write = Instant::now();

    loop {
        terminal.draw(|f| ui(f, app))?;

        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.kind != KeyEventKind::Press {
                    continue;
                }
                let shift = key.modifiers.contains(KeyModifiers::SHIFT);

                match app.edit.take() {
                    // ---- editing a literal value ----
                    Some(mut buf) => match key.code {
                        KeyCode::Esc => {}
                        KeyCode::Enter => match buf.trim().parse::<f32>() {
                            Ok(v) if v.is_finite() => {
                                app.set(app.sel, v);
                                app.status = format!("set {} = {v}", app.label(app.sel));
                            }
                            _ => app.status = format!("'{buf}' is not a finite number"),
                        },
                        KeyCode::Backspace => {
                            buf.pop();
                            app.edit = Some(buf);
                        }
                        KeyCode::Char(c) if c.is_ascii_digit() || ".-+eE".contains(c) => {
                            buf.push(c);
                            app.edit = Some(buf);
                        }
                        _ => app.edit = Some(buf),
                    },

                    // ---- normal ----
                    None => match key.code {
                        KeyCode::Char('q') | KeyCode::Esc => break,
                        KeyCode::Tab | KeyCode::Right => app.sel = (app.sel + 1) % app.rows.len(),
                        KeyCode::BackTab | KeyCode::Left => {
                            app.sel = (app.sel + app.rows.len() - 1) % app.rows.len()
                        }
                        KeyCode::Up => app.nudge(app.sel, 1.0, if shift { COARSE } else { FINE }),
                        KeyCode::Down => app.nudge(app.sel, -1.0, if shift { COARSE } else { FINE }),
                        KeyCode::Char('0') => app.set(app.sel, 0.0),
                        KeyCode::Enter | KeyCode::Char('e') => {
                            app.edit = Some(format!("{}", app.get(app.sel)))
                        }
                        KeyCode::Char('r') => {
                            for i in 0..app.groups.len() {
                                let ch = app.groups[i].ch.clone();
                                let n = app.groups[i].fields.len();
                                if let Ok(v) = p.read(&ch).await {
                                    if let Some(vals) = decode(&v, n) {
                                        app.groups[i].values = vals;
                                    }
                                }
                                app.groups[i].dirty = false;
                            }
                            app.status = "reloaded from device".into();
                        }
                        _ => {}
                    },
                }
            }
        }

        // Coalesce writes: holding an arrow key auto-repeats far faster than a
        // BLE round trip, and every write is the whole characteristic.
        if last_write.elapsed() >= WRITE_COALESCE {
            let pending: Vec<(usize, Characteristic, Vec<u8>)> = app
                .groups
                .iter()
                .enumerate()
                .filter(|(_, g)| g.dirty)
                .map(|(i, g)| (i, g.ch.clone(), encode(&g.values)))
                .collect();

            for (i, ch, bytes) in pending {
                match p.write(&ch, &bytes, WriteType::WithResponse).await {
                    Ok(()) => {
                        app.groups[i].dirty = false;
                        let g = &app.groups[i];
                        let shown = g
                            .fields
                            .iter()
                            .zip(&g.values)
                            .map(|(f, v)| format!("{f}={v:.6}"))
                            .collect::<Vec<_>>()
                            .join(" ");
                        app.status = format!("wrote {}  {shown}", g.name);
                    }
                    Err(e) => app.status = format!("write failed: {e}"),
                }
            }
            last_write = Instant::now();
        }
    }

    Ok(())
}
