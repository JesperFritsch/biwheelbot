//! Terminal UI. A reader of `control::State`, never a participant in the
//! control loop -- a slow redraw or a resize cannot cost a setpoint.
//!
//! The one thing it does drive directly is gains, via `Handle`, which reaches
//! the robot on this task rather than through the loop.
//!
//! # Two keyboards
//!
//! Drive input comes from evdev (the raw device) while UI keys come from
//! crossterm (the terminal). If you pick a keyboard as the drive device, the
//! arrow keys do both at once. Every UI binding therefore has a non-arrow
//! alternative: use `hjkl` when driving from the keyboard.

mod draw;

use std::future::Future;
use std::io;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use crossterm::event::{Event, EventStream, KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use crossterm::execute;
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use futures::StreamExt;
use ratatui::backend::CrosstermBackend;
use ratatui::Terminal;
use tokio::time::{self, MissedTickBehavior};

use crate::control::{Handle, State};
use crate::input::DeviceInfo;

/// Redraw rate. Independent of the control loop's tick by construction.
const FRAME: Duration = Duration::from_millis(33);
/// Quiet period before a nudged gain is written, so holding a key down does not
/// put one BLE write on the air per key repeat.
const WRITE_COALESCE: Duration = Duration::from_millis(60);

/// jk / arrow step, as a fraction of the gain's own magnitude.
const FINE: f32 = 0.01;
/// JK / shift-arrow step.
const COARSE: f32 = 0.10;
/// A gain at exactly zero has no magnitude to scale, so it steps by this
/// instead -- otherwise a zeroed gain could never be raised.
const FLOOR_STEP: f32 = 1e-4;

type Term = Terminal<CrosstermBackend<io::Stdout>>;

/// The terminal, held for as long as anything needs to draw.
///
/// Restoring happens in `Drop`, which covers the ordinary return, the `?` on a
/// failed connect, and an unwinding panic alike -- there is no path that leaves
/// a shell in raw mode with no echo.
pub struct Screen {
    term: Term,
}

impl Screen {
    pub fn open() -> Result<Screen> {
        // Opens /dev/tty, so it fails with ENXIO when there is no controlling
        // terminal -- a bare "No such device or address" is worth explaining.
        enable_raw_mode().context("no terminal: this needs to run from a tty")?;
        let mut out = io::stdout();
        execute!(out, EnterAlternateScreen)?;
        Ok(Screen {
            term: Terminal::new(CrosstermBackend::new(out))?,
        })
    }

    /// Arrow-key menu over the discovered input devices. `None` means quit.
    pub async fn pick(&mut self, devices: &[DeviceInfo]) -> Result<Option<usize>> {
        let mut sel = 0usize;
        let mut keys = EventStream::new();
        loop {
            self.term.draw(|f| draw::picker(f, devices, sel))?;

            let Some(Ok(ev)) = keys.next().await else {
                return Ok(None); // stdin closed
            };
            let Event::Key(key) = ev else { continue };
            if key.kind != KeyEventKind::Press {
                continue;
            }
            match key.code {
                KeyCode::Up | KeyCode::Char('k') => {
                    sel = sel.checked_sub(1).unwrap_or(devices.len() - 1)
                }
                KeyCode::Down | KeyCode::Char('j') => sel = (sel + 1) % devices.len(),
                KeyCode::Home => sel = 0,
                KeyCode::End => sel = devices.len() - 1,
                KeyCode::Enter | KeyCode::Char(' ') => return Ok(Some(sel)),
                KeyCode::Char('q') | KeyCode::Esc => return Ok(None),
                _ => {}
            }
        }
    }

    /// Run `work` behind a spinner, so a 15 second scan reads as working rather
    /// than hung.
    pub async fn working<T>(
        &mut self,
        label: &str,
        work: impl Future<Output = Result<T>>,
    ) -> Result<T> {
        tokio::pin!(work);
        let mut spin = time::interval(Duration::from_millis(120));
        let mut frame = 0usize;
        loop {
            tokio::select! {
                done = &mut work => return done,
                _ = spin.tick() => {
                    self.term.draw(|f| draw::working(f, label, frame))?;
                    frame += 1;
                }
            }
        }
    }

    pub async fn run(&mut self, handle: Handle, device: String) -> Result<()> {
        let mut app = App::new(&handle, device).await;
        let mut keys = EventStream::new();
        let mut frame = time::interval(FRAME);
        frame.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let mut pending: Option<Instant> = None;
        let mut handle = handle;

        loop {
            tokio::select! {
                _ = frame.tick() => {
                    app.state = handle.state();
                    if pending.is_some_and(|t| t.elapsed() >= WRITE_COALESCE) {
                        pending = None;
                        app.flush(&handle).await;
                    }
                    self.term.draw(|f| draw::ui(f, &mut app))?;
                }
                Some(Ok(Event::Key(key))) = keys.next() => {
                    if app.key(key, &mut pending, &handle).await {
                        return Ok(());
                    }
                }
            }
        }
    }
}

impl Drop for Screen {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
        let _ = execute!(self.term.backend_mut(), LeaveAlternateScreen);
        let _ = self.term.show_cursor();
    }
}

pub struct GainRow {
    pub name: String,
    pub fields: Vec<String>,
    pub values: Vec<f32>,
    /// Edited since the last successful write.
    pub dirty: bool,
}

pub struct App {
    pub blocks: Vec<GainRow>,
    /// Flattened (block, field) index in display order -- the selection moves
    /// over fields, not blocks.
    pub rows: Vec<(usize, usize)>,
    pub sel: usize,
    pub edit: Option<String>,
    pub status: String,
    pub label_w: usize,
    /// First visible line of the gains pane. Owned by the draw code, which is
    /// the only place the viewport height is known.
    pub scroll: u16,

    pub state: State,
    pub telem_fields: Vec<String>,
    pub device: String,
    pub started: Instant,
}

impl App {
    async fn new(handle: &Handle, device: String) -> Self {
        let blocks = load(handle).await;
        let mut rows = Vec::new();
        for (bi, b) in blocks.iter().enumerate() {
            for fi in 0..b.fields.len() {
                rows.push((bi, fi));
            }
        }
        let label_w = blocks.iter().map(|b| b.name.len()).max().unwrap_or(8);
        App {
            blocks,
            rows,
            sel: 0,
            edit: None,
            status: String::new(),
            label_w,
            scroll: 0,
            state: State {
                drive: Default::default(),
                telem: Vec::new(),
                telem_packets: 0,
                connected: true,
                input_dead: false,
            },
            telem_fields: handle.telem_fields().to_vec(),
            device,
            started: Instant::now(),
        }
    }

    pub fn telem_hz(&self) -> f64 {
        let secs = self.started.elapsed().as_secs_f64();
        if secs > 0.0 {
            self.state.telem_packets as f64 / secs
        } else {
            0.0
        }
    }

    pub fn get(&self, i: usize) -> f32 {
        let (b, f) = self.rows[i];
        self.blocks[b].values[f]
    }

    fn set(&mut self, i: usize, v: f32) {
        let (b, f) = self.rows[i];
        self.blocks[b].values[f] = v;
        self.blocks[b].dirty = true;
    }

    pub fn label(&self, i: usize) -> String {
        let (b, f) = self.rows[i];
        format!(
            "{:<w$}  {}",
            self.blocks[b].name,
            self.blocks[b].fields[f],
            w = self.label_w
        )
    }

    /// Step by a fraction of the value's own magnitude, so one keypress means
    /// the same thing whether the gain is 0.0005 or 20.
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

    /// Write every dirty block, clearing the flag only on success so a failed
    /// write is retried rather than silently forgotten.
    async fn flush(&mut self, handle: &Handle) {
        for i in 0..self.blocks.len() {
            if !self.blocks[i].dirty {
                continue;
            }
            let values = self.blocks[i].values.clone();
            match handle.write_gains(i, &values).await {
                Ok(()) => {
                    self.blocks[i].dirty = false;
                    self.status.clear();
                }
                Err(e) => self.status = format!("{e}"),
            }
        }
    }

    /// Returns true when the user asked to quit.
    async fn key(
        &mut self,
        key: KeyEvent,
        pending: &mut Option<Instant>,
        handle: &Handle,
    ) -> bool {
        // Windows terminals report press and release; without this every
        // binding fires twice.
        if key.kind != KeyEventKind::Press {
            return false;
        }
        let shift = key.modifiers.contains(KeyModifiers::SHIFT);

        // Typing a value: the buffer swallows everything until enter or esc.
        if let Some(buf) = self.edit.as_mut() {
            match key.code {
                KeyCode::Esc => self.edit = None,
                KeyCode::Enter => {
                    let text = buf.trim().to_string();
                    match text.parse::<f32>() {
                        Ok(v) if v.is_finite() => {
                            let sel = self.sel;
                            self.set(sel, v);
                            *pending = Some(Instant::now());
                            self.edit = None;
                        }
                        _ => self.status = format!("not a number: {text:?}"),
                    }
                }
                KeyCode::Backspace => {
                    buf.pop();
                }
                KeyCode::Char(c) if c.is_ascii_digit() || ".-+eE".contains(c) => buf.push(c),
                _ => {}
            }
            return false;
        }

        if self.rows.is_empty() {
            return matches!(key.code, KeyCode::Char('q') | KeyCode::Esc);
        }
        let last = self.rows.len() - 1;
        let sel = self.sel;

        match key.code {
            KeyCode::Char('q') | KeyCode::Esc => return true,

            // hl/jk exist so the arrows stay free for driving.
            KeyCode::Tab | KeyCode::Char('l') | KeyCode::Right => {
                self.sel = if sel >= last { 0 } else { sel + 1 }
            }
            KeyCode::BackTab | KeyCode::Char('h') | KeyCode::Left => {
                self.sel = if sel == 0 { last } else { sel - 1 }
            }

            KeyCode::Up | KeyCode::Char('k') => {
                self.nudge(sel, 1.0, if shift { COARSE } else { FINE });
                *pending = Some(Instant::now());
            }
            KeyCode::Down | KeyCode::Char('j') => {
                self.nudge(sel, -1.0, if shift { COARSE } else { FINE });
                *pending = Some(Instant::now());
            }
            KeyCode::Char('K') => {
                self.nudge(sel, 1.0, COARSE);
                *pending = Some(Instant::now());
            }
            KeyCode::Char('J') => {
                self.nudge(sel, -1.0, COARSE);
                *pending = Some(Instant::now());
            }

            KeyCode::Char('0') => {
                self.set(sel, 0.0);
                *pending = Some(Instant::now());
            }
            KeyCode::Enter | KeyCode::Char('e') => self.edit = Some(String::new()),

            KeyCode::Char('r') => {
                self.blocks = load(handle).await;
                self.status = "reloaded from robot".into();
            }
            _ => {}
        }
        false
    }
}

async fn load(handle: &Handle) -> Vec<GainRow> {
    let mut blocks = Vec::new();
    for (i, b) in handle.gains().iter().enumerate() {
        let values = handle
            .read_gains(i)
            .await
            .unwrap_or_else(|_| vec![f32::NAN; b.fields.len()]);
        blocks.push(GainRow {
            name: b.name.clone(),
            fields: b.fields.clone(),
            values,
            dirty: false,
        });
    }
    blocks
}

#[cfg(test)]
impl App {
    /// Rendering tests need an App without a robot behind it.
    pub fn for_test(blocks: Vec<GainRow>, device: String) -> Self {
        let mut rows = Vec::new();
        for (bi, b) in blocks.iter().enumerate() {
            for fi in 0..b.fields.len() {
                rows.push((bi, fi));
            }
        }
        let label_w = blocks.iter().map(|b| b.name.len()).max().unwrap_or(8);
        App {
            blocks,
            rows,
            sel: 0,
            edit: None,
            status: String::new(),
            label_w,
            scroll: 0,
            state: crate::control::State {
                drive: Default::default(),
                telem: Vec::new(),
                telem_packets: 0,
                connected: true,
                input_dead: false,
            },
            telem_fields: Vec::new(),
            device,
            started: Instant::now(),
        }
    }
}
