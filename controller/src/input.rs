//! Host-side input: find usable Linux evdev devices and turn their events into
//! a normalised drive state.
//!
//! Three device classes are supported, and they need genuinely different
//! handling -- see the submodules:
//!   ABS  absolute axes (gamepad/joystick), map straight through
//!   REL  relative deltas (mouse), integrate and decay
//!   KEY  arrow keys, track which are held

use std::io;
use std::path::PathBuf;

use evdev::{AbsoluteAxisCode, Device, EventType, KeyCode, RelativeAxisCode};
use tokio::sync::watch;

mod abs;
mod key;
mod rel;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DriveState {
    pub linear: f32,  // -1.0 ..= 1.0, positive is forward
    pub angular: f32, // -1.0 ..= 1.0, positive is right
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Abs,
    Rel,
    Key,
}

#[derive(Debug, Clone)]
pub struct DeviceInfo {
    pub path: PathBuf,
    pub name: String,
    pub kind: Kind,
}

const ARROWS: [KeyCode; 4] = [
    KeyCode::KEY_UP,
    KeyCode::KEY_DOWN,
    KeyCode::KEY_LEFT,
    KeyCode::KEY_RIGHT,
];

/// Devices overlap heavily, so this is ordered most-specific first and every
/// arm pairs an axis test with a button test. Advertised axes on their own are
/// not evidence of anything: an ASRock LED controller claims
/// ABS_X/Y/Z/RX/RY/RZ/THROTTLE/RUDDER *and* the four arrow keys while being
/// incapable of emitting a single event, and a Keychron K10 claims REL_X/REL_Y
/// and BTN_LEFT for its mouse-key emulation.
fn classify(dev: &Device) -> Option<Kind> {
    let ev = dev.supported_events();
    let keys = dev.supported_keys();
    let has = |k: KeyCode| keys.is_some_and(|s| s.contains(k));

    // Gamepad or joystick: a real one carries a face button or a trigger.
    if ev.contains(EventType::ABSOLUTE) {
        if let Some(ax) = dev.supported_absolute_axes() {
            if ax.contains(AbsoluteAxisCode::ABS_X)
                && ax.contains(AbsoluteAxisCode::ABS_Y)
                && (has(KeyCode::BTN_SOUTH) || has(KeyCode::BTN_TRIGGER))
            {
                return Some(Kind::Abs);
            }
        }
    }

    // Keyboard: the four arrows, plus letters. The letters do double duty --
    // they rule out remotes and power-button nodes that claim arrow keys, and
    // they settle the keyboard-vs-mouse question below. Nothing but a keyboard
    // has a full alphabet, so this is tested before RELATIVE: a Keychron K10
    // advertises REL_X/REL_Y *and* BTN_LEFT for its mouse-key emulation, and
    // would otherwise be offered as a mouse.
    if ev.contains(EventType::KEY)
        && ARROWS.iter().all(|k| has(*k))
        && has(KeyCode::KEY_A)
        && has(KeyCode::KEY_Z)
    {
        return Some(Kind::Key);
    }

    // Mouse or trackball: BTN_LEFT separates a real pointer from a device that
    // merely advertises relative axes.
    if ev.contains(EventType::RELATIVE) {
        if let Some(ax) = dev.supported_relative_axes() {
            if ax.contains(RelativeAxisCode::REL_X)
                && ax.contains(RelativeAxisCode::REL_Y)
                && has(KeyCode::BTN_LEFT)
            {
                return Some(Kind::Rel);
            }
        }
    }

    None
}

/// Every device we know how to drive from. Devices we cannot open (permissions)
/// are skipped rather than reported -- `/dev/input/event*` is `root:input`, so
/// the user needs to be in the `input` group.
pub fn enumerate() -> Vec<DeviceInfo> {
    let mut found: Vec<DeviceInfo> = evdev::enumerate()
        .filter_map(|(path, dev)| {
            let kind = classify(&dev)?;
            Some(DeviceInfo {
                path,
                name: dev.name().unwrap_or("<unnamed>").to_owned(),
                kind,
            })
        })
        .collect();

    // One physical device often exposes several event nodes (a keyboard's
    // consumer-control node, a gamepad's motion node). Keep the first of each
    // (name, kind) pair; the lowest event number is the one that carries the
    // events we want.
    found.sort_by(|a, b| (&a.name, a.kind as u8, &a.path).cmp(&(&b.name, b.kind as u8, &b.path)));
    found.dedup_by(|a, b| a.name == b.name && a.kind == b.kind);
    found
}

/// A live input device. Enum rather than `Box<dyn>`: the set of sources is
/// closed and lives in this crate, so dispatch stays static and each source is
/// free to keep whatever internal state it needs.
enum Source {
    Abs(abs::AbsSource),
    Rel(rel::RelSource),
    Key(key::KeySource),
}

impl Source {
    /// Resolves with the full current drive state every time it changes.
    ///
    /// Each source folds raw events into state internally, so the caller never
    /// sees deltas or key transitions -- just the latest target.
    async fn next(&mut self) -> io::Result<DriveState> {
        match self {
            Source::Abs(s) => s.next().await,
            Source::Rel(s) => s.next().await,
            Source::Key(s) => s.next().await,
        }
    }
}

/// Open a device and start reading it in the background.
///
/// The returned receiver is the interface every consumer uses: `control` reads
/// it to build packets, the UI reads it to draw, and neither can starve the
/// other. Latest-wins is the correct semantic for a setpoint -- a consumer that
/// falls behind loses resolution, never correctness.
///
/// When the device disappears the reader task ends and the channel closes,
/// which is how `control` learns the input is dead. Receivers keep returning
/// the last value forever, so detecting that closure is the *only* way to tell
/// a stopped stick from an unplugged one.
pub fn open(info: &DeviceInfo) -> io::Result<watch::Receiver<DriveState>> {
    let dev = Device::open(&info.path)?;
    let mut src = match info.kind {
        Kind::Abs => Source::Abs(abs::AbsSource::new(dev)?),
        Kind::Rel => Source::Rel(rel::RelSource::new(dev)?),
        Kind::Key => Source::Key(key::KeySource::new(dev)?),
    };

    let (tx, rx) = watch::channel(DriveState::default());
    tokio::spawn(async move {
        while let Ok(state) = src.next().await {
            if tx.send(state).is_err() {
                break; // nobody listening
            }
        }
    });
    Ok(rx)
}
