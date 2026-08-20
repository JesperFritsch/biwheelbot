//! Arrow keys.
//!
//! Unlike a terminal, evdev delivers key *release* events, so hold-to-drive
//! works: we track which arrows are down and derive a full-scale setpoint from
//! the four bits.

use std::io;

use evdev::{Device, EventStream, EventType, KeyCode};

use super::DriveState;

#[derive(Default)]
struct Held {
    up: bool,
    down: bool,
    left: bool,
    right: bool,
}

pub struct KeySource {
    events: EventStream,
    held: Held,
}

impl KeySource {
    pub fn new(dev: Device) -> io::Result<Self> {
        Ok(KeySource {
            events: dev.into_event_stream()?,
            held: Held::default(),
        })
    }

    pub async fn next(&mut self) -> io::Result<DriveState> {
        loop {
            let ev = self.events.next_event().await?;
            if ev.event_type() != EventType::KEY {
                continue;
            }

            // 0 = release, 1 = press, 2 = autorepeat. Repeats carry nothing new
            // -- the key is already marked held.
            let down = match ev.value() {
                0 => false,
                1 => true,
                _ => continue,
            };

            match KeyCode(ev.code()) {
                KeyCode::KEY_UP => self.held.up = down,
                KeyCode::KEY_DOWN => self.held.down = down,
                KeyCode::KEY_LEFT => self.held.left = down,
                KeyCode::KEY_RIGHT => self.held.right = down,
                _ => continue,
            }

            // Opposing keys cancel, which is what you want when a release event
            // is lost: pressing the other direction still recovers.
            let axis = |pos: bool, neg: bool| pos as i8 as f32 - neg as i8 as f32;
            return Ok(DriveState {
                linear: axis(self.held.up, self.held.down),
                angular: axis(self.held.right, self.held.left),
            });
        }
    }
}
