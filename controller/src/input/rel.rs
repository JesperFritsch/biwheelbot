//! Relative axes: mice and trackballs.
//!
//! The awkward case. REL reports deltas and says nothing at all once the mouse
//! stops, so there is no resting position to fall back to -- integrating alone
//! would leave the bot driving forever after one flick. This is the only source
//! that needs a clock: it integrates deltas and decays toward zero on a timer,
//! so letting go of the mouse coasts to a stop.

use std::io;
use std::time::Duration;

use evdev::{Device, EventStream, EventType, RelativeAxisCode};
use tokio::time::{interval, Interval};

use super::DriveState;

/// Counts of mouse movement to reach full scale.
const SENS: f32 = 1.0 / 400.0;
const DECAY_DT: Duration = Duration::from_millis(20);
/// Seconds to fall to 1/e of the current setpoint.
const TAU: f32 = 0.25;
/// Below this, snap to zero rather than trailing an exponential tail forever.
const SNAP: f32 = 0.01;

pub struct RelSource {
    events: EventStream,
    decay: Interval,
    k: f32,
    state: DriveState,
}

impl RelSource {
    pub fn new(mut dev: Device) -> io::Result<Self> {
        // Without the grab the mouse drives the desktop cursor at the same time
        // as the bot.
        dev.grab()?;
        Ok(RelSource {
            events: dev.into_event_stream()?,
            decay: interval(DECAY_DT),
            k: (-DECAY_DT.as_secs_f32() / TAU).exp(),
            state: DriveState::default(),
        })
    }

    pub async fn next(&mut self) -> io::Result<DriveState> {
        loop {
            tokio::select! {
                ev = self.events.next_event() => {
                    let ev = ev?;
                    if ev.event_type() != EventType::RELATIVE {
                        continue;
                    }
                    let d = ev.value() as f32 * SENS;
                    match RelativeAxisCode(ev.code()) {
                        // Y grows downward; pushing the mouse away is forward.
                        RelativeAxisCode::REL_Y => {
                            self.state.linear = (self.state.linear - d).clamp(-1.0, 1.0)
                        }
                        RelativeAxisCode::REL_X => {
                            self.state.angular = (self.state.angular + d).clamp(-1.0, 1.0)
                        }
                        _ => continue,
                    }
                    return Ok(self.state);
                }
                _ = self.decay.tick() => {
                    // Already stopped: stay quiet rather than waking the caller
                    // 50 times a second with an unchanged state.
                    if self.state == DriveState::default() {
                        continue;
                    }
                    self.state.linear *= self.k;
                    self.state.angular *= self.k;
                    if self.state.linear.abs() < SNAP {
                        self.state.linear = 0.0;
                    }
                    if self.state.angular.abs() < SNAP {
                        self.state.angular = 0.0;
                    }
                    return Ok(self.state);
                }
            }
        }
    }
}
