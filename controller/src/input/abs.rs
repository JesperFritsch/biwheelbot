//! Absolute axes: gamepad sticks and joysticks.
//!
//! The easy case. The device reports where the stick *is*, so the mapping is a
//! normalise-and-clamp with the driver's own deadzone applied.

use std::io;

use evdev::{AbsInfo, AbsoluteAxisCode, Device, EventStream, EventType};

const DEADZONE: f32 = 0.18;

use super::DriveState;

struct Axis {
    center: f32,
    half: f32,
    flat: f32,
}

impl Axis {
    fn new(i: AbsInfo) -> Self {
        let (min, max) = (i.minimum() as f32, i.maximum() as f32);
        let half = ((max - min) / 2.0).max(1.0);
        Axis {
            center: (max + min) / 2.0,
            half,
            flat: DEADZONE,
        }
    }

    fn norm(&self, raw: i32) -> f32 {
        let v = ((raw as f32 - self.center) / self.half).clamp(-1.0, 1.0);
        let v_abs = v.abs();
        if v_abs < self.flat {
            0.0
        } else {
            ((v_abs - self.flat) / (1.0 - self.flat)).copysign(v)
        }
    }
}

pub struct AbsSource {
    events: EventStream,
    x: Axis,
    y: Axis,
    state: DriveState,
}

impl AbsSource {
    pub fn new(dev: Device) -> io::Result<Self> {
        let mut x = None;
        let mut y = None;
        for (axis, info) in dev.get_absinfo()? {
            match axis {
                AbsoluteAxisCode::ABS_X => x = Some(Axis::new(info)),
                AbsoluteAxisCode::ABS_Y => y = Some(Axis::new(info)),
                _ => {}
            }
        }
        // classify() only hands us devices that advertise both axes.
        let x = x.ok_or_else(|| io::Error::other("ABS_X missing"))?;
        let y = y.ok_or_else(|| io::Error::other("ABS_Y missing"))?;

        Ok(AbsSource {
            events: dev.into_event_stream()?,
            x,
            y,
            state: DriveState::default(),
        })
    }

    pub async fn next(&mut self) -> io::Result<DriveState> {
        loop {
            let ev = self.events.next_event().await?;
            if ev.event_type() != EventType::ABSOLUTE {
                continue;
            }
            match AbsoluteAxisCode(ev.code()) {
                // evdev's Y grows downward, but forward is up.
                AbsoluteAxisCode::ABS_Y => self.state.linear = -self.y.norm(ev.value()),
                AbsoluteAxisCode::ABS_X => self.state.angular = self.x.norm(ev.value()),
                _ => continue,
            }
            return Ok(self.state);
        }
    }
}
