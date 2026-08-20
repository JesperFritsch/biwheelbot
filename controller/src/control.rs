//! The loop that actually flies the robot.
//!
//! Owns the only thing with a hard timing requirement: read the latest input,
//! put a setpoint on the air at 50 Hz, fold telemetry back in, publish the
//! result. It runs in its own task and does not care whether anything is
//! watching -- the UI is a reader of `State`, never a participant, so the robot
//! keeps flying at the same rate whether the terminal is busy, resizing, or
//! absent entirely.
//!
//! Gains deliberately do not travel through here. They are rare, they are
//! request/response, and putting a 40 ms write-with-response inside the tick
//! would cost setpoints for no reason -- so `Handle` reaches the robot directly
//! for those.

use std::sync::Arc;
use std::time::Duration;

use anyhow::Result;
use tokio::sync::watch;
use tokio::time::{self, MissedTickBehavior};

use crate::ble::{com::GainBlock, Robot};
use crate::input::DriveState;

/// Setpoint rate. Each packet carries the full target, so a dropped one just
/// means the robot holds the last.
const TICK: Duration = Duration::from_millis(20);

/// A stalled radio must not freeze the published state. One tick's worth of
/// grace, then the write is abandoned and the link is called down.
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);

/// Everything a reader needs, republished every tick.
#[derive(Clone, Debug)]
pub struct State {
    /// What we intend to send -- pre-quantisation, which is what a human wants
    /// to see. The wire bytes are a troubleshooting concern, not a UI one.
    pub drive: DriveState,
    /// Latest telemetry packet, NaN-filled until the first one lands.
    pub telem: Vec<f32>,
    pub telem_packets: u64,
    /// False once a write fails or times out. Not diagnostics -- without it a
    /// dropped link looks exactly like a working one that nobody is driving.
    pub connected: bool,
    /// The input device stopped producing. The setpoint is pinned to zero and
    /// will stay there.
    pub input_dead: bool,
}

pub struct Handle {
    state: watch::Receiver<State>,
    robot: Arc<Robot>,
    telem_fields: Vec<String>,
}

impl Handle {
    /// The latest published state. Cheap enough to call every frame.
    pub fn state(&mut self) -> State {
        self.state.borrow_and_update().clone()
    }

    /// Names for `State::telem`, from the robot's 0x2901 schema. Empty if the
    /// robot has no telemetry service.
    pub fn telem_fields(&self) -> &[String] {
        &self.telem_fields
    }

    /// Discovered gain blocks -- names and field schemas, fixed at connect.
    pub fn gains(&self) -> &[GainBlock] {
        &self.robot.gains
    }

    /// Straight to the robot, off the control loop's task, so a gain round trip
    /// never delays a setpoint.
    pub async fn read_gains(&self, i: usize) -> Result<Vec<f32>> {
        self.robot.read_gains(i).await
    }

    pub async fn write_gains(&self, i: usize, values: &[f32]) -> Result<()> {
        self.robot.write_gains(i, values).await
    }
}

/// Subscribe to telemetry, start the loop, hand back a reader.
pub async fn spawn(robot: Arc<Robot>, input: watch::Receiver<DriveState>) -> Handle {
    // A robot with no telemetry is still drivable, so this degrades rather
    // than fails.
    let (telem_fields, telem_rx) = match robot.subscribe_telemetry().await {
        Ok((fields, rx)) => (fields, Some(rx)),
        Err(_) => (Vec::new(), None),
    };

    let (tx, rx) = watch::channel(State {
        drive: DriveState::default(),
        telem: vec![f32::NAN; telem_fields.len()],
        telem_packets: 0,
        connected: true,
        input_dead: false,
    });

    let driver = robot.clone();
    tokio::spawn(async move { run(driver, input, telem_rx, tx).await });

    Handle {
        state: rx,
        robot,
        telem_fields,
    }
}

async fn run(
    robot: Arc<Robot>,
    mut input: watch::Receiver<DriveState>,
    mut telem_rx: Option<tokio::sync::mpsc::UnboundedReceiver<Vec<f32>>>,
    tx: watch::Sender<State>,
) {
    let mut tick = time::interval(TICK);
    // A 1 kHz mouse can leave the tick behind; catching up in a burst would put
    // a backlog of stale setpoints on the air.
    tick.set_missed_tick_behavior(MissedTickBehavior::Delay);

    let mut state = tx.borrow().clone();
    let mut seq: u8 = 0;

    loop {
        tokio::select! {
            _ = tick.tick() => {
                if !state.input_dead {
                    // Err means the sender was dropped: the device is gone.
                    // Receivers would otherwise hand back the last value
                    // forever, which is exactly the setpoint we must not keep.
                    if input.has_changed().is_err() {
                        state.input_dead = true;
                        state.drive = DriveState::default();
                    } else {
                        state.drive = *input.borrow_and_update();
                    }
                }

                // Zeros keep going out after the input dies -- an active stop
                // rather than silence, so the robot does not have to wait for
                // its own link timeout to notice.
                let write = robot.send_drive(state.drive.linear, state.drive.angular, 0, seq);
                state.connected = matches!(time::timeout(WRITE_TIMEOUT, write).await, Ok(Ok(())));
                seq = seq.wrapping_add(1);

                // Never fails, unlike send(): the loop outlives its readers.
                tx.send_replace(state.clone());
            }

            Some(values) = async {
                match telem_rx.as_mut() {
                    Some(rx) => rx.recv().await,
                    None => None,
                }
            } => {
                // Folded in here, published on the next tick 20 ms later. No
                // reader redraws faster than that, and counting here is the
                // only way to count at all -- a watch drops intermediates.
                state.telem = values;
                state.telem_packets += 1;
            }
        }
    }
}
