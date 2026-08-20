//! Ground station for BiWheelBot.
//!
//! Four parts, wired together here and nowhere else:
//!
//!   input    reads an evdev device, publishes DriveState on a watch channel
//!   ble      finds the robot, moves bytes, knows the wire format
//!   control  owns the 50 Hz loop; input in, packets out, telemetry back,
//!            all of it republished as one State
//!   ui       draws that State and edits gains; a reader, never a participant
//!
//! `input` and `ble` do not know about each other. `control` joins them. The UI
//! can be slow, resizing, or absent without costing a setpoint.
//!
//!     cargo run

use std::sync::Arc;

use anyhow::Result;

mod ble;
mod control;
mod input;
mod ui;

#[tokio::main]
async fn main() -> Result<()> {
    let devices = input::enumerate();
    if devices.is_empty() {
        anyhow::bail!("no usable input devices -- are you in the `input` group?");
    }

    let mut screen = ui::Screen::open()?;

    let Some(pick) = screen.pick(&devices).await? else {
        return Ok(());
    };
    let info = &devices[pick];

    // Opened only after the choice is made, so the picker's arrow keys can
    // never be stolen by an evdev device we already hold.
    let input = input::open(info)?;

    let robot = Arc::new(
        screen.working(
            &format!("scanning for {}…", ble::DEVICE_NAME),
            ble::connect(),
        )
        .await?,
    );

    let handle = control::spawn(robot.clone(), input).await;
    let result = screen.run(handle, info.name.clone()).await;

    // Restore the terminal before talking to the radio again: disconnecting can
    // block, and BlueZ otherwise holds the link open after we exit.
    drop(screen);
    robot.disconnect().await.ok();
    result
}
