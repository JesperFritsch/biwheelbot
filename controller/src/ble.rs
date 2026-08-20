//! Link to the robot.
//!
//! Split on the line that matters: `proto` is the wire format and nothing else
//! -- pure byte layout, no I/O, unit-testable without a robot, and the one
//! place that has to stay in step with the firmware's `src/com.cpp`. `com` is
//! the transport: scanning, connecting, and moving those bytes over btleplug.

pub mod com;
pub mod proto;

// Just the handful the binary reaches for constantly. Everything else stays
// addressable at `ble::com::…` / `ble::proto::…` rather than being re-exported
// into a facade that has to be maintained alongside it.
pub use com::{connect, Robot};
pub use proto::DEVICE_NAME;
