use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter, WriteType};
use btleplug::platform::Manager;
use std::error::Error;
use std::time::Duration;
use tokio::time;
use uuid::Uuid;

// Must match the Arduino side exactly.
const CMD_CHAR_UUID: Uuid = Uuid::from_u128(0x19b10001_e8f2_537e_4f6c_d104768a1214);
const DEVICE_NAME: &str = "BiWheelBot";

// Pack command into 4 bytes: [linear_i8, angular_i8, flags_u8, seq_u8]
fn encode(linear: i8, angular: i8, flags: u8, seq: u8) -> [u8; 4] {
    [linear as u8, angular as u8, flags, seq]
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let manager = Manager::new().await?;
    let adapter = manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or("no BLE adapter found")?;


    let _ = adapter.stop_scan().await;
    println!("scanning...");
    adapter.start_scan(ScanFilter::default()).await?;
    time::sleep(Duration::from_secs(3)).await;
    println!("Scanning done!");
    // Find our robot by advertised name.
    let robot = {
        let mut found = None;
        for p in adapter.peripherals().await? {
            if let Some(props) = p.properties().await? {
                if props.local_name.as_deref() == Some(DEVICE_NAME) {
                    found = Some(p);
                    break;
                }
            }
        }
        found.ok_or(format!("{} not found", DEVICE_NAME))?
    };

    adapter.stop_scan().await?;
    println!("connecting...");
    robot.connect().await?;
    robot.discover_services().await?;

    // Locate the command characteristic.
    let cmd_char = robot
        .characteristics()
        .into_iter()
        .find(|c| c.uuid == CMD_CHAR_UUID)
        .ok_or("command characteristic not found")?;

    println!("connected. streaming commands.");

    // Stream setpoints at 50 Hz (20 ms). Each packet carries the full target,
    // so a dropped one just means the robot holds the last setpoint.
    let mut seq: u8 = 0;
    let mut interval = time::interval(Duration::from_millis(20));
    loop {
        // Example: drive forward, no turn. Replace with joystick/keyboard input.
        let packet = encode(40, 0, 0, seq);
        seq = seq.wrapping_add(1);

        robot
            .write(&cmd_char, &packet, WriteType::WithoutResponse)
            .await?;

        interval.tick().await;
    }
}