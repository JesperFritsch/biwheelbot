//! Wire format for the link to the robot.
//!
//! Pure byte layout, no I/O -- all of it is testable without a robot present.
//! It must stay in step with the firmware's `src/com.cpp`. Note that the
//! packets themselves carry no field identifiers: the only description of what
//! a telemetry or gains payload means is the 0x2901 schema string the firmware
//! attaches to each characteristic, which is why nothing here hardcodes field
//! names.

use uuid::{uuid, Uuid};

pub const DEVICE_NAME: &str = "BiWheelBot";

/// The only service the robot advertises. Empty by design -- it is an identity
/// token, not a data carrier, so the scan filter stays fixed no matter how the
/// services behind it are rearranged.
pub const ID_SVC: Uuid = uuid!("19b1000a-e8f2-537e-4f6c-d104768a1214");

pub const CMD_SVC: Uuid = uuid!("19b10000-e8f2-537e-4f6c-d104768a1214");
pub const DRIVE_CHAR: Uuid = uuid!("19b10001-e8f2-537e-4f6c-d104768a1214");
pub const GAINS_SVC: Uuid = uuid!("19b10002-e8f2-537e-4f6c-d104768a1214");
pub const TELEM_SVC: Uuid = uuid!("19b10006-e8f2-537e-4f6c-d104768a1214");

/// SIG 0x2901, Characteristic User Description.
pub const USER_DESC: Uuid = sig_uuid(0x2901);

const fn sig_uuid(short: u16) -> Uuid {
    Uuid::from_u128(0x00000000_0000_1000_8000_00805f9b34fb | ((short as u128) << 96))
}

/// Bits in the drive packet's `flags` byte. The firmware reserves the byte but
/// does not act on it yet -- see the note in ble.rs.
pub const FLAG_ESTOP: u8 = 1 << 0;
pub const FLAG_LOCK_LEFT: u8 = 1 << 1;
pub const FLAG_LOCK_RIGHT: u8 = 1 << 2;

/// `[linear_i8, angular_i8, flags_u8, seq_u8]`
pub const DRIVE_LEN: usize = 4;

pub fn encode_drive(linear: f32, angular: f32, flags: u8, seq: u8) -> [u8; DRIVE_LEN] {
    [to_i8(linear) as u8, to_i8(angular) as u8, flags, seq]
}

/// -1.0..=1.0 onto the i8 the firmware expects.
///
/// NaN is mapped to zero explicitly. A float-to-int cast in Rust saturates
/// rather than wrapping, but NaN casts to 0 only by convention -- being
/// explicit means a bad axis reading can never become a lurch.
fn to_i8(v: f32) -> i8 {
    if v.is_nan() {
        return 0;
    }
    (v.clamp(-1.0, 1.0) * 127.0).round() as i8
}

/// Parse a 0x2901 descriptor of the form `name:field,field,...`.
///
/// ArduinoBLE pads descriptor reads with NULs, hence the trim.
pub fn parse_schema(s: &str) -> Option<(String, Vec<String>)> {
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

pub fn encode_f32s(v: &[f32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(v.len() * 4);
    for x in v {
        out.extend_from_slice(&x.to_le_bytes());
    }
    out
}

/// Decode a payload of exactly `n` little-endian f32.
///
/// Deliberately permissive about NaN: a telemetry field the firmware has no
/// reading for arrives as NaN and the UI renders it as `--`, so filtering here
/// would turn "no data" into "bad packet". Gain *writes* are where finiteness
/// gets enforced, since that is the direction that can poison a PID block.
pub fn decode_f32s(b: &[u8], n: usize) -> Option<Vec<f32>> {
    if b.len() != n * 4 {
        return None;
    }
    Some(
        b.chunks_exact(4)
            .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect(),
    )
}

/// The 16-bit slice of a 128-bit UUID, for terse diagnostics.
pub fn short(u: &Uuid) -> String {
    u.to_string()[4..8].to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drive_maps_full_scale_to_i8_extremes() {
        assert_eq!(encode_drive(1.0, -1.0, 0, 7), [127, 0x81, 0, 7]);
        assert_eq!(encode_drive(0.0, 0.0, 0, 0)[0], 0);
    }

    #[test]
    fn drive_clamps_and_rejects_nan() {
        assert_eq!(encode_drive(9.0, -9.0, 0, 0)[..2], [127, 0x81]);
        assert_eq!(encode_drive(f32::NAN, f32::NAN, 0, 0)[..2], [0, 0]);
    }

    #[test]
    fn drive_flags_and_seq_pass_through() {
        let p = encode_drive(0.0, 0.0, FLAG_ESTOP | FLAG_LOCK_LEFT, 200);
        assert_eq!(p[2], 0b011);
        assert_eq!(p[3], 200);
    }

    #[test]
    fn f32s_round_trip_at_any_width() {
        for v in [vec![1.5, -0.25, 3.0], vec![0.5], vec![1.0, 2.0, 3.0, 4.0]] {
            assert_eq!(decode_f32s(&encode_f32s(&v), v.len()), Some(v));
        }
    }

    #[test]
    fn f32s_preserve_nan_for_telemetry() {
        let raw = encode_f32s(&[f32::NAN, 1.0]);
        let out = decode_f32s(&raw, 2).unwrap();
        assert!(out[0].is_nan());
        assert_eq!(out[1], 1.0);
    }

    #[test]
    fn schema_parses_firmware_strings() {
        let (name, fields) = parse_schema("balance:kp,ki,kd\0\0").unwrap();
        assert_eq!(name, "balance");
        assert_eq!(fields, ["kp", "ki", "kd"]);

        let (name, fields) = parse_schema(
            "telem:ang,rate,kfa,cmp,tpos,pos,tang,tspd,eff,spd,d_a,d_b,t_d,bat,en,ovr",
        )
        .unwrap();
        assert_eq!(name, "telem");
        assert_eq!(fields.len(), 16);
    }

    #[test]
    fn schema_rejects_malformed() {
        assert_eq!(parse_schema("no-colon"), None);
        assert_eq!(parse_schema(":kp,ki"), None);
        assert_eq!(parse_schema("empty:"), None);
    }

    #[test]
    fn f32s_require_exact_length() {
        assert_eq!(decode_f32s(&1.0f32.to_le_bytes(), 1), Some(vec![1.0]));
        assert_eq!(decode_f32s(&[0; 5], 1), None);
    }

    #[test]
    fn user_desc_is_the_sig_uuid() {
        assert_eq!(USER_DESC.to_string(), "00002901-0000-1000-8000-00805f9b34fb");
    }
}
