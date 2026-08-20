//! Transport: find the robot, hold its characteristics, move bytes.
//!
//! Everything about *what* the bytes mean lives in `proto`; this file only
//! knows how to get them across.

use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use btleplug::api::{
    Central, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::{Manager, Peripheral};
use futures::StreamExt;
use tokio::sync::mpsc::{self, UnboundedReceiver};
use tokio::time::{sleep, Instant};

use super::proto;

pub const SCAN_TIMEOUT: Duration = Duration::from_secs(15);

pub struct GainBlock {
    pub name: String,
    pub fields: Vec<String>,
    ch: Characteristic,
}

pub struct Robot {
    p: Peripheral,
    drive: Characteristic,
    telem: Option<Characteristic>,
    pub gains: Vec<GainBlock>,
    /// Characteristics in the gains service we could not make sense of. Kept
    /// rather than dropped so a firmware missing a descriptor says so instead
    /// of silently vanishing from the list.
    pub skipped: Vec<String>,
}

/// Scan for the robot by advertised name, connect, and discover everything.
pub async fn connect() -> Result<Robot> {
    let manager = Manager::new().await.context("creating BLE manager")?;
    let central = manager
        .adapters()
        .await
        .context("listing adapters")?
        .into_iter()
        .next()
        .ok_or_else(|| anyhow!("no Bluetooth adapter found"))?;

    // A scan left running from a previous crashed run blocks a fresh one.
    central.stop_scan().await.ok();
    // BlueZ applies this itself, so anything not advertising idService never
    // reaches us. That is the point: no name string to match, no waiting on a
    // scan response, and the robot is identified from the first advertising
    // packet it sends. Firmware older than idService will not be found.
    central
        .start_scan(ScanFilter {
            services: vec![proto::ID_SVC],
        })
        .await
        .context("starting scan")?;

    let deadline = Instant::now() + SCAN_TIMEOUT;
    let p = 'found: loop {
        for p in central.peripherals().await.context("listing peripherals")? {
            let Ok(Some(props)) = p.properties().await else {
                continue;
            };
            // The filter is a request, not a guarantee -- BlueZ also reports
            // devices it already has cached, whatever the current filter says.
            // So confirm the match ourselves rather than trusting the first
            // peripheral handed back.
            if props.services.contains(&proto::ID_SVC) {
                central.stop_scan().await.ok();
                p.connect().await.context("connecting to peripheral")?;
                p.discover_services()
                    .await
                    .context("discovering services")?;
                break 'found p;
            }
        }
        if Instant::now() > deadline {
            central.stop_scan().await.ok();
            return Err(anyhow!(
                "{} not found in {}s -- is it powered, not already connected, \
                 and running firmware that advertises {}?",
                proto::DEVICE_NAME,
                SCAN_TIMEOUT.as_secs(),
                proto::ID_SVC
            ));
        }
        sleep(Duration::from_millis(300)).await;
    };

    Robot::from_peripheral(p).await
}

impl Robot {
    async fn from_peripheral(p: Peripheral) -> Result<Self> {
        let chars = p.characteristics();

        let drive = chars
            .iter()
            .find(|c| c.uuid == proto::DRIVE_CHAR)
            .cloned()
            .ok_or_else(|| anyhow!("no drive characteristic {}", proto::DRIVE_CHAR))?;

        let telem = chars
            .iter()
            .find(|c| c.service_uuid == proto::TELEM_SVC)
            .cloned();

        let mut blocks: Vec<Characteristic> = chars
            .iter()
            .filter(|c| c.service_uuid == proto::GAINS_SVC)
            .cloned()
            .collect();
        // BTreeSet order is already by UUID, but the display order is
        // user-facing so make it explicit.
        blocks.sort_by_key(|c| c.uuid);

        let mut gains = Vec::new();
        let mut skipped = Vec::new();
        for ch in blocks {
            match read_schema(&p, &ch).await {
                Ok((name, fields)) => gains.push(GainBlock { name, fields, ch }),
                Err(e) => skipped.push(format!("{}: {e}", proto::short(&ch.uuid))),
            }
        }

        Ok(Robot { p, drive, telem, gains, skipped })
    }

    pub async fn send_drive(&self, linear: f32, angular: f32, flags: u8, seq: u8) -> Result<()> {
        let pkt = proto::encode_drive(linear, angular, flags, seq);
        self.p
            .write(&self.drive, &pkt, WriteType::WithoutResponse)
            .await
            .context("drive write failed")?;
        Ok(())
    }

    /// Sized by the block's own schema, not by a hardcoded field count -- a
    /// firmware that grows a fourth gain term needs no change here.
    pub async fn read_gains(&self, i: usize) -> Result<Vec<f32>> {
        let b = self.gains.get(i).ok_or_else(|| anyhow!("no gain block {i}"))?;
        let raw = self.p.read(&b.ch).await?;
        proto::decode_f32s(&raw, b.fields.len()).ok_or_else(|| {
            anyhow!(
                "{}: schema says {} fields ({} bytes), got {}",
                b.name,
                b.fields.len(),
                b.fields.len() * 4,
                raw.len()
            )
        })
    }

    /// Written *with* response: a dropped gain write would leave the UI showing
    /// a value the robot never adopted.
    pub async fn write_gains(&self, i: usize, values: &[f32]) -> Result<()> {
        let b = self.gains.get(i).ok_or_else(|| anyhow!("no gain block {i}"))?;
        if values.len() != b.fields.len() {
            return Err(anyhow!(
                "{}: schema says {} fields, got {}",
                b.name,
                b.fields.len(),
                values.len()
            ));
        }
        // decode_gains() in com.cpp rejects these too, but a NaN that reaches
        // the wire is a NaN in a PID block if that check is ever relaxed.
        if let Some(bad) = values.iter().find(|v| !v.is_finite()) {
            return Err(anyhow!("{}: refusing to write {bad}", b.name));
        }
        self.p
            .write(&b.ch, &proto::encode_f32s(values), WriteType::WithResponse)
            .await
            .with_context(|| format!("writing gains for {}", b.name))?;
        Ok(())
    }

    /// Subscribe to telemetry notifications.
    ///
    /// Returns the field names from the 0x2901 schema plus a receiver fed by a
    /// background task, so a draw loop can drain packets without awaiting the
    /// notification stream itself. Payloads that do not match the schema's
    /// field count are dropped rather than shown misaligned.
    pub async fn subscribe_telemetry(&self) -> Result<(Vec<String>, UnboundedReceiver<Vec<f32>>)> {
        let ch = self
            .telem
            .as_ref()
            .ok_or_else(|| anyhow!("no characteristic in telemetry service {}", proto::TELEM_SVC))?;

        let (_, fields) = read_schema(&self.p, ch)
            .await
            .context("telemetry characteristic has no usable 0x2901 schema")?;

        self.p.subscribe(ch).await?;
        let mut stream = self.p.notifications().await?;

        let uuid = ch.uuid;
        let n = fields.len();
        let (tx, rx) = mpsc::unbounded_channel();
        tokio::spawn(async move {
            while let Some(note) = stream.next().await {
                if note.uuid != uuid {
                    continue;
                }
                let Some(values) = proto::decode_f32s(&note.value, n) else {
                    continue;
                };
                if tx.send(values).is_err() {
                    break; // receiver dropped -- caller is shutting down
                }
            }
        });

        Ok((fields, rx))
    }

    pub async fn disconnect(&self) -> Result<()> {
        self.p.disconnect().await?;
        Ok(())
    }
}

/// Read a characteristic's 0x2901 descriptor and parse it as `name:f1,f2,...`.
///
/// BlueZ issues Read Blob requests for values longer than one MTU, so a schema
/// bigger than a single response still arrives whole.
async fn read_schema(p: &Peripheral, ch: &Characteristic) -> Result<(String, Vec<String>)> {
    let desc = ch
        .descriptors
        .iter()
        .find(|d| d.uuid == proto::USER_DESC)
        .ok_or_else(|| anyhow!("no 0x2901 descriptor"))?;
    let raw = p.read_descriptor(desc).await.context("descriptor unreadable")?;
    let text = String::from_utf8_lossy(&raw).into_owned();
    proto::parse_schema(&text).ok_or_else(|| anyhow!("bad schema {text:?}"))
}
