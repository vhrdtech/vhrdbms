use crate::config;
use vhrdcan::FrameId;
use mcu_helper::tim_cyccnt::U32Ext;
use core::fmt::Write;
use crate::tasks::bms::BmsEvent;
use crate::util::EventSource;
use serde::{Serialize};
use uavcan::session_id::{TransferPriority};
use core::convert::TryInto;

#[derive(Debug)]
pub enum Event {
    SendHeartbeat,
    SendPowerOnBurst(PowerBurstContext),

}

#[derive(Debug)]
pub struct PowerBurstContext {
    messages_left: u32
}
impl PowerBurstContext {
    pub fn new() -> Self {
        PowerBurstContext {
            messages_left: config::POWER_ON_BURST_DURATION_MS / config::POWER_ON_BURST_INTERVAL_MS
        }
    }
}

pub fn api(cx: crate::api::Context, e: Event) {
    let clocks = cx.resources.clocks;
    let can_tx: &mut config::CanTX = cx.resources.can_tx;
    match e {
        Event::SendHeartbeat => {
            let frame = can_tx.pool.new_frame(FrameId::new_extended(uavcan::session_id::message::MessageSessionId::as_u32(50.try_into().unwrap(), 0.try_into().unwrap(), TransferPriority::Optional)).unwrap(), &[]).unwrap();
            let _ = can_tx.heap.push(frame);
            cx.schedule.api(cx.scheduled + ms2cycles!(clocks, config::HEARTBEAT_INTERVAL_MS), Event::SendHeartbeat).ok();
        }
        Event::SendPowerOnBurst(pcx) => {
            let payload = r#"ON"#;
            let frame = can_tx.pool.new_frame(config::POWER_CONTROL_FRAME_ID, payload.as_bytes()).unwrap();
            let _ = can_tx.heap.push(frame);
            if pcx.messages_left > 0 {
                let pcx = PowerBurstContext {
                    messages_left: pcx.messages_left - 1
                };
                cx.schedule.api(cx.scheduled + ms2cycles!(clocks, config::POWER_ON_BURST_INTERVAL_MS), Event::SendPowerOnBurst(pcx)).ok();
            }
        }
    }

    rtic::pend(config::CAN_TX_HANDLER);
}

pub fn broadcast_power_control_frame(can_tx: &mut config::CanTX, on: bool) {
    let payload = if on { r#"ON"# } else { r#"OFF"# };
    let frame = can_tx.pool.new_frame(config::POWER_CONTROL_FRAME_ID, payload.as_bytes()).unwrap();
    let _ = can_tx.heap.push(frame);
    rtic::pend(config::CAN_TX_HANDLER);
}

#[derive(Serialize)]
pub struct Telemetry<'a> {
    pub pack_voltage: bq769x0::MilliVolts,
    pub pack_current: bq769x0::MilliAmperes,
    pub cell_voltages: &'a [bq769x0::MilliVolts],
}

pub fn send_telemetry(can_tx: &mut config::CanTX, telemetry: &Telemetry) {
    let mut buf = [0u8; 64];
    let used = postcard::to_slice(telemetry, &mut buf);
    if let Ok(payload) = used {
        for (frame_data, frame_len) in uavcan_bridge::to_uavcan(payload) {
            let frame = can_tx.pool.new_frame(FrameId::new_extended(uavcan::session_id::message::MessageSessionId::as_u32(50.try_into().unwrap(), 10.try_into().unwrap(), TransferPriority::Optional)).unwrap(), &frame_data[..frame_len]).unwrap();
            let _ = can_tx.heap.push(frame);
        }
    }
}

pub fn can_rx(cx: crate::can_rx::Context) {
    let rtt = cx.resources.rtt;
    let can_rx: &mut config::CanRX = cx.resources.can_rx;
    while let Some(frame) = can_rx.heap.pop() {
        writeln!(rtt, "{:?}", frame).ok();
        if frame.id() == config::SOFTOFF_NOTIFY_FRAME_ID {
            // cx.spawn.softoff(EventSource::RemoteNoForward).ok();
        } else if frame.id() == config::POWER_CONTROL_FRAME_ID {
            if frame.data().eq(r#"ON"#.as_bytes()) {
                cx.spawn.bms_event(BmsEvent::PowerOn(EventSource::RemoteNoForward)).ok();
            } else if frame.data().eq(r#"OFF"#.as_bytes()) {
                if config::SOFTOFF_TIMEOUT_MS == 0 {
                    cx.spawn.bms_event(BmsEvent::PowerOff(EventSource::RemoteNoForward)).ok();
                } else {
                    cx.spawn.softoff().ok();
                }
            }
        }
    }
}