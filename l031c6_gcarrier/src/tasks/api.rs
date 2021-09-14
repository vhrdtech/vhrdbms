use crate::config;
use mcu_helper::tim_cyccnt::U32Ext;
// use core::fmt::Write;
use crate::tasks::bms::BmsEvent;
use crate::util::EventSource;
use serde::{Serialize};
use uavcan::tail_byte::TransferId;
use vhrdcan::Frame;

#[derive(Debug)]
pub enum Event {
    SendHeartbeat,
    SendPowerOnBurst(PowerBurstContext),
    SendPowerOffBurst(PowerBurstContext),
}

#[derive(Debug, Copy, Clone)]
pub struct PowerBurstContext {
    messages_left: u32
}
impl PowerBurstContext {
    pub fn new_on() -> Self {
        PowerBurstContext {
            messages_left: config::POWER_ON_BURST_DURATION_MS / config::POWER_ON_OFF_BURST_INTERVAL_MS
        }
    }

    pub fn new_off() -> Self {
        PowerBurstContext {
            messages_left: config::POWER_OFF_BURST_DURATION_MS / config::POWER_ON_OFF_BURST_INTERVAL_MS
        }
    }
}

pub fn api(cx: crate::api::Context, e: Event) {
    let clocks = cx.resources.clocks;
    let can_tx: &mut config::CanTX = cx.resources.can_tx;
    let config = config::Config::get();
    match e {
        Event::SendHeartbeat => {
            // let frame = can_tx.pool.new_frame(uavcan_frameid!(config.uavcan_node_id, 0), &[]).unwrap();
            let _ = can_tx.push(Frame::new(uavcan_frameid!(config.uavcan_node_id, 0), &[]).unwrap(), ());
            cx.schedule.api(cx.scheduled + ms2cycles!(clocks, config::HEARTBEAT_INTERVAL_MS), Event::SendHeartbeat).ok();
        }
        Event::SendPowerOnBurst(pcx) => {
            let payload = r#"ON"#;
            let frame = Frame::new(config::POWER_CONTROL_FRAME_ID, payload.as_bytes()).unwrap();
            let _ = can_tx.push(frame, ());
            if pcx.messages_left > 0 {
                let pcx = PowerBurstContext {
                    messages_left: pcx.messages_left - 1
                };
                cx.schedule.api(cx.scheduled + ms2cycles!(clocks, config::POWER_ON_OFF_BURST_INTERVAL_MS), Event::SendPowerOnBurst(pcx)).ok();
            }
        }
        Event::SendPowerOffBurst(pcx) => {
            let payload = r#"OFF"#;
            let frame = Frame::new(config::POWER_CONTROL_FRAME_ID, payload.as_bytes()).unwrap();
            let _ = can_tx.push(frame, ());
            if pcx.messages_left > 0 {
                let pcx = PowerBurstContext {
                    messages_left: pcx.messages_left - 1
                };
                cx.schedule.api(cx.scheduled + ms2cycles!(clocks, config::POWER_ON_OFF_BURST_INTERVAL_MS), Event::SendPowerOffBurst(pcx)).ok();
            }
        }
    }

    rtic::pend(config::CAN_TX_HANDLER);
}

#[derive(Serialize)]
pub struct Telemetry<'a> {
    pub pack_voltage: bq769x0::MilliVolts,
    pub pack_current: bq769x0::MilliAmperes,
    pub cell_voltages: &'a [bq769x0::MilliVolts],
}

static mut TELEM_SEQ_NUMBER: TransferId = TransferId::new_checked(0).unwrap();
pub fn send_telemetry(can_tx: &mut config::CanTX, telemetry: &Telemetry) {
    let mut buf = [0u8; 64];
    let used = postcard::to_slice(telemetry, &mut buf);
    let config = config::Config::get();
    let telem_frame_id = uavcan_frameid!(config.uavcan_node_id, 10);
    if let Ok(payload) = used {
        for (frame_data, frame_len) in uavcan_bridge::to_uavcan(payload, unsafe { TELEM_SEQ_NUMBER }) {
            let frame = Frame::new(telem_frame_id, &frame_data[..frame_len]).unwrap();
            let _ = can_tx.push(frame, ());
        }
    }

    let not_a_soc = telemetry.cell_voltages.iter().min().unwrap();
    let not_a_soc = not_a_soc.0 as i32 - 3300;
    let not_a_soc = not_a_soc * 100 / 800;
    let not_a_soc = if not_a_soc < 0 {
        0
    } else if not_a_soc > 100 {
        100
    } else {
        not_a_soc as u8
    };
    let soc_frame_id = uavcan_frameid!(config.uavcan_node_id, 11);
    for (frame_data, frame_len) in uavcan_bridge::to_uavcan(&[not_a_soc], unsafe { TELEM_SEQ_NUMBER }) {
        let frame = Frame::new(soc_frame_id, &frame_data[..frame_len]).unwrap();
        let _ = can_tx.push(frame, ());
    }

    unsafe { TELEM_SEQ_NUMBER.advance(); }
}

pub fn can_rx(cx: crate::can_rx::Context) {
    // let rtt = cx.resources.rtt;
    let can_rx: &mut config::CanRX = cx.resources.can_rx;
    while let Some((frame, _)) = can_rx.pop() {
        // writeln!(rtt, "{:?}", frame).ok();
        if frame.id == config::SOFTOFF_NOTIFY_FRAME_ID {
            // cx.spawn.softoff(EventSource::RemoteNoForward).ok();
        } else if frame.id == config::POWER_CONTROL_FRAME_ID {
            if frame.data().eq(r#"ON"#.as_bytes()) {
                cx.spawn.bms_event(BmsEvent::PowerOn(EventSource::RemoteNoForward)).ok();
            } else if frame.data().eq(r#"OFF"#.as_bytes()) {
                if config::Config::get().softoff_timeout == 0 {
                    cx.spawn.bms_event(BmsEvent::PowerOff(EventSource::RemoteNoForward)).ok();
                } else {
                    cx.spawn.softoff(crate::tasks::softoff::Event::new_start_soft_off()).ok();
                }
            } else if frame.data().eq(r#"HALT"#.as_bytes()) {
                cx.spawn.bms_event(BmsEvent::Halt).ok();
            } else if frame.data().eq(r#"RESET"#.as_bytes()) {
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    }
}