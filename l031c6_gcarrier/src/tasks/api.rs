use crate::config;
use vhrdcan::FrameId;
use mcu_helper::tim_cyccnt::U32Ext;
use core::fmt::Write;
use crate::tasks::bms::BmsEvent;
use crate::util::EventSource;

#[derive(Debug)]
pub enum Event {
    SendHeartbeat,

}

pub fn api(cx: crate::api::Context, _e: Event) {
    let clocks = cx.resources.clocks;

    let can_tx: &mut config::CanTX = cx.resources.can_tx;
    let frame = can_tx.pool.new_frame(FrameId::new_extended(0x1234567).unwrap(), &[0]).unwrap();
    let _ = can_tx.heap.push(frame);
    let frame = can_tx.pool.new_frame(FrameId::new_extended(0x15555555).unwrap(), &[0]).unwrap();
    let _ = can_tx.heap.push(frame);

    cx.schedule.api(cx.scheduled + ms2cycles!(clocks, config::HEARTBEAT_INTERVAL_MS), Event::SendHeartbeat).ok();
    rtic::pend(config::CAN_TX_HANDLER);
}

pub fn broadcast_power_control_frame(can_tx: &mut config::CanTX, on: bool) {
    let payload = if on { r#"ON"# } else { r#"OFF"# };
    let frame = can_tx.pool.new_frame(config::POWER_CONTROL_FRAME_ID, payload.as_bytes()).unwrap();
    let _ = can_tx.heap.push(frame);
    rtic::pend(config::CAN_TX_HANDLER);
}

pub fn send_telemetry(can_tx: &mut config::CanTX) {
    let data = [0u8; 128];
    for (frame_data, frame_len) in uavcan_bridge::to_uavcan(&data) {
        let frame = can_tx.pool.new_frame(FrameId::new_extended(0x010203).unwrap(), &frame_data[..frame_len]).unwrap();
        let _ = can_tx.heap.push(frame);
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