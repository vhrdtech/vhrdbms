use crate::config;
use vhrdcan::FrameId;
use mcu_helper::tim_cyccnt::U32Ext;
use core::fmt::Write;

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

pub fn can_rx(cx: crate::can_rx::Context) {
    let rtt = cx.resources.rtt;
    let can_rx: &mut config::CanRX = cx.resources.can_rx;
    while let Some(frame) = can_rx.heap.pop() {
        writeln!(rtt, "{:?}", frame).ok();
        if frame.id() == config::SOFTOFF_NOTIFY_FRAME_ID {
            cx.spawn.bms_event(crate::tasks::bms::BmsEvent::PowerOff).ok();
        }
    }
}