use crate::config;
use crate::tasks::bms::BmsEvent;
use mcu_helper::tim_cyccnt::U32Ext;
use crate::util::EventSource;

pub enum State {
    Initial,
    TurnOffInProgress(u32)
}
impl Default for State {
    fn default() -> Self {
        State::Initial
    }
}

fn send_notify(can_tx: &mut config::CanTX) {
    let frame = can_tx.pool.new_frame(config::SOFTOFF_NOTIFY_FRAME_ID, r#"SOFTOFF"#.as_bytes()).unwrap();
    let _ = can_tx.heap.push(frame);
    rtic::pend(config::CAN_TX_HANDLER);
}

pub fn softoff(cx: crate::softoff::Context) {
    let can_tx: &mut config::CanTX = cx.resources.can_tx;
    *cx.resources.softoff_state = match *cx.resources.softoff_state {
        State::Initial => {
            if config::SOFTOFF_TIMEOUT_MS == 0 {
                cx.spawn.bms_event(BmsEvent::PowerOff(EventSource::LocalNoForward)).ok(); // TODO: bb
                State::Initial
            } else {
                cx.schedule.softoff(cx.scheduled + ms2cycles!(cx.resources.clocks, config::SOFTOFF_NOTIFY_INTERVAL_MS)).ok(); // TODO: bb;
                State::TurnOffInProgress(0)
            }
        },
        State::TurnOffInProgress(notifications_sent) => {
            if config::SOFTOFF_TIMEOUT_MS / config::SOFTOFF_NOTIFY_INTERVAL_MS == notifications_sent {
                cx.spawn.bms_event(BmsEvent::PowerOff(EventSource::LocalForward)).ok(); // TODO: bb
                State::Initial
            } else {
                cx.schedule.softoff(cx.scheduled + ms2cycles!(cx.resources.clocks, config::SOFTOFF_NOTIFY_INTERVAL_MS)).ok(); // TODO: bb;
                send_notify(can_tx);
                State::TurnOffInProgress(notifications_sent + 1)
            }
        }
    };
}