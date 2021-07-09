use crate::config;
use crate::tasks::bms::BmsEvent;
use mcu_helper::tim_cyccnt::U32Ext;
use crate::util::EventSource;

pub enum State {
    Initial,
    TurnOffInProgress(u32)
}
impl State {
    pub const fn new() -> Self {
        State::Initial
    }
}

#[derive(Eq, PartialEq)]
pub struct Event {
    e: InternalEvent
}
impl Event {
    pub fn new_start_soft_off() -> Self {
        Event {
            e: InternalEvent::StartSoftOffProcess
        }
    }

    fn new_soft_off_in_progress() -> Self {
        Event {
            e: InternalEvent::SoftOffInProgress
        }
    }
}
#[derive(Eq, PartialEq)]
enum InternalEvent {
    StartSoftOffProcess,
    SoftOffInProgress
}

fn send_notify(can_tx: &mut config::CanTX) {
    let frame = can_tx.pool.new_frame(config::SOFTOFF_NOTIFY_FRAME_ID, r#"SOFTOFF"#.as_bytes()).unwrap();
    let _ = can_tx.heap.push(frame);
    rtic::pend(config::CAN_TX_HANDLER);
}

pub fn softoff(cx: crate::softoff::Context, e: Event, state: &mut State) {
    let can_tx: &mut config::CanTX = cx.resources.can_tx;
    *state = match *state {
        State::Initial => {
            if config::Config::get().softoff_timeout == 0 {
                cx.spawn.bms_event(BmsEvent::PowerOff(EventSource::LocalNoForward)).ok(); // TODO: bb
                State::Initial
            } else {
                cx.schedule.softoff(cx.scheduled + ms2cycles!(cx.resources.clocks, config::SOFTOFF_NOTIFY_INTERVAL_MS), Event::new_soft_off_in_progress()).ok(); // TODO: bb;
                State::TurnOffInProgress(0)
            }
        },
        State::TurnOffInProgress(notifications_sent) => {
            if e.e == InternalEvent::StartSoftOffProcess {
                return;
            }
            if config::Config::get().softoff_timeout / config::SOFTOFF_NOTIFY_INTERVAL_MS == notifications_sent {
                cx.spawn.bms_event(BmsEvent::PowerOff(EventSource::LocalForward)).ok(); // TODO: bb
                State::Initial
            } else {
                cx.schedule.softoff(cx.scheduled + ms2cycles!(cx.resources.clocks, config::SOFTOFF_NOTIFY_INTERVAL_MS), Event::new_soft_off_in_progress()).ok(); // TODO: bb;
                send_notify(can_tx);
                State::TurnOffInProgress(notifications_sent + 1)
            }
        }
    };
}