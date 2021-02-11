use stm32l0xx_hal::prelude::ToggleableOutputPin;
use mcu_helper::tim_cyccnt::U32Ext;
use core::fmt::Write;

#[derive(Debug)]
pub enum Event {
    Toggle,
    OnMode,
    OffMode
}

pub fn blinker(cx: crate::blinker::Context, e: Event, on: &mut bool) {
    match e {
        Event::Toggle => {
            cx.resources.status_led.toggle().ok();
            let schedule_at = cx.scheduled + if *on {
                ms2cycles!(cx.resources.clocks, 150)
            } else {
                ms2cycles!(cx.resources.clocks, 2000)
            };
            cx.schedule.blinker(schedule_at, Event::Toggle).ok();
            writeln!(cx.resources.rtt, ".").ok();
        }
        Event::OnMode => {
            *on = true;
        }
        Event::OffMode => {
            *on = false;
        }
    }
}