use core::fmt::Write;
use mcu_helper::tim_cyccnt::U32Ext;
use stm32l0xx_hal::exti::{Exti, GpioLine, ExtiLine};
use crate::config;
use stm32l0xx_hal::prelude::InputPin;
use crate::tasks::bms::{BmsEvent, PowerRailCommand};
use crate::power_block::PowerBlockId;
use stm32l0xx_hal::time::MicroSeconds;

#[derive(PartialEq, Eq, Debug)]
pub enum ButtonState {
    Unpressed,
    Pressed,
    Debounce(u32)
}
impl Default for ButtonState {
    fn default() -> Self {
        ButtonState::Unpressed
    }
}

pub fn button_irq(cx: crate::button_irq::Context) {
    Exti::unpend(GpioLine::from_raw_line(cx.resources.button.pin_number()).unwrap());
    match cx.resources.button_state {
        ButtonState::Unpressed => {
            *cx.resources.button_state = ButtonState::Pressed;
            cx.spawn.button_event().ok();
        },
        ButtonState::Pressed => {},
        ButtonState::Debounce(_) => {}
    }
}

pub fn button_event(cx: crate::button_event::Context) {
    const CHECK_INTERVAL: u32 = 100;
    let rtt = cx.resources.rtt;

    *cx.resources.button_state = match cx.resources.button_state {
        ButtonState::Unpressed => {
            ButtonState::Unpressed
        },
        ButtonState::Pressed => {
            if cx.resources.button.is_low().unwrap() {
                let _ = cx.schedule.button_event(cx.scheduled + ms2cycles!(cx.resources.clocks, CHECK_INTERVAL));
                ButtonState::Debounce(0)
            } else {
                ButtonState::Unpressed
            }
        },
        ButtonState::Debounce(checks_done) => {
            if *checks_done >= config::BUTTON_LONG_PRESS_MS / CHECK_INTERVAL {
                writeln!(rtt, "LONG_PRESS").ok();
                cx.spawn.bms_event(BmsEvent::TogglePower).ok(); // TODO: bb
                ButtonState::Unpressed
            } else  {
                if cx.resources.button.is_low().unwrap() {
                    cx.schedule.button_event(cx.scheduled + ms2cycles!(cx.resources.clocks, CHECK_INTERVAL)).ok(); // TODO: bb;
                    ButtonState::Debounce(*checks_done + 1)
                } else if *checks_done >= config::BUTTON_SHORT_PRESS_MS / CHECK_INTERVAL {
                    writeln!(rtt, "SHORT_PRESS").ok(); // TODO: bb
                    cx.spawn.bms_event(BmsEvent::PowerRailControl(PowerRailCommand{
                        rail: PowerBlockId::Switch3V3Tms,
                        powered: false,
                        delay_after: Some(MicroSeconds(300_000))
                    })).ok();
                    cx.spawn.bms_event(BmsEvent::PowerRailControl(PowerRailCommand{
                        rail: PowerBlockId::Switch3V3Tms,
                        powered: true,
                        delay_after: None
                    })).ok();
                    ButtonState::Unpressed
                } else {
                    ButtonState::Unpressed
                }
            }
        }
    };
}