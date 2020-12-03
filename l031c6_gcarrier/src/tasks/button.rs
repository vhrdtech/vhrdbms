use core::fmt::Write;
use mcu_helper::tim_cyccnt::U32Ext;
use stm32l0xx_hal::exti::{Exti, GpioLine, ExtiLine};
use crate::config;
use stm32l0xx_hal::prelude::InputPin;
use crate::tasks::bms::BmsEvent;

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
                ButtonState::Debounce(config::BUTTON_LONG_PRESS_MS / CHECK_INTERVAL)
            } else {
                ButtonState::Unpressed
            }
        },
        ButtonState::Debounce(checks_remaining) => {
            if *checks_remaining == 0 {
                writeln!(rtt, "LONG_PRESS").ok();
                cx.spawn.bms_event(BmsEvent::TogglePower).ok(); // TODO: bb
                ButtonState::Unpressed
            } else {
                if cx.resources.button.is_low().unwrap() {
                    let _ = cx.schedule.button_event(cx.scheduled + ms2cycles!(cx.resources.clocks, CHECK_INTERVAL));
                    ButtonState::Debounce(*checks_remaining - 1)
                } else {
                    ButtonState::Unpressed
                }
            }
        }
    };
}