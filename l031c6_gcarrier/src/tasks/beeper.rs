use stm32l0xx_hal::pwm::{Pwm, C1, Assigned};
use stm32l0xx_hal::gpio::gpioa::PA0;
use stm32l0xx_hal::gpio::Analog;
use stm32l0xx_hal::pac::TIM2;
use embedded_time::rate::Hertz;
use embedded_time::duration::Milliseconds;
use stm32l0xx_hal::prelude::OutputPin;
use stm32l0xx_hal::prelude::_embedded_hal_PwmPin;
use mcu_helper::tim_cyccnt::U32Ext;

pub type BuzzerPwmChannel = Pwm<TIM2, C1, Assigned<PA0<Analog>>>;

#[derive(Copy, Clone)]
pub enum Event {
    BeepTone(Hertz, Milliseconds),
    BeepTwoTones(Hertz, Milliseconds, Hertz, Milliseconds),
    _Internal,
}

pub struct State {
    last_event: Option<Event>,
    step: u8,
}

impl State {
    pub const fn new() -> Self {
        State {
            last_event: None,
            step: 0
        }
    }
}

pub fn beeper(cx: crate::beeper::Context, e: Event, state: &mut State) {
    let buzzer_pwm_channel: &mut BuzzerPwmChannel = cx.resources.buzzer_pwm_channel;

    match e {
        Event::BeepTone(_, duration) => {
            match state.last_event {
                None => {}
                Some(_) => { return; }
            }
            state.last_event = Some(e);
            state.step = 0;
            // cx.resources.afe_io.lock(|afe_io: &mut AfeIo| afe_io.switch_5v0_aux_en.set_high().ok());
            cx.resources.afe_io.switch_5v0_aux_en.set_high().ok();
            buzzer_pwm_channel.set_duty(buzzer_pwm_channel.get_max_duty() / 2);
            // buzzer_pwm_channel.timer.set_
            buzzer_pwm_channel.enable();
            cx.schedule.beeper(cx.scheduled + ms2cycles!(cx.resources.clocks, duration.0), Event::_Internal).ok();
        }
        Event::BeepTwoTones(_, _, _, _) => {

        },
        Event::_Internal => {
            match state.last_event {
                None => { return; }
                Some(last_event) => {
                    match last_event {
                        Event::BeepTone(_, _) => {
                            // cx.resources.afe_io.lock(|afe_io: &mut AfeIo| afe_io.switch_5v0_aux_en.set_low().ok());
                            cx.resources.afe_io.switch_5v0_aux_en.set_low().ok();
                            buzzer_pwm_channel.disable();
                            state.last_event = None;
                        }
                        Event::BeepTwoTones(_, _, _, _) => {}
                        Event::_Internal => {}
                    }
                }
            }
        }
    }
}