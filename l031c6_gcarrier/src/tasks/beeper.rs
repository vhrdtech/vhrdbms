use stm32l0xx_hal::pwm::{Pwm, C1, Assigned};
use stm32l0xx_hal::gpio::gpioa::PA0;
use stm32l0xx_hal::gpio::Analog;
use stm32l0xx_hal::pac::TIM2;

pub type BuzzerPwmChannel = Pwm<TIM2, C1, Assigned<PA0<Analog>>>;

pub enum Event {

}

pub fn beeper(_cx: crate::beeper::Context, _e: Event, _state: &mut u32,) {

}