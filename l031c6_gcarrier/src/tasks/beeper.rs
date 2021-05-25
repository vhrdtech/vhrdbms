use stm32l0xx_hal::pwm::{Pwm, C1, Assigned};
use stm32l0xx_hal::gpio::gpioa::PA0;
use stm32l0xx_hal::gpio::Analog;
use stm32l0xx_hal::pac::TIM2;

pub type BuzzerPwmChannel = Pwm<TIM2, C1, Assigned<PA0<Analog>>>;

pub enum Event {

}

pub fn beeper(cx: crate::beeper::Context, e: Event, state: &mut u32,) {

}