use crate::hal;
use hal::prelude::*;

pub fn panic_blink_led() {
    unsafe {
        let device = hal::pac::Peripherals::steal();
        let mut rcc = device.RCC.freeze(hal::rcc::Config::hsi16());
        let gpioc = device.GPIOC.split(&mut rcc);
        let mut led = gpioc.pc14.into_push_pull_output();

        for _ in 0..10 {
            led.toggle().ok();
            cortex_m::asm::delay(200_000);
        }
    }
}