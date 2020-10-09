#![no_std]
#![no_main]

extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics

// use cortex_m::asm;
use cortex_m_rt::entry;
use stm32l0xx_hal::{pac, prelude::*, rcc::Config};

use core::fmt::Write;
use jlink_rtt;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut rtt = jlink_rtt::NonBlockingOutput::new();
    rtt.write("\x1B[2J\x1B[1;36;40mvhrdBMS\x1B[0m ");
    rtt.write(env!("CARGO_PKG_VERSION"));
    rtt.write("\n=======\n\n");

    let mut rcc = dp.RCC.freeze(Config::hsi16());
    let mut delay = cp.SYST.delay(rcc.clocks);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa15.into_push_pull_output();

    let mut n = 0u32;
    loop {
        led.set_high().unwrap();
        delay.delay_ms(1000_u16);

        led.set_low().unwrap();
        delay.delay_ms(1000_u16);

        let _ = write!(rtt, "Alive {}!\n", n);
        n = n + 1;
    }
}
