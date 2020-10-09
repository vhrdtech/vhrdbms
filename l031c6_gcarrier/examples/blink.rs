#![no_std]
#![no_main]
//#![deny(warnings)]

use cortex_m_rt::entry;
use stm32l0xx_hal as hal;
use hal::{pac, prelude::*, rcc::Config};
// use rtt_target::rprintln;
use jlink_rtt;
use panic_ramlog::PanicInfoMeta;
use core::fmt::Write;
use mcu_helper::color;

#[entry]
fn main() -> ! {
    panic_ramlog::set_panic_led_blinker(mcu_helper::panic_helper::panic_blink_led);

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    // rtt_target::rtt_init_default!();
    // rprintln!("{} v{}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
    let mut rtt = jlink_rtt::NonBlockingOutput::new(false);
    cortex_m::asm::delay(1_000_000);

    let _ = write!(rtt, "{} v{}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));

    match PanicInfoMeta::detect_and_reset() {
        Some(panic_info) => {
            writeln!(rtt, "{}Reset happened due to panic:", color::RED).ok();
            writeln!(rtt, "at: {} {}:{}", panic_info.filename(), panic_info.line, panic_info.column).ok();
            writeln!(rtt, "message: {}{}", panic_info.message(), color::DEFAULT).ok();
        },
        None => {}
    }

    let mut rcc = dp.RCC.freeze(Config::hsi16());
    let mut delay = cp.SYST.delay(rcc.clocks);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let mut led = gpioc.pc14.into_push_pull_output();

    let mut n = 0u32;
    loop {
        led.set_high().unwrap();
        delay.delay_ms(100_u16);

        led.set_low().unwrap();
        delay.delay_ms(300_u16);

        n = n + 1;
        // rprintln!("Blink {}", n);
        let _ = write!(rtt, "Blink {}!\n", n);
        if n == 10 {
            //panic!("Panic message here");

            unsafe {
                // read an address outside of the RAM region; causes a HardFault exception
                core::ptr::read_volatile(0x2FFF_FFFF as *const u32);
            }
        }
    }
}

use cortex_m_rt::exception;
#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault: {:#?}", ef);
}