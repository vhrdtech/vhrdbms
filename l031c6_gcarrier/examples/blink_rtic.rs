#![no_main]
#![no_std]

use stm32l0xx_hal as hal;
use core::panic::PanicInfo;
use rtic::app;
use hal::gpio::{Output, PushPull};
use hal::gpio::gpioc::PC14;
use hal::prelude::*;
use core::fmt::Write;
use jlink_rtt::NonBlockingOutput;
use stm32l0xx_hal::rcc::MSIRange;
use mcu_helper::tim_cyccnt::{TimCyccnt, U32Ext};
use rtic::Monotonic;

#[app(device = stm32l0xx_hal::pac, peripherals = true, monotonic = mcu_helper::tim_cyccnt::TimCyccnt)]
const APP: () = {
    struct Resources {
        clocks: hal::rcc::Clocks,
        status_led: PC14<Output<PushPull>>,
        rtt: NonBlockingOutput
    }

    #[init(
        spawn = [blinker]
    )]
    fn init(cx: init::Context) -> init::LateResources {
        let mut rtt = jlink_rtt::NonBlockingOutput::new(false);
        writeln!(rtt, "{} v{}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION")).ok();

        let _core/*: cortex_m::Peripherals */= cx.core;

        let device: hal::pac::Peripherals = cx.device;

        let rcc = device.RCC;//.constrain();
        let mut rcc = rcc.freeze(hal::rcc::Config::msi(MSIRange::Range5)); // Range5 = 2.097MHz
        let clocks = rcc.clocks;

        let gpioc = device.GPIOC.split(&mut rcc);
        let mut status_led = gpioc.pc14.into_push_pull_output();
        status_led.set_low().ok();

        cx.spawn.blinker().ok();

        init::LateResources {
            clocks,
            status_led,
            rtt
        }
    }

    #[task(
        resources = [
            &clocks,
            status_led,
            rtt
        ],
        schedule = [
            blinker,
        ]
    )]
    fn blinker(cx: blinker::Context) {
        cx.resources.status_led.toggle().ok();
        let schedule_at = cx.scheduled + 2_000_000.cycles();
        writeln!(cx.resources.rtt, "scheduled: {:?}", schedule_at).ok();
        cx.schedule.blinker(schedule_at).ok();
    }

    #[idle(
        resources = [rtt]
    )]
    fn idle(mut cx: idle::Context) -> ! {
        use core::sync::atomic;
        loop {
            let instant = TimCyccnt::now();
            cx.resources.rtt.lock(|rtt| {
                writeln!(rtt, "idle {:?}", instant).ok();
            });
            cortex_m::asm::delay(2_000_000);
            atomic::compiler_fence(atomic::Ordering::SeqCst);
        }
    }

    extern "C" {
        fn AES_RNG_LPUART1();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset(); // -> !
}