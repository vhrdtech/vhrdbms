#![no_main]
#![no_std]
// #![feature(alloc_error_handler)]
#![feature(const_option)]
#![feature(asm)]

#[macro_use]
mod util;
mod tasks;
mod config;
// mod power_block;
// mod board_components;

// use core::fmt::Write;
use jlink_rtt::NonBlockingOutput;
// extern crate alloc;

use rtic::app;

use stm32l0xx_hal as hal;
use hal::prelude::*;
use hal::gpio::{Output, PushPull};
use hal::adc::Adc;
use hal::exti::{Exti};
use hal::watchdog::IndependedWatchdog;

use mcu_helper::tim_cyccnt::{U32Ext};

#[app(device = stm32l0xx_hal::pac, peripherals = true, monotonic = mcu_helper::tim_cyccnt::TimCyccnt)]
const APP: () = {
    struct Resources {
        clocks: hal::rcc::Clocks,
        rcc: hal::rcc::Rcc,
        watchdog: IndependedWatchdog,
        bms_state: tasks::bms::BmsState,
        afe_io: tasks::bms::AfeIo,
        adc: Adc<hal::adc::Ready>,
        status_led: PB2<Output<PushPull>>,
        rtt: NonBlockingOutput,
        mcp25625_state: tasks::canbus::Mcp25625State,
        // mcp25625_irq: config::Mcp25625Irq,
        can_tx: config::CanTX,
        can_rx: config::CanRX,
        i2c: config::InternalI2c,
        bq76920: config::BQ769x0,
        // power_blocks: config::PowerBlocksMap,
        exti: Exti,
        button: config::ButtonPin,
        button_state: tasks::button::ButtonState,
        softoff_state: tasks::softoff::State,
        charge_indicator: tasks::led::ChargeIndicator,
        buzzer_pwm_channel: tasks::beeper::BuzzerPwmChannel,
    }

    #[init(
        spawn = [bms_event, api, blinker, watchdog]
    )]
    fn init(cx: init::Context) -> init::LateResources {
        tasks::init::init(cx)
    }

    #[task(
        resources = [
            &clocks,
            bms_state,
            i2c,
            bq76920,
            rtt,
            adc,
            afe_io,
            can_tx,
        ],
        capacity = 8,
        schedule = [
            bms_event,
        ],
        spawn = [
            bms_event,
            softoff,
            blinker
        ]
    )]
    fn bms_event(cx: bms_event::Context, e: tasks::bms::BmsEvent) {
        tasks::bms::bms_event(cx, e);
    }

    #[task(
        capacity = 3,
        resources = [
            &clocks,
            status_led,
            can_tx,
            charge_indicator,
            rtt
        ],
        schedule = [
            blinker,
        ]
    )]
    fn blinker(cx: blinker::Context, e: tasks::led::Event) {
        static mut STATE: bool = false;
        static mut COUNTER: u8 = 0;
        tasks::led::blinker(cx, e, STATE, COUNTER);
    }

    #[task(
        resources = [
            &clocks,
            watchdog,
        ],
        schedule = [
            watchdog,
        ]
    )]
    fn watchdog(cx: watchdog::Context) {
        let schedule_at = cx.scheduled + ms2cycles!(cx.resources.clocks, 1000);
        cx.schedule.watchdog(schedule_at).ok();
        cx.resources.watchdog.feed();
    }

    #[task(
        resources = [
            &clocks,
            can_tx
        ],
        schedule = [api],
    )]
    fn api(cx: api::Context, e: tasks::api::Event) {
        tasks::api::api(cx, e);
    }

    #[task(
        resources = [
            &clocks,
            can_rx,
            rtt
        ],
        spawn = [bms_event, softoff],
        schedule = []
    )]
    fn can_rx(cx: can_rx::Context) {
        tasks::api::can_rx(cx);
    }

    #[task(
        resources = [
            &clocks,
            can_tx,
            softoff_state,
        ],
        schedule = [softoff],
        spawn = [bms_event]
    )]
    fn softoff(cx: softoff::Context) {
        tasks::softoff::softoff(cx);
    }

    // #[task(
    //     binds = EXTI0_1,
    //     resources = [
    //         &clocks,
    //         exti,
    //         mcp25625,
    //         mcp25625_irq,
    //         can_tx,
    //         can_rx,
    //         rtt
    //     ],
    //     spawn = [can_rx]
    // )]
    // fn canctrl_irq(cx: canctrl_irq::Context) {
    //     tasks::canbus::canctrl_irq(cx);
    // }

    #[task(
        binds = EXTI4_15,
        resources = [
            &clocks,
            button,
            button_state,
            mcp25625_state,
            can_tx,
            can_rx,
            rtt
        ],
        spawn = [button_event, can_rx]
    )]
    fn exti4_15(mut cx: exti4_15::Context) {
        // let nvic = cortex_m::peripheral::NVIC::is_pending(CAN_TX_HANDLER);
        // use core::fmt::Write;
        // let pr = unsafe { (*stm32l0xx_hal::pac::EXTI::ptr()).pr.read().bits() };

        // writeln!(cx.resources.rtt, "exti {:32b} nvic:{}", pr, nvic);
        tasks::canbus::canctrl_irq(&mut cx);
        tasks::button::button_irq(&mut cx);
    }

    #[task(
        resources = [
            &clocks,
            button,
            button_state,
            rtt
        ],
        schedule = [button_event],
        spawn = [bms_event]
    )]
    fn button_event(cx: button_event::Context) {
        tasks::button::button_event(cx);
    }

    #[idle(
        resources = [i2c, bq76920, rtt, afe_io, adc, rcc, exti, mcp25625_state],
        spawn = [bms_event]
    )]
    fn idle(cx: idle::Context) -> ! {
        tasks::idle::idle(cx);
    }

    #[task(
        resources = [buzzer_pwm_channel],
        schedule = [beeper]
    )]
    fn beeper(cx: beeper::Context, e: tasks::beeper::Event) {
        static mut STATE: u32 = 0;
        tasks::beeper::beeper(cx, e, STATE);
    }

    extern "C" {
        fn AES_RNG_LPUART1();
    }
};

use cortex_m_rt::exception;
#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HF: {:#?}", ef);
}

// use core::alloc::Layout;
// #[alloc_error_handler]
// fn oom(_: Layout) -> ! {
//     panic!("OOM");
// }

use core::panic::PanicInfo;
use stm32l0xx_hal::gpio::gpiob::PB2;
use mcu_helper::color;

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    use core::fmt::Write;
    let mut rtt = jlink_rtt::NonBlockingOutput::new(false);
    // writeln!(rtt, "\n{}PANIC{}\n", color::RED, color::DEFAULT).ok();
    writeln!(rtt, "{}{:?}{}", color::RED, _info, color::DEFAULT).ok();

    cortex_m::asm::delay(6_000_000);
    cortex_m::peripheral::SCB::sys_reset(); // -> !
}