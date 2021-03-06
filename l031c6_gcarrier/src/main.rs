#![no_main]
#![no_std]
#![feature(alloc_error_handler)]
#![feature(const_option)]

#[macro_use]
mod util;
mod tasks;
mod config;
mod power_block;
mod board_components;

use core::fmt::Write;
use jlink_rtt::NonBlockingOutput;
extern crate alloc;

use rtic::app;

use stm32l0xx_hal as hal;
use hal::prelude::*;
use hal::gpio::{Output, PushPull};
use hal::gpio::gpioc::PC14;
use hal::adc::Adc;
use hal::exti::{Exti};
use hal::watchdog::IndependedWatchdog;

use mcu_helper::tim_cyccnt::{U32Ext};

#[app(device = stm32l0xx_hal::pac, peripherals = true, monotonic = mcu_helper::tim_cyccnt::TimCyccnt)]
const APP: () = {
    struct Resources {
        clocks: hal::rcc::Clocks,
        watchdog: IndependedWatchdog,
        bms_state: tasks::bms::BmsState,
        afe_io: tasks::bms::AfeIo,
        adc: Adc<hal::adc::Ready>,
        status_led: PC14<Output<PushPull>>,
        rtt: NonBlockingOutput,
        mcp25625: config::Mcp25625Instance,
        mcp25625_irq: config::Mcp25625Irq,
        can_tx: config::CanTX,
        can_rx: config::CanRX,
        i2c: config::InternalI2c,
        bq76920: bq769x0::BQ769x0,
        power_blocks: config::PowerBlocksMap,
        tca9534: tca9535::tca9534::Tca9534<config::InternalI2c>,
        exti: Exti,
        button: config::ButtonPin,
        button_state: tasks::button::ButtonState,
        softoff_state: tasks::softoff::State
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
            power_blocks,
            tca9534,
            adc,
            afe_io
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
            rtt
        ],
        schedule = [
            blinker,
        ]
    )]
    fn blinker(cx: blinker::Context, e: tasks::led::Event) {
        static mut STATE: bool = false;
        tasks::led::blinker(cx, e, STATE);
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
        schedule = [api]
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
        spawn = [bms_event],
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

    #[task(
        binds = EXTI0_1,
        resources = [
            &clocks,
            exti,
            mcp25625,
            mcp25625_irq,
            can_tx,
            can_rx,
            rtt
        ],
        spawn = [can_rx]
    )]
    fn canctrl_irq(cx: canctrl_irq::Context) {
        tasks::canbus::canctrl_irq(cx);
    }

    #[task(
        binds = EXTI4_15,
        resources = [
            &clocks,
            button,
            button_state,
        ],
        spawn = [button_event]
    )]
    fn button_irq(cx: button_irq::Context) {
        tasks::button::button_irq(cx);
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
        resources = [i2c, bq76920, rtt, power_blocks, tca9534],
        spawn = [bms_event]
    )]
    fn idle(cx: idle::Context) -> ! {
        tasks::idle::idle(cx);
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

use core::alloc::Layout;
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    panic!("OOM");
}

use core::panic::PanicInfo;
#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::asm::delay(6_000_000);
    let mut rtt = jlink_rtt::NonBlockingOutput::new(false);
    writeln!(rtt, "{:?}", _info).ok();
    cortex_m::peripheral::SCB::sys_reset(); // -> !
}