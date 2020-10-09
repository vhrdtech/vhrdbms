#![no_main]
#![no_std]

use core::fmt::Write;
use jlink_rtt::NonBlockingOutput;
#[allow(unused_imports)]
use panic_ramlog;

use rtic::app;
use rtic::Monotonic;

use stm32l0xx_hal as hal;
use hal::prelude::*;
use hal::gpio::{Output, PushPull};
use hal::gpio::gpioc::PC14;
use hal::rcc::MSIRange;

use mcu_helper::tim_cyccnt::{TimCyccnt, U32Ext};

use mcp25625::{MCP25625, MCP25625Config};

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

        let gpioa = device.GPIOA.split(&mut rcc);
        let _gpiob = device.GPIOB.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);
        let mut status_led = gpioc.pc14.into_push_pull_output();
        status_led.set_low().ok();

        // SPI<->CANBus MCP25625T-E/ML
        let mcp_freq = 1.mhz();
        // GPIO init
        let _mcp_int         = gpioa.pa1;
        let mcp_stby         = gpioa.pa0; // NORMAL mode is low, SLEEP is high
        let mcp_cs         = gpioa.pa4;
        let mcp_sck         = gpioa.pa5;
        let mcp_miso         = gpioa.pa6;
        let mcp_mosi         = gpioa.pa7;
        let mut mcp_stby = mcp_stby.into_push_pull_output();
        mcp_stby.set_low().ok();
        let mut mcp_cs = mcp_cs.into_push_pull_output();
        mcp_cs.set_high().ok();
        // SPI init
        let spi = device.SPI1.spi(
            (mcp_sck, mcp_miso, mcp_mosi),
            hal::spi::MODE_0,
            mcp_freq,
            &mut rcc
        );
        // MCP25625 init
        let one_cp: u32 = 1000; // tweak to be approx. one spi clock cycle
        cortex_m::asm::delay(100_000);
        let mut mcp = MCP25625::new(spi, mcp_cs, one_cp);
        let mcp_config = MCP25625Config {
            brp: 0, // Fosc=16MHz
            prop_seg: 3,
            ph_seg1: 2,
            ph_seg2: 2,
            sync_jump_width: 2
        };
        let r = mcp.apply_config(mcp_config);
        writeln!(rtt, "mcp: {:?}", r).ok();

        // Internal I2C to AFE and Gauge
        use crc_any::CRCu8;
        use bq769x0::BQ769x0;
        use bq769x0::{SCDDelay, Amperes, OCDDelay, MilliVolts, MicroOhms, UVDelay, OVDelay};
        use bq769x0::Config as BQ769x0Config;
        let scl = gpioa.pa9.into_open_drain_output();
        let sda = gpioa.pa10.into_open_drain_output();
        let mut i2c = device.I2C1.i2c(sda, scl, 100.khz(), &mut rcc);
        // Prepare crc8 library
        let mut crc8 = CRCu8::crc8();
        // Prepare bq76920 config
        let mut bq76920 = BQ769x0::new(0x08);
        let bq769x0_config = BQ769x0Config {
            shunt: MicroOhms(2000),
            scd_delay: SCDDelay::_400uS,
            scd_threshold: Amperes(100),
            ocd_delay: OCDDelay::_1280ms,
            ocd_threshold: Amperes(50),
            uv_delay: UVDelay::_4s,
            uv_threshold: MilliVolts(3050),
            ov_delay: OVDelay::_4s,
            ov_threshold: MilliVolts(4150)
        };
        match bq76920.init(&mut i2c, &bq769x0_config) {
            Ok(actual) => {
                let _ = writeln!(rtt, "bq769x0 init ok");
                let _ = writeln!(rtt, "adc gain:{}uV/LSB offset:{}mV", bq76920.adc_gain(), bq76920.adc_offset());
                let _ = writeln!(rtt, "SCD: {}, OCD: {}, range: {:?}", actual.scd_threshold, actual.ocd_threshold, actual.ocdscd_range_used);
                let _ = writeln!(rtt, "UV: {}, OV: {}", actual.uv_threshold, actual.ov_threshold);
            }
            Err(e) => {
                let _ = writeln!(rtt, "bq769x0 init err: {:?}", e);
            }
        }

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

use cortex_m_rt::exception;
#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault: {:#?}", ef);
}