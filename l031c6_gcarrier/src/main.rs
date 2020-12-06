#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

#[macro_use]
mod util;
mod tasks;
mod config;
mod power_block;
mod board_components;

use core::fmt::Write;
use jlink_rtt::NonBlockingOutput;
//#[allow(unused_imports)]
//use panic_ramlog;

use alloc_cortex_m::CortexMHeap;
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
extern crate alloc;
use alloc::boxed::Box;

use rtic::app;
//use rtic::Monotonic;

use stm32l0xx_hal as hal;
use hal::prelude::*;
use hal::gpio::{Output, PushPull};
use hal::gpio::gpioc::PC14;
use hal::rcc::MSIRange;
use hal::adc::Adc;

use mcu_helper::tim_cyccnt::{U32Ext};
use power_helper::power_block::{PowerBlock, PowerBlockType, DummyInputPin, PowerBlockControl};
use crate::power_block::PowerBlockId;

use mcp25625::{MCP25625, MCP25625Config};
use bq769x0::BQ769x0;
use tca9535::tca9534::Tca9534;

use cfg_if::cfg_if;

#[app(device = stm32l0xx_hal::pac, peripherals = true, monotonic = mcu_helper::tim_cyccnt::TimCyccnt)]
const APP: () = {
    struct Resources {
        clocks: hal::rcc::Clocks,
        bms_state: tasks::bms::BmsState,
        afe_io: tasks::bms::AfeIo,
        adc: Adc<hal::adc::Ready>,
        status_led: PC14<Output<PushPull>>,
        rtt: NonBlockingOutput,
        mcp25625: config::Mcp25625Instance,
        mcp25625_irq: config::Mcp25625Irq,
        i2c: config::InternalI2c,
        bq76920: BQ769x0,
        power_blocks: config::PowerBlocksMap,
        tca9534: Tca9534<config::InternalI2c>,
        exti: Exti,
        button: config::ButtonPin,
        button_state: tasks::button::ButtonState
    }

    #[init(
        spawn = [bms_event, blinker]
    )]
    fn init(cx: init::Context) -> init::LateResources {
        let mut rtt = jlink_rtt::NonBlockingOutput::new(false);
        writeln!(rtt, "{} v{}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION")).ok();

        // Initialize allocator
        // Used only in init to store PowerBlock trait objects in array
        let start = cortex_m_rt::heap_start() as usize;
        let size = 1024; // in bytes
        unsafe { ALLOCATOR.init(start, size) }

        // let core/*: cortex_m::Peripherals */= cx.core;
        let device: hal::pac::Peripherals = cx.device;

        let rcc = device.RCC;//.constrain();
        let mut rcc = rcc.freeze(hal::rcc::Config::msi(MSIRange::Range5)); // Range5 = 2.097MHz
        let clocks = rcc.clocks;
        let mut syscfg = SYSCFG::new(device.SYSCFG, &mut rcc);
        let mut exti = Exti::new(device.EXTI);

        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiob = device.GPIOB.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);
        let mut status_led = gpioc.pc14.into_push_pull_output();
        status_led.set_low().ok();

        // Power switches and DC-DCs
        use PowerBlockType::*;
        use PowerBlockId::*;
        // Be careful with the map size!
        let mut power_blocks = config::PowerBlocksMap::new();
        // BMS Low Iq DC-DC
        let dcdc_5v0_bms_en = gpioc.pc13.into_push_pull_output();
        let mut pb_5v0_bms: PowerBlock<_, DummyInputPin> = PowerBlock::new(
            DcDc, dcdc_5v0_bms_en, None
        );
        pb_5v0_bms.enable(); // enable right away
        let _ = power_blocks.insert(DcDc5V0Bms, Box::new(pb_5v0_bms)); // .expect("I0");
        // CAN bus transceivers switch U16
        let ps_5v0_syscan_en = gpioa.pa3.into_push_pull_output();
        let ps_5v0_syscan_pgood = gpiob.pb3.into_pull_up_input();
        let pb_5v0_syscan = PowerBlock::new(
            Switch, ps_5v0_syscan_en, Some(ps_5v0_syscan_pgood)
        );
        let _ = power_blocks.insert(Switch5V0Syscan, Box::new(pb_5v0_syscan));
        // External connector switch U15
        let ps_5v0_m12_en = gpiob.pb11.into_push_pull_output();
        let ps_5v0_m12_pgood = gpiob.pb8.into_pull_up_input(); // trace reworked
        let pb_5v0_m12 = PowerBlock::new(
            Switch, ps_5v0_m12_en, Some(ps_5v0_m12_pgood)
        );
        let _ = power_blocks.insert(Switch5V0M12, Box::new(pb_5v0_m12));
        // Flex to radars switch U17
        let ps_5v0_flex_en = gpiob.pb9.into_push_pull_output();
        let ps_5v0_flex_pgood = gpiob.pb4.into_pull_up_input();
        let pb_5v0_flex = PowerBlock::new(
            Switch, ps_5v0_flex_en, Some(ps_5v0_flex_pgood)
        );
        let _ = power_blocks.insert(Switch5V0Flex, Box::new(pb_5v0_flex));
        // 5V0 DC-DC U7
        let dcdc_5v0_hc_en = gpiob.pb0.into_push_pull_output();
        let dcdc_5v0_hc_pgood = gpiob.pb2.into_pull_up_input();
        let pb_5v0_hc = PowerBlock::new(
            DcDc, dcdc_5v0_hc_en, Some(dcdc_5v0_hc_pgood)
        );
        let _ = power_blocks.insert(DcDc5V0Hc, Box::new(pb_5v0_hc));
        // 3V3 DC-DC U9
        let dcdc_3v3_hc_en = gpioa.pa8.into_push_pull_output();
        let dcdc_3v3_hc_pgood = gpiob.pb15.into_pull_up_input();
        let pb_3v3_hc = PowerBlock::new(
            DcDc, dcdc_3v3_hc_en, Some(dcdc_3v3_hc_pgood)
        );
        let _ = power_blocks.insert(DcDc3V3Hc, Box::new(pb_3v3_hc));
        // HC switch U13 to DC-DC U9, LC switch U14 to BMS Low Iq DC-DC, OR-ed fault
        let ps_3v3_uwblc_en = gpiob.pb12.into_push_pull_output();
        let ps_3v3_uwbhc_en = gpiob.pb6.into_push_pull_output();
        let ps_3v3_uwb_pgood = gpiob.pb7.into_pull_up_input();
        let pb_3v3_uwblc: PowerBlock<_, DummyInputPin>   = PowerBlock::new(
            Switch, ps_3v3_uwblc_en, None
        );
        let pb_3v3_uwbhc= PowerBlock::new(
            Switch, ps_3v3_uwbhc_en, Some(ps_3v3_uwb_pgood)
        );
        let _ = power_blocks.insert(Switch3V3UWBLc, Box::new(pb_3v3_uwblc));
        let _ = power_blocks.insert(Switch3V3UWBHc, Box::new(pb_3v3_uwbhc));
        // TMS switch U11
        let ps_3v3_tms_en = gpiob.pb5.into_push_pull_output();
        let ps_3v3_tms_pgood = gpioc.pc15.into_pull_up_input();
        let pb_3v3_tms = PowerBlock::new(
            Switch, ps_3v3_tms_en, Some(ps_3v3_tms_pgood)
        );
        let _ = power_blocks.insert(Switch3V3Tms, Box::new(pb_3v3_tms));
        // IMX DC-DC U6
        let dcdc_3v8_imx_en = gpiob.pb14.into_push_pull_output();
        let dcdc_3v8_imx_flt = gpiob.pb1.into_pull_up_input();
        let pb_3v8_imx = PowerBlock::new(
            DcDc, dcdc_3v8_imx_en, Some(dcdc_3v8_imx_flt)
        );
        let _ = power_blocks.insert(DcDc3V8Imx, Box::new(pb_3v8_imx));
        // USB switch U12
        let ps_5v0_usb_en = gpiob.pb13.into_push_pull_output();
        let ps_5v0_usb_pgood = gpiob.pb10.into_pull_up_input();
        let pb_5v0_usb = PowerBlock::new(
            Switch, ps_5v0_usb_en, Some(ps_5v0_usb_pgood)
        );
        let _ = power_blocks.insert(Switch5V0Usb, Box::new(pb_5v0_usb));

        // power_blocks.get_mut(&PowerBlockId::DcDc3V3Hc).expect("I1").enable();
        // power_blocks.get_mut(&PowerBlockId::Switch3V3Tms).expect("I1").enable();
        power_blocks.get_mut(&PowerBlockId::Switch5V0Syscan).expect("I1").enable();

        // Enable IRQ on power button input
        let button = gpioa.pa11.into_floating_input();
        let button_line = GpioLine::from_raw_line(button.pin_number()).unwrap();
        exti.listen_gpio(&mut syscfg, button.port(), button_line, TriggerEdge::Falling);

        // BQ76920 wake pin
        let mut afe_wake_pin = gpioa.pa12.into_push_pull_output();
        afe_wake_pin.set_high().ok();
        cortex_m::asm::delay(100_000);
        afe_wake_pin.set_low().ok();
        // Charge voltage sense, divider enable on TCA9534.P3
        let vchg_div_pin = gpioa.pa2.into_analog();
        let mut adc = device.ADC.constrain(&mut rcc);
        unsafe {
            let device = hal::pac::Peripherals::steal();
            let adc = device.ADC;
            adc.cfgr2.modify(|_, w| w.ckmode().pclk());
            adc.cr.modify(|_, w| w.adcal().set_bit());
            while adc.cr.read().adcal().bit_is_set() {}
            writeln!(rtt, "ADC cal done: {}", adc.calfact.read().bits()).ok();
        }
        // let mut adc = Adc::new(device.ADC, &mut rcc);
        adc.set_precision(Precision::B_12);
        adc.set_sample_time(SampleTime::T_160_5);
        let afe_io = tasks::bms::AfeIo {
            afe_wake_pin,
            vchg_div_pin
        };

        // SPI<->CANBus MCP25625T-E/ML
        let mcp_freq = 1.mhz();
        // GPIO init
        let mcp25625_irq         = gpioa.pa1.into_pull_down_input();
        let mcp_int_line = GpioLine::from_raw_line(mcp25625_irq.pin_number()).unwrap();
        exti.listen_gpio(&mut syscfg, mcp25625_irq.port(), mcp_int_line, TriggerEdge::Falling);
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
        let mut mcp25625 = MCP25625::new(spi, mcp_cs, one_cp);
        let mcp_config = MCP25625Config {
            brp: 0, // Fosc=16MHz
            prop_seg: 3,
            ph_seg1: 2,
            ph_seg2: 2,
            sync_jump_width: 2
        };
        let r = mcp25625.apply_config(mcp_config);
        writeln!(rtt, "mcp: {:?}", r).ok();
        let _ = mcp25625.reset_interrupt_flags(0);
        let _ = mcp25625.reset_error_flags();
        let _ = mcp25625.masks_rxall();
        let r = mcp25625.change_mode(mcp25625::McpOperationMode::Normal);
        writeln!(rtt, "mcp->normal: {:?}", r).ok();
        // mcp25625.enable_interrupts();

        // Internal I2C to AFE and Gauge
        //use crc_any::CRCu8;
        use bq769x0::BQ769x0;
        use bq769x0::{SCDDelay, Amperes, OCDDelay, MilliVolts, MicroOhms, UVDelay, OVDelay};
        use bq769x0::Config as BQ769x0Config;
        let scl = gpioa.pa9.into_open_drain_output();
        let sda = gpioa.pa10.into_open_drain_output();
        cfg_if! {
            if #[cfg(feature = "bitbang-i2c")] {
                let i2c_timer = device.TIM2.timer(200.khz(), &mut rcc);
                let mut i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, i2c_timer);
            } else {
                let mut i2c = device.I2C1.i2c(sda, scl, 100.khz(), &mut rcc);
            }
        }
        // Prepare crc8 library
        //let mut crc8 = CRCu8::crc8();
        // Prepare bq76920 config
        let mut bq76920 = BQ769x0::new(0x08);
        let bq769x0_config = BQ769x0Config {
            shunt: MicroOhms(2000),
            scd_delay: SCDDelay::_400uS,
            scd_threshold: Amperes(100),
            ocd_delay: OCDDelay::_640ms,
            ocd_threshold: Amperes(25),
            uv_delay: UVDelay::_4s,
            uv_threshold: MilliVolts(3050),
            ov_delay: OVDelay::_4s,
            ov_threshold: MilliVolts(4250)
        };
        match bq76920.init(&mut i2c, &bq769x0_config) {
            Ok(_actual) => {
                let _ = writeln!(rtt, "afe init ok");
                //let _ = writeln!(rtt, "adc gain:{}uV/LSB offset:{}mV", bq76920.adc_gain(), bq76920.adc_offset());
                //let _ = writeln!(rtt, "SCD: {}, OCD: {}, range: {:?}", actual.scd_threshold, actual.ocd_threshold, actual.ocdscd_range_used);
                //let _ = writeln!(rtt, "UV: {}, OV: {}", actual.uv_threshold, actual.ov_threshold);
            }
            Err(e) => {
                let _ = writeln!(rtt, "afe init err:{:?}", e);
            }
        }
        // let r = bq76920.discharge(&mut i2c, true);
        // let _ = writeln!(rtt, "DE: {:?}", r);

        let tca9534 = Tca9534::new(&mut i2c, tca9535::Address::ADDR_0x20);
        let r = tca9534.write_config(&mut i2c, tca9535::tca9534::Port::empty());
        let _ = writeln!(rtt, "tca9534: {}", r.is_ok());
        let _ = tca9534.write_outputs(&mut i2c, tca9535::tca9534::Port::empty());

        cx.spawn.bms_event(tasks::bms::BmsEvent::CheckCharger).unwrap();
        cx.spawn.bms_event(tasks::bms::BmsEvent::CheckBalancing).unwrap();
        cx.spawn.api(tasks::api::Event::SendHeartbeat).unwrap();
        cx.spawn.blinker().unwrap();

        let bms_state = tasks::bms::BmsState::default();
        let button_state = tasks::button::ButtonState::default();

        init::LateResources {
            clocks,
            bms_state,
            afe_io,
            adc,
            status_led,
            rtt,
            mcp25625,
            mcp25625_irq,
            i2c,
            bq76920,
            power_blocks,
            tca9534,
            exti,
            button,
            button_state,
        }
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
        capacity = 4,
        schedule = [
            bms_event,
        ]
    )]
    fn bms_event(cx: bms_event::Context, e: tasks::bms::BmsEvent) {
        tasks::bms::bms_event(cx, e);
    }

    #[task(
        resources = [
            &clocks,
            status_led,
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

    #[task(
        binds = EXTI0_1,
        resources = [
            &clocks,
            exti,
            mcp25625,
            mcp25625_irq,
            rtt
        ],
            spawn = [button_event]
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
        resources = [i2c, bq76920, rtt, power_blocks, tca9534]
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
use stm32l0xx_hal::exti::{GpioLine, ExtiLine, TriggerEdge, Exti};
use stm32l0xx_hal::syscfg::SYSCFG;
use stm32l0xx_hal::adc::{Precision, SampleTime};

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset(); // -> !
}