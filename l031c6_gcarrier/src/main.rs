#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

use core::fmt::Write;
use jlink_rtt::NonBlockingOutput;
#[allow(unused_imports)]
use panic_ramlog;

use alloc_cortex_m::CortexMHeap;
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
extern crate alloc;
use alloc::vec::Vec;
use alloc::boxed::Box;

use rtic::app;
use rtic::Monotonic;

use stm32l0xx_hal as hal;
use hal::prelude::*;
use hal::gpio::{Output, PushPull};
use hal::gpio::gpioc::PC14;
use hal::rcc::MSIRange;

use mcu_helper::tim_cyccnt::{TimCyccnt, U32Ext};

use mcp25625::{MCP25625, MCP25625Config};

use hal::gpio::gpioa::{PA4, PA5, PA6, PA7};
use hal::gpio::Analog;
//use hal::gpio::PushPull;
type Mcp25625Sck = PA5<Analog>;
type Mcp25625Miso = PA6<Analog>;
type Mcp25625Mosi = PA7<Analog>;
type Mcp25625Cs = PA4<Output<PushPull>>;
type Mcp25625Instance = mcp25625::MCP25625<hal::spi::Spi<hal::pac::SPI1, (Mcp25625Sck, Mcp25625Miso, Mcp25625Mosi)>, Mcp25625Cs>;

use power_helper::power_block::{PowerBlock, PowerBlockType, DummyInputPin, PowerBlockControl};

#[app(device = stm32l0xx_hal::pac, peripherals = true, monotonic = mcu_helper::tim_cyccnt::TimCyccnt)]
const APP: () = {
    struct Resources {
        clocks: hal::rcc::Clocks,
        status_led: PC14<Output<PushPull>>,
        rtt: NonBlockingOutput,
        mcp25625: Mcp25625Instance
    }

    #[init(
        spawn = [blinker]
    )]
    fn init(cx: init::Context) -> init::LateResources {
        let mut rtt = jlink_rtt::NonBlockingOutput::new(false);
        writeln!(rtt, "{} v{}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION")).ok();

        // Initialize allocator
        // Used only in init to store PowerBlock trait objects in array
        let start = cortex_m_rt::heap_start() as usize;
        let size = 1024; // in bytes
        unsafe { ALLOCATOR.init(start, size) }

        let _core/*: cortex_m::Peripherals */= cx.core;
        let device: hal::pac::Peripherals = cx.device;

        let rcc = device.RCC;//.constrain();
        let mut rcc = rcc.freeze(hal::rcc::Config::msi(MSIRange::Range5)); // Range5 = 2.097MHz
        let clocks = rcc.clocks;

        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiob = device.GPIOB.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);
        let mut status_led = gpioc.pc14.into_push_pull_output();
        status_led.set_low().ok();

        // Power switches and DC-DCs
        use PowerBlockType::*;
        // BMS Low Iq DC-DC
        let dcdc_5v0_bms_en = gpioc.pc13.into_push_pull_output();
        let pb_5v0_bms: PowerBlock<_, DummyInputPin> = PowerBlock::new(
            DcDc, dcdc_5v0_bms_en, None, "5v0_bms"
        );
        // CAN bus transceivers switch U16
        let ps_5v0_syscan_en = gpioa.pa3.into_push_pull_output();
        let ps_5v0_syscan_pgood = gpiob.pb3.into_pull_up_input();
        let pb_5v0_syscan = PowerBlock::new(
            Switch, ps_5v0_syscan_en, Some(ps_5v0_syscan_pgood), "5v0_syscan"
        );
        // External connector switch U15
        let ps_5v0_m12_en = gpiob.pb11.into_push_pull_output();
        let ps_5v0_m12_pgood = gpiob.pb8.into_pull_up_input();
        let pb_5v0_m12 = PowerBlock::new(
            Switch, ps_5v0_m12_en, Some(ps_5v0_m12_pgood), "5v0_m12"
        );
        // Flex to radars switch U17
        let ps_5v0_flex_en = gpiob.pb9.into_push_pull_output();
        let ps_5v0_flex_pgood = gpiob.pb4.into_pull_up_input();
        let pb_5v0_flex = PowerBlock::new(
            Switch, ps_5v0_flex_en, Some(ps_5v0_flex_pgood), "5v0_flex"
        );
        // 5V0 DC-DC U7
        let dcdc_5v0_hc_en = gpiob.pb0.into_push_pull_output();
        let dcdc_5v0_hc_pgood = gpiob.pb2.into_pull_up_input();
        let pb_5v0_hc = PowerBlock::new(
            DcDc, dcdc_5v0_hc_en, Some(dcdc_5v0_hc_pgood), "5v0_hc"
        );
        // 3V3 DC-DC U9
        let dcdc_3v3_hc_en = gpioa.pa8.into_push_pull_output();
        let dcdc_3v3_hc_pgood = gpiob.pb15.into_pull_up_input();
        let pb_3v3_hc = PowerBlock::new(
            DcDc, dcdc_3v3_hc_en, Some(dcdc_3v3_hc_pgood), "3v3_hc"
        );
        // HC switch U13 to DC-DC U9, LC switch U14 to BMS Low Iq DC-DC, OR-ed fault
        let ps_3v3_uwblc_en = gpiob.pb12.into_push_pull_output();
        let ps_3v3_uwbhc_en = gpiob.pb6.into_push_pull_output();
        let ps_3v3_uwb_pgood = gpiob.pb7.into_pull_up_input();
        let pb_3v3_uwblc = PowerBlock::new(
            Switch, ps_3v3_uwblc_en, Some(ps_3v3_uwb_pgood), "3v3_uwblc"
        );
        let pb_3v3_uwbhc: PowerBlock<_, DummyInputPin> = PowerBlock::new(
            Switch, ps_3v3_uwbhc_en, None, "3v3_uwbhc"
        );
        // TMS switch U11
        let ps_3v3_tms_en = gpiob.pb5.into_push_pull_output();
        let ps_3v3_tms_pgood = gpioc.pc15.into_pull_up_input();
        let pb_3v3_tms = PowerBlock::new(
            Switch, ps_3v3_tms_en, Some(ps_3v3_tms_pgood), "3v3_tms"
        );
        // IMX DC-DC U6
        let dcdc_3v8_imx_en = gpiob.pb14.into_push_pull_output();
        let dcdc_3v8_imx_flt = gpiob.pb1.into_pull_up_input();
        let pb_3v8_imx = PowerBlock::new(
            DcDc, dcdc_3v8_imx_en, Some(dcdc_3v8_imx_flt), "3v8_imx"
        );
        // USB switch U12
        let ps_5v0_usb_en = gpiob.pb13.into_push_pull_output();
        let ps_5v0_usb_pgood = gpiob.pb10.into_pull_up_input();
        let pb_3v8_usb = PowerBlock::new(
            Switch, ps_5v0_usb_en, Some(ps_5v0_usb_pgood), "5v0_usb"
        );

        let mut power_blocks: Vec<Box<dyn PowerBlockControl>> = Vec::new();
        power_blocks.push(Box::new(pb_5v0_bms));
        power_blocks.push(Box::new(pb_5v0_syscan));
        power_blocks.push(Box::new(pb_5v0_m12));
        power_blocks.push(Box::new(pb_5v0_flex));
        power_blocks.push(Box::new(pb_5v0_hc));
        power_blocks.push(Box::new(pb_3v3_hc));
        power_blocks.push(Box::new(pb_3v3_uwblc));
        power_blocks.push(Box::new(pb_3v3_uwbhc));
        power_blocks.push(Box::new(pb_3v3_tms));
        power_blocks.push(Box::new(pb_3v8_imx));
        power_blocks.push(Box::new(pb_3v8_usb));

        power_blocks[0].enable();
        power_blocks[1].enable();

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
        mcp25625.reset_interrupt_flags(0);
        mcp25625.reset_error_flags();

        // Internal I2C to AFE and Gauge
        //use crc_any::CRCu8;
        use bq769x0::BQ769x0;
        use bq769x0::{SCDDelay, Amperes, OCDDelay, MilliVolts, MicroOhms, UVDelay, OVDelay};
        use bq769x0::Config as BQ769x0Config;
        let scl = gpioa.pa9.into_open_drain_output();
        let sda = gpioa.pa10.into_open_drain_output();
        let mut i2c = device.I2C1.i2c(sda, scl, 100.khz(), &mut rcc);
        // Prepare crc8 library
        //let mut crc8 = CRCu8::crc8();
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
            rtt,
            mcp25625,
        }
    }

    #[task(
        resources = [
            &clocks,
            status_led,
            rtt,
            mcp25625
        ],
        schedule = [
            blinker,
        ]
    )]
    fn blinker(cx: blinker::Context) {
        cx.resources.status_led.toggle().ok();
        let schedule_at = cx.scheduled + 2_000_000.cycles();
        cx.schedule.blinker(schedule_at).ok();

        let rtt = cx.resources.rtt;

        use mcp25625::{McpReceiveBuffer, CanAddress, McpPriority};
        let mcp25625 = cx.resources.mcp25625;
        let intf = mcp25625.interrupt_flags();
        writeln!(rtt, "{:?}", intf).ok();
        let errf = mcp25625.error_flags();
        writeln!(rtt, "{:?}", errf).ok();

        if intf.rx0if_is_set() {
            let message = mcp25625.receive(McpReceiveBuffer::Buffer0);
            writeln!(rtt, "rx0:{:?}", message).ok();
        }
        if intf.rx1if_is_set() {
            let message = mcp25625.receive(McpReceiveBuffer::Buffer1);
            writeln!(rtt, "rx1:{:?}", message).ok();
        }
        let tec = mcp25625.tec();
        let rec = mcp25625.rec();
        writeln!(rtt, "tec:{}, rec:{}", tec, rec).ok();

        //mcp25625.reset_interrupt_flags(intf.bits);
        //mcp25625.reset_error_flags();

        let mut empty_tx_bufs = 0u8;
        if intf.tx0if_is_set() {
            empty_tx_bufs = empty_tx_bufs + 1
        }
        if intf.tx1if_is_set() {
            empty_tx_bufs = empty_tx_bufs + 1
        }
        if intf.tx2if_is_set() {
            empty_tx_bufs = empty_tx_bufs + 1
        }
        if empty_tx_bufs > 0 {
            writeln!(rtt, "{} tx buffers free", empty_tx_bufs).ok();
        }


        let r = mcp25625.send(CanAddress::extended(0x1769A5F3), &[0xaa, 0xbb, 0xcc, 0xdd], McpPriority::Low);
        writeln!(rtt, "tx: {:?}", r).ok();
        writeln!(rtt, "\n\n\n").ok();
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

use core::alloc::Layout;
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    panic!("OOM");
}