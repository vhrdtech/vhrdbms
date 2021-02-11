use core::fmt::Write;
//#[allow(unused_imports)]
//use panic_ramlog;

use alloc_cortex_m::CortexMHeap;
#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
extern crate alloc;
use alloc::boxed::Box;

use stm32l0xx_hal as hal;
use hal::prelude::*;
use hal::rcc::MSIRange;

use power_helper::power_block::{PowerBlock, PowerBlockType, DummyInputPin};
use crate::power_block::PowerBlockId;

use mcp25625::{MCP25625, MCP25625Config, FiltersConfig, McpOperationMode, FiltersConfigBuffer0};
use tca9535::tca9534::Tca9534;

use cfg_if::cfg_if;
use stm32l0xx_hal::exti::{GpioLine, ExtiLine, TriggerEdge, Exti};
use stm32l0xx_hal::syscfg::SYSCFG;
use stm32l0xx_hal::adc::{Precision, SampleTime};
use vhrdcan::FrameId;
use stm32l0xx_hal::pwr::PWR;
use stm32l0xx_hal::pac::Peripherals;
use bq769x0::BQ769x0;

use crate::tasks;
use crate::config;

pub fn init(cx: crate::init::Context) -> crate::init::LateResources {
    let mut rtt = jlink_rtt::NonBlockingOutput::new(false);
    writeln!(rtt, "{} v{}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION")).ok();

    let core/*: cortex_m::Peripherals */= cx.core;
    let device: hal::pac::Peripherals = cx.device;
    let mut watchdog = device.IWDG.watchdog();
    //watchdog.start(1000.ms()); 1s maximum, /0 otherwise
    watchdog.set_config(5, 4095); // 5->38?kHz/128 approx 16s
    watchdog.feed();

    let rcc = device.RCC;//.constrain();
    let mut rcc = rcc.freeze(hal::rcc::Config::msi(MSIRange::Range5)); // Range5 = 2.097MHz
    let clocks = rcc.clocks;
    let mut syscfg = SYSCFG::new(device.SYSCFG, &mut rcc);
    let mut exti = Exti::new(device.EXTI);
    let mut scb   = core.SCB;
    let mut pwr = PWR::new(device.PWR, &mut rcc);

    let gpioa = device.GPIOA.split(&mut rcc);
    let gpiob = device.GPIOB.split(&mut rcc);
    let gpioc = device.GPIOC.split(&mut rcc);

    unsafe {
        let device = Peripherals::steal();
        device.RCC.csr.modify(|_, w| w.rtcsel().bits(0b00).rtcen().clear_bit().lseon().clear_bit());
    }

    let mut status_led = gpioc.pc14.into_push_pull_output();
    status_led.set_high().ok();
    cortex_m::asm::delay(ms2cycles_raw!(clocks, 10));

    // Power switches and DC-DCs
    use PowerBlockType::*;
    use PowerBlockId::*;
    // Be careful with the map size!
    let mut power_blocks = config::PowerBlocksMap::new();
    // BMS Low Iq DC-DC
    let mut dcdc_5v0_bms_en = gpioc.pc13.into_push_pull_output();
    dcdc_5v0_bms_en.set_high().ok();

    // Go back to standby mode if button is not pressed (woken by watchdog)
    let button = gpioa.pa11.into_floating_input();
    // if button.is_high().unwrap() {
    //     watchdog.feed();
    //     writeln!(rtt, "Back to sleep").ok();
    //     pwr.standby_mode(&mut scb).enter();
    // }

    // Initialize RTC, this date will only be used if RTC is not yet initialised
    // let instant = Instant::new()
    //     .set_year(21)
    //     .set_month(1)
    //     .set_day(29)
    //     .set_hour(0)
    //     .set_minute(0)
    //     .set_second(0);
    // let _rtc = RTC::new(device.RTC, &mut rcc, &mut pwr, instant);

    // Initialize allocator
    // Used only in init to store PowerBlock trait objects in array
    let start = cortex_m_rt::heap_start() as usize;
    let size = 512; // in bytes
    unsafe { ALLOCATOR.init(start, size) }

    // let mut pb_5v0_bms: PowerBlock<_, DummyInputPin> = PowerBlock::new(
    //     DcDc, dcdc_5v0_bms_en, None
    // );
    // pb_5v0_bms.enable(); // enable right away
    // let _ = power_blocks.insert(DcDc5V0Bms, Box::new(pb_5v0_bms)); // .expect("I0");
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
    // power_blocks.get_mut(&PowerBlockId::Switch5V0Syscan).expect("I1").enable();

    // Enable IRQ on power button input
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
        vchg_div_pin,
        dcdc_en_pin: dcdc_5v0_bms_en
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
    let filters_buffer0 = FiltersConfigBuffer0 {
        mask: vhrdcan::EXTENDED_ID_ALL_BITS,
        filter0: config::SOFTOFF_NOTIFY_FRAME_ID,
        filter1: Some(FrameId::new_standard(0x123).unwrap())
    };
    let filters_config = FiltersConfig::Filter(filters_buffer0, None);
    let mcp_config = MCP25625Config {
        brp: 0, // Fosc=16MHz
        prop_seg: 3,
        ph_seg1: 2,
        ph_seg2: 2,
        sync_jump_width: 2,
        rollover_to_buffer1: true,
        filters_config,
        //filters_config: FiltersConfig::ReceiveAll,
        operation_mode: McpOperationMode::Normal
    };
    let r = mcp25625.apply_config(mcp_config);
    writeln!(rtt, "mcp25625 config: {:?}", r).ok();
    mcp25625.enable_interrupts(0b0001_1111);
    // Queues
    let can_tx = config::CanTX::new();
    let can_rx = config::CanRX::new();

    // Internal I2C to AFE and Gauge
    //use crc_any::CRCu8;
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
    match crate::tasks::bms::afe_init(&mut i2c, &mut bq76920) {
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

    cx.spawn.watchdog().unwrap();
    cx.spawn.bms_event(tasks::bms::BmsEvent::CheckAfe).unwrap();
    cx.spawn.bms_event(tasks::bms::BmsEvent::CheckBalancing).unwrap();
    cx.spawn.api(tasks::api::Event::SendHeartbeat).unwrap();
    cx.spawn.blinker(tasks::led::Event::Toggle).unwrap();
    rtic::pend(config::BUTTON_IRQ); // if we got to this point, button was pressed

    let bms_state = tasks::bms::BmsState::default();
    let button_state = tasks::button::ButtonState::default();
    let softoff_state = tasks::softoff::State::default();

     crate::init::LateResources {
        clocks,
        watchdog,
        bms_state,
        afe_io,
        adc,
        status_led,
        rtt,
        mcp25625,
        mcp25625_irq,
        can_tx,
        can_rx,
        i2c,
        bq76920,
        power_blocks,
        tca9534,
        exti,
        button,
        button_state,
        softoff_state
    }
}