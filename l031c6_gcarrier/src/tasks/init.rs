use core::fmt::Write;
//#[allow(unused_imports)]
//use panic_ramlog;

// use alloc_cortex_m::CortexMHeap;
// #[global_allocator]
// static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
// extern crate alloc;
// use alloc::boxed::Box;

use stm32l0xx_hal as hal;
use hal::{
    prelude::*,
    rcc::MSIRange,
    exti::{GpioLine, ExtiLine, TriggerEdge, Exti},
    syscfg::SYSCFG,
    pwr::PWR,
    rtc::{Instant, RTC, ClockSource as RtcClockSource},
};

use mcp25625::MCP25625;

use cfg_if::cfg_if;
use stm32l0xx_hal::adc::{Precision, SampleTime};
use bq769x0::BQ769x0;

use crate::tasks;
use crate::config;
use crate::tasks::led::ChargeIndicator;
use crate::util::{current_stack_pointer, stack_lower_bound};
use stm32l0xx_hal::pwm;

pub fn init(cx: crate::init::Context) -> crate::init::LateResources {
    let free_stack_bytes = current_stack_pointer() - stack_lower_bound();
    unsafe {
        let p = stack_lower_bound() as *mut u32;
        let free_stack_words = free_stack_bytes / 4;
        for i in 0..free_stack_words {
            p.offset(i as isize).write(crate::util::STACK_PROBE_MAGICWORD);
        }
    }

    let mut rtt = jlink_rtt::NonBlockingOutput::new(false);
    writeln!(rtt, "{} v{}", env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION")).ok();
    writeln!(rtt, "free: {}B", free_stack_bytes).ok();

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
    let _scb   = core.SCB;
    let mut pwr = PWR::new(device.PWR, &mut rcc);

    let gpioa = device.GPIOA.split(&mut rcc);
    let gpiob = device.GPIOB.split(&mut rcc);
    let gpioc = device.GPIOC.split(&mut rcc);
    let gpioh = device.GPIOH.split(&mut rcc);

    // Disable crystal, otherwise PC14 & PC15 won't work. Not needed since crystal is used now.
    // unsafe {
    //     let device = Peripherals::steal();
    //     device.RCC.csr.modify(|_, w| w.rtcsel().bits(0b00).rtcen().clear_bit().lseon().clear_bit());
    // }

    // User Feedback
    // Blue/Green LED
    let mut status_led = gpiob.pb2.into_push_pull_output();
    status_led.set_high().ok();
    cortex_m::asm::delay(ms2cycles_raw!(clocks, 10));
    // Red LED
    let mut error_led = gpiob.pb1.into_push_pull_output();
    error_led.set_low().ok();
    // Buzzer
    let pwm = pwm::Timer::new(device.TIM2, 360.hz(), &mut rcc);
    let buzzer = gpioa.pa0;
    let buzzer_pwm_channel = pwm.channel1.assign(buzzer);
    // Ring LEDs
    let led1 = gpiob.pb12.into_push_pull_output();
    let led2 = gpiob.pb13.into_push_pull_output();
    let led3 = gpiob.pb14.into_push_pull_output();
    let led4 = gpiob.pb11.into_push_pull_output();
    let led5 = gpioa.pa8.into_push_pull_output();
    let led6 = gpiob.pb15.into_push_pull_output();
    let charge_indicator = ChargeIndicator {
        led1, led2, led3, led4, led5, led6
    };

    // Communication with charger
    let _charger_comm = gpioc.pc13.into_analog();
    // Reset for UWB module (hack)
    let _uwb_reset = gpiob.pb10.into_push_pull_output();


    // Power switches and DC-DCs
    // use PowerBlockType::*;
    // use PowerBlockId::*;
    // Be careful with the map size!
    // let mut power_blocks = config::PowerBlocksMap::new();

    // Go back to standby mode if button is not pressed (woken by watchdog)
    let button = gpioa.pa15.into_floating_input();
    // if button.is_high().unwrap() {
    //     watchdog.feed();
    //     writeln!(rtt, "Back to sleep").ok();
    //     pwr.standby_mode(&mut scb).enter();
    // }
// let x = AnyPin::new(dcdc_5v0_bms_en);
    // Initialize RTC, this date will only be used if RTC is not yet initialised
    let instant = Instant::new()
        .set_year(21)
        .set_month(1)
        .set_day(29)
        .set_hour(0)
        .set_minute(0)
        .set_second(0);
    let _rtc = RTC::new(device.RTC, &mut rcc, RtcClockSource::LSI, &mut pwr, instant);

    // Initialize allocator
    // Used only in init to store PowerBlock trait objects in array
    // let start = cortex_m_rt::heap_start() as usize;
    // writeln!(rtt, "heap: {}", start);
    // let size = 512; // in bytes
    // unsafe { ALLOCATOR.init(start, size) }

    // +5V0_BMS & +3V3_BMS (Self power, ORed with BQ76920 +3V3_AFE)
    let mut ps_5v0_bms_en = gpioa.pa3.into_push_pull_output();
    ps_5v0_bms_en.set_high().ok();
    // +5V0_S0 switch (CAN transceiver, )
    let switch_5v0_s0_en = gpioa.pa5.into_push_pull_output();
    // +3V3_S0 switch (MCP25625, )
    let switch_3v3_s0_en = gpioa.pa4.into_push_pull_output();
    // +5V0_AUX switch (Buzzer, )
    let switch_5v0_aux_en = gpiob.pb0.into_push_pull_output();
    cortex_m::asm::delay(ms2cycles_raw!(clocks, 10));

    // Enable IRQ on power button input
    let button_line = GpioLine::from_raw_line(button.pin_number()).unwrap();
    exti.listen_gpio(&mut syscfg, button.port(), button_line, TriggerEdge::Falling);

    // BQ76920 wake pin
    let mut afe_wake_pin = gpiob.pb4.into_push_pull_output();
    afe_wake_pin.set_high().ok();
    cortex_m::asm::delay(100_000);
    afe_wake_pin.set_low().ok();

    // ADC channels
    let pack_div_en_pin = gpiob.pb9.into_push_pull_output();
    let pack_div_pin = gpioa.pa6.into_analog();
    let bat_div_en_pin = gpioa.pa2.into_push_pull_output();
    let bat_div_pin = gpioa.pa7.into_analog();
    let mut adc = device.ADC.constrain(&mut rcc);
    adc.set_precision(Precision::B_12);
    adc.set_sample_time(SampleTime::T_160_5);

    // SPI<->CANBus MCP25625T-E/ML
    // GPIO init
    let mcp25625_irq         = gpioa.pa10;//.into_pull_up_input();
    let mcp_int_line = GpioLine::from_raw_line(mcp25625_irq.pin_number()).unwrap();
    exti.listen_gpio(&mut syscfg, mcp25625_irq.port(), mcp_int_line, TriggerEdge::Falling);
    // let mcp_stby         = gpioa.pa0; // NORMAL mode is low, SLEEP is high
    let mcp_cs         = gpioa.pa9;
    // let mcp_sck         = gpiob.pb3;
    // let mcp_miso         = gpioa.pa11;
    // let mcp_mosi         = gpioa.pa12;
    // let mut mcp_stby = mcp_stby.into_push_pull_output();
    // mcp_stby.set_low().ok();
    let mut mcp_cs = mcp_cs.into_push_pull_output();
    mcp_cs.set_low().ok();
    let mcp25625_parts = crate::tasks::canbus::Mcp25625Parts {
        cs: mcp_cs,
        sck: gpiob.pb3,
        miso: gpioa.pa11,
        mosi: gpioa.pa12,
        spi: device.SPI1,
        irq: mcp25625_irq
    };
    let mcp25625_state = crate::tasks::canbus::Mcp25625State::PoweredDown(Some(mcp25625_parts));
    // Queues
    let can_tx = config::CanTX::new();
    let can_rx = config::CanRX::new();

    // Power control
    // Enable predischarge MOSFET with current inrush limiting
    let mut predischarge_en = gpioa.pa1.into_push_pull_output();
    predischarge_en.set_low().ok();
    // Override CHG line from AFE to gate driver
    let mut afe_chg_override = gpioh.ph1.into_push_pull_output();
    afe_chg_override.set_low().ok();
    // Override DSG line from AFE to gate driver
    let mut afe_dsg_override = gpioh.ph0.into_push_pull_output();
    afe_dsg_override.set_low().ok();
    // Disable zero voltage charging, enabled by default (depletion FET)
    let zvchg_disable_pin = gpiob.pb8.into_push_pull_output();
    // let mut precharge_enable = gpiob.pb8.into_push_pull_output();
    // precharge_enable.set_high().ok();
    let afe_io = tasks::bms::AfeIo {
        afe_wake_pin,
        dcdc_en_pin: ps_5v0_bms_en,
        pack_div_en_pin,
        pack_div_pin,
        bat_div_en_pin,
        bat_div_pin,
        afe_chg_override,
        afe_dsg_override,
        precharge_control_pin: zvchg_disable_pin,
        switch_5v0_s0_en,
        switch_3v3_s0_en,
        switch_5v0_aux_en,
    };


    // Internal I2C to AFE and Gauge
    let scl = gpiob.pb6.into_open_drain_output();
    let sda = gpiob.pb7.into_open_drain_output();
    cfg_if! {
            if #[cfg(feature = "bitbang-i2c")] {
                let i2c_timer = device.TIM6.timer(200.khz(), &mut rcc);
                let mut i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, i2c_timer);
            } else {
                let mut i2c = device.I2C1.i2c(sda, scl, 100.khz(), &mut rcc);
            }
        }
    // Prepare bq76920 config
    let mut bq76920 = BQ769x0::new(0x08, config::CELL_COUNT as u8).unwrap();
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

    cx.spawn.watchdog().unwrap();
    cx.spawn.bms_event(tasks::bms::BmsEvent::CheckAfe).unwrap();
    cx.spawn.bms_event(tasks::bms::BmsEvent::CheckBalancing).unwrap();
    cx.spawn.api(tasks::api::Event::SendHeartbeat).unwrap();
    cx.spawn.blinker(tasks::led::Event::Toggle).unwrap();
    cx.spawn.canctrl_event(tasks::canbus::Event::BringDownWithPeriodicBringUp).ok();
    rtic::pend(config::BUTTON_IRQ); // if we got to this point, button was pressed

    let bms_state = tasks::bms::BmsState::default();
    let button_state = tasks::button::ButtonState::default();
    let softoff_state = tasks::softoff::State::default();

     crate::init::LateResources {
        clocks,
        rcc,
        watchdog,
        bms_state,
        afe_io,
        adc,
        status_led,
        rtt,
        mcp25625_state,
        can_tx,
        can_rx,
        i2c,
        bq76920,
        // power_blocks,
        exti,
        button,
        button_state,
        softoff_state,
        charge_indicator,
        buzzer_pwm_channel,

    }
}