#![no_std]
#![no_main]
//#![feature(const_generics)]

//extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
//extern crate panic_rtt;
#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    interrupt::disable();

    let mut out = jlink_rtt::Output::new();
    writeln!(out, "PANIC: {}", info).ok();

    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

// use cortex_m::asm;
use cortex_m_rt::entry;
use stm32l0xx_hal::{
    pac,
    prelude::*,
    rcc::Config,
    pwr::PWR,
    syscfg::SYSCFG,
    exti,
    pwr,
    spi::{MODE_0},
    adc::Adc,

};

use embedded_hal::blocking::spi::{Write, Transfer};

use core::fmt::{Write as FmtWrite};
use jlink_rtt;
use jlink_rtt::try_read as rtt_try_read;

use crc_any::CRCu8;
use mcp25625::{MCP25625, MCP25625Config, CanAddress, McpOperationMode, McpReceiveBuffer, McpPriority, McpErrorKind};
use bq769x0::{BQ769x0, Config as BQ769x0Config, Error as BQ769x0Error, SCDDelay, OCDDelay, UVDelay, OVDelay, MicroOhms, Amperes, MilliVolts};
use stm32l0xx_hal::adc::{Precision, SampleTime};
use stm32l0xx_hal::rtc::{Instant, RTC};
use core::panic::PanicInfo;
use cortex_m::interrupt;
use core::sync::atomic;
use core::sync::atomic::Ordering;

fn mcp_init<E, SPI, CS>(mcp: &mut MCP25625<SPI, CS>) -> Result<(), McpErrorKind>
    where
        SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
        CS: OutputPin
{
    let mcp_config = MCP25625Config {
        brp: 0, // Fosc=16MHz
        prop_seg: 3,
        ph_seg1: 2,
        ph_seg2: 2,
        sync_jump_width: 2
    };
    mcp.apply_config(mcp_config)?;
    //writeln!(rtt, "mcp:apply_config:{:?}", r);
    mcp.masks_rxall()?;
    //writeln!(rtt, "mcp:masks_rxall:{:?}", r);
    mcp.rx_configure(true, false)?;
    //writeln!(rtt, "mcp:rx_conf:{:?}", r);
    mcp.change_mode(McpOperationMode::Normal)?;
    //writeln!(rtt, "mcp:req_normal:{:?}", r);
    Ok(())
}

struct DummyDog {

}

impl DummyDog {
    pub fn feed(&self) {

    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut rtt = jlink_rtt::NonBlockingOutput::new(false);
    rtt.write("\n=======\n\n");
    rtt.write("\n=======\n\n");
    //rtt.write("\x1B[2J\x1B[1;36;40mvhrdBMS\x1B[0m ");
    rtt.write("\x1B[1;36;40mvhrdBMS\x1B[0m ");
    rtt.write(env!("CARGO_PKG_VERSION"));
    rtt.write("\n=======\n\n");

    //let mut watchdog = dp.IWDG.watchdog();
    //watchdog.start(1000.ms());
    let watchdog = DummyDog {};

    let mut rcc = dp.RCC.freeze(Config::hsi16());
    let mut pwr   = PWR::new(dp.PWR, &mut rcc);
    let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
    let mut scb   = cp.SCB;

    let mut delay = cp.SYST.delay(rcc.clocks);

    // let bit = dp.USART1.cr1.read().tcie().bit_is_set();
    // dp.USART1.cr1.write(|w| w.tcie().set_bit());

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // Connector with LED and Button
    //let user_button     = gpioa.pa0;  // J8.2 through 560R+Zener, input only (rev.A pcb), weak pull up to 1P
    let user_button     = gpioa.pa11;
    let led             = gpioa.pa15; // J8.1 through 560R+Zener, LED on board

    let user_button = user_button.into_pull_up_input();
    let mut exti = dp.EXTI;
    exti.listen(
        &mut syscfg,
        user_button.port(),
         user_button.pin_number(),
        exti::TriggerEdge::Falling
    );


    let mut led = led.into_push_pull_output();

    let instant = Instant::new()
        .set_year(20)
        .set_month(05)
        .set_day(29)
        .set_hour(11)
        .set_minute(35)
        .set_second(30);

    let mut rtc = RTC::new(
        dp.RTC,
        &mut rcc,
        &mut pwr,
        instant,
    );

    // Voltage dividers
    // vbat can be read out from bq76930 as a sum of cell voltages or from bq34z100 from it's Rdiv
    let vdiv_en         = gpioa.pa10; // enable vcharge_div and vdischarge_div
    let vdischarge_div  = gpioa.pa1;  // from DSG connector to bq76200 to Rdiv, Rt=806k + Rfet=1.5..3.5k, Rb=16k5
    let vcharge_div     = gpioa.pa4;  // from CHG connector to FET switch to Rdiv, Rt=806k, Rb=16k5

    let mut vdiv_en = vdiv_en.into_push_pull_output();
    vdiv_en.set_low().ok();

    let mut adc = Adc::new(dp.ADC, &mut rcc);
    adc.set_precision(Precision::B_12);
    adc.set_sample_time(SampleTime::T_160_5);
    let mut vdischarge_div = vdischarge_div.into_analog();
    let mut vcharge_div = vcharge_div.into_analog();


    // SPI<->CANBus MCP25625T-E/ML
    let _mcp_int         = gpioa.pa2;
    let _mcp_tx0_rts     = gpioa.pa3;
    let _mcp_rx0_bf      = gpiob.pb4;
    let mcp_stby        = gpiob.pb5; // NORMAL mode is low, SLEEP is high
    let mcp_cs           = gpioa.pa12;
    let mcp_sck          = gpioa.pa5;
    let mcp_miso         = gpioa.pa6;
    let mcp_mosi         = gpioa.pa7;
    let mut mcp_stby = mcp_stby.into_push_pull_output();
    mcp_stby.set_low().ok();
    let mut mcp_cs = mcp_cs.into_push_pull_output();
    mcp_cs.set_high().ok();
    // let pins = (
    //     gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh), // sck
    //     gpiob.pb14.into_floating_input(&mut gpiob.crh), // miso
    //     gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh), // mosi
    // );
    let mcp_freq = 1.mhz();
    // let spi_mode = Mode {
    //     polarity: Polarity::IdleLow,
    //     phase: Phase::CaptureOnFirstTransition
    // };
    // let mut spi = Spi::spi2(
    //     dp.SPI2,
    //     pins,
    //     spi_mode,
    //     mcp_freq,
    //     clocks,
    //     &mut rcc.apb1
    // );
    let spi = dp
        .SPI1
        .spi((mcp_sck, mcp_miso, mcp_mosi), MODE_0, mcp_freq, &mut rcc);

    let one_cp: u32 = 1000; // tweak to be approx. one spi clock cycle
    watchdog.feed();
    delay.delay_ms(10u16);
    let mut mcp = MCP25625::new(spi, mcp_cs, one_cp);
    for i in 0..=10 {
        let r = mcp_init(&mut mcp);
        match r {
            Ok(()) => {
                let _ = writeln!(rtt, "mcp init ok {}", i);
                break;
            }
            Err(e) => {
                let _ = writeln!(rtt, "mcp init err: {:?}", e);
            }
        }
    }

    mcp.reset_interrupt_flags(0);
    mcp.reset_error_flags();

    // bq76200 high side driver
    let predischarge_en  = gpioa.pa8;  // Enable pre-discharge P-FET
    let _charge_pump_en  = gpioa.pa9;  // Enable charge pump before high side FETs (CHG and DSG)

    let mut predischarge_en = predischarge_en.into_push_pull_output();
    predischarge_en.set_low().ok();

    // bq769x0
    let afe_wake        = gpiob.pb0;  // To magical curcuit, can be replaced with user_button in rev.B
    let mut afe_wake = afe_wake.into_push_pull_output();
    afe_wake.set_high().ok();
    delay.delay_ms(50u16);
    afe_wake.set_low().ok();
    delay.delay_ms(100u16);
    let alert_afe       = gpiob.pb1;  // Can be input as an interrupt from bq769x0 or can be driven
    let alert_afe = alert_afe.into_floating_input();

    // Low Iq DC-DC to +5V_CAN and +3V3_MCU
    let dcdc_en         = gpiob.pb3;  // Keep enabled BEFORE putting bq769x0 to sleep
    let mut dcdc_en = dcdc_en.into_push_pull_output();
    dcdc_en.set_high().ok();

    // Internal I2C bus to bq34z100 and bq769x0
    let scl             = gpiob.pb6;
    let sda             = gpiob.pb7;

    let scl = scl.into_open_drain_output();
    let sda = sda.into_open_drain_output();
    let mut i2c = dp.I2C1.i2c(sda, scl, 100.khz(), &mut rcc);

    let mut crc8 = CRCu8::crc8();

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
    watchdog.feed();
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

    // crc8.digest(b"\x10\x04\x18");
    // let _ = writeln!(rtt, "crc: {:#04x}", crc8.get_crc());

    //dp.GPIOA.brr.write(|w| w.br0().set_bit());
    let mut pressed_count = 0u8;
    let mut long_press_detected = false;
    let mut buf = [0u8; 1];
    let mut discharge_enabled = false;
    let mut charge_enabled = false;
    match bq76920.read_raw(&mut i2c, 0x05, &mut buf) {
        Ok(_) => {
            discharge_enabled = buf[0] & 0b0000_0010 != 0;
            charge_enabled = buf[0] & 0b0000_0001 != 0;
        },
        Err(e) => { let _ = write!(rtt, "sysctrl2readerr: {:?}", e); },
    };
    let mut n = 30u32;
    //let mut sleep_timeout = 15u32;
    const CHARGE_DIFF_THRESHOLD: MilliVolts = MilliVolts(700);
    let mut vchg_prev = MilliVolts(0);
    watchdog.feed();
    loop {
        // Feed the wathdog on time.
        watchdog.feed();

        if discharge_enabled {
            led.set_high().ok();
            delay.delay_ms(500_u16);
            led.set_low().ok();
            delay.delay_ms(50_u16);
        } else {
            led.set_high().ok();
            delay.delay_ms(50_u16);
            led.set_low().ok();
            delay.delay_ms(500_u16);
        }
        watchdog.feed();

        match bq76920.sys_stat(&mut i2c) {
            Ok(stat) => {
                if !stat.is_ok() {
                    let _ = write!(rtt, "\nSYS_STAT not ok: {:?}", stat);
                    let r = bq76920.sys_stat_reset(&mut i2c);
                    let _ = write!(rtt, "SYS_STAT reset: {:?}", r);
                    charge_enabled = !stat.overvoltage_is_set(); // CHG is disabled
                    if discharge_enabled && (stat.ocd_is_set() || stat.scd_is_set()) {
                        delay.delay_ms(500_u32);
                        watchdog.feed();
                        delay.delay_ms(500_u32);
                        watchdog.feed();
                        delay.delay_ms(500_u32);
                        watchdog.feed();
                        delay.delay_ms(500_u32);
                        watchdog.feed();
                        predischarge_en.set_high().ok();
                        delay.delay_ms(500_u32);
                        watchdog.feed();
                        let r = bq76920.discharge(&mut i2c, true);
                        predischarge_en.set_low().ok();
                        let _ = write!(rtt, "DSG reEN: {:?}", r);
                    } else {
                        let _ = write!(rtt, "DSG reEN postponed");
                    }
                }
            },
            Err(e) => {
                let _ = write!(rtt, "SYS_STAT read err: {:?}", e);
            }
        }

        if !charge_enabled {
            vdiv_en.set_high().ok();
            delay.delay_ms(10u16);
            let vchg_raw = adc.read(&mut vcharge_div) as Result<u32, _>;
            let vchg_raw = vchg_raw.unwrap_or(0);
            vdiv_en.set_low().ok();
            let vchg = (vchg_raw * 997 * 3300) / 20 / 4095;
            //let _ = writeln!(rtt, "vchg: {}mV", vchg);

            if vchg_prev.0 == 0 {
                vchg_prev = MilliVolts(vchg);
            } else {
                let diff = vchg as i32 - vchg_prev.0 as i32;
                let _ = writeln!(rtt, "diff: {}mV", diff);
                vchg_prev = MilliVolts(vchg);
                if diff > CHARGE_DIFF_THRESHOLD.0 as i32 {
                    match bq76920.voltage(&mut i2c) {
                        Ok(afe_v) => {
                            if afe_v.0 < 20500 {
                                let _ = write!(rtt, "Enabling charge: ");
                                charge_enabled = true;
                                watchdog.feed();
                                delay.delay_ms(500_u32);
                                watchdog.feed();
                                delay.delay_ms(500_u32);
                                watchdog.feed();
                                delay.delay_ms(500_u32);
                                watchdog.feed();
                                delay.delay_ms(500_u32);
                                watchdog.feed();
                                match bq76920.charge(&mut i2c, true) {
                                    Ok(_) => {let _ = writeln!(rtt, "Ok");},
                                    Err(e) => {let _ = writeln!(rtt, "{:?}", e);},
                                }
                                if discharge_enabled {
                                    predischarge_en.set_high().ok();
                                    delay.delay_ms(500_u32);
                                    watchdog.feed();
                                    let r = bq76920.discharge(&mut i2c, true);
                                    predischarge_en.set_low().ok();
                                    let _ = write!(rtt, "Post_DSG reEN: {:?}", r);
                                }
                            }
                        }
                        Err(e) => {
                            let _ = writeln!(rtt, "vread err={:?}", e);
                        }
                    }
                }
            }
        } else {
            match bq76920.current(&mut i2c) {
                Ok(i) => {
                    if i.0 < 100 {
                        let _ = write!(rtt, "Disabling charge: ");
                        match bq76920.charge(&mut i2c, false) {
                            Ok(_) => {let _ = writeln!(rtt, "Ok");},
                            Err(e) => {let _ = writeln!(rtt, "{:?}", e);},
                        }
                        charge_enabled = false;
                    }
                }
                Err(e) => {
                    let _ = writeln!(rtt, "ireaderr={:?}", e);
                }
            }
        }




        // sleep_timeout = sleep_timeout - 1;
        // if sleep_timeout == 0 {
        //     let _ = write!(rtt, "Trying to go to sleep");
        //     sleep_timeout = 15;
        //     match bq76920.ship_enter(&mut i2c) {
        //         Ok(_) => {
        //             let _ = write!(rtt, "AFE in SHIP, going to Sleep...");
        //             exti.wait_for_irq(
        //                 user_button.pin_number(),
        //                 pwr.stop_mode(
        //                     &mut scb,
        //                     &mut rcc,
        //                     pwr::StopModeConfig {
        //                         ultra_low_power: true,
        //                     },
        //                 ),
        //             );
        //             let _ = write!(rtt, "Woken, waking AFE...");
        //             afe_wake.set_high().ok();
        //             delay.delay_ms(10_u16);
        //             afe_wake.set_low().ok();
        //             let _ = writeln!(rtt, "afe wake?");
        //         },
        //         Err(e) => {
        //             let _ = write!(rtt, "SHIP err: {:?}", e);
        //         },
        //     }
        // }

        let pressed = user_button.is_low().unwrap();
        let alert = alert_afe.is_high().unwrap();
        if pressed {
            //sleep_timeout = 15;
            pressed_count = pressed_count + 1;
            if pressed_count == 5 {
                pressed_count = 0;
                long_press_detected = true;
            }
            let _ = write!(rtt, "_");
        } else if alert {
            pressed_count = 0;
            let _ = write!(rtt, "a");
        } else {
            pressed_count = 0;
            let _ = write!(rtt, ".");
        }
        if n == 0 {
            n = 30;
            let _ = writeln!(rtt, "");
        } else {
            n = n - 1;
        }

        if long_press_detected {
            if discharge_enabled {
                let r = bq76920.discharge(&mut i2c, false);
                let _ = writeln!(rtt, "DSG DIS: {:?}", r);
                discharge_enabled = false;
            } else {
                predischarge_en.set_high().ok();
                delay.delay_ms(500u16);
                watchdog.feed();
                let r = bq76920.discharge(&mut i2c, true);
                let _ = writeln!(rtt, "DSG EN: {:?}", r);
                predischarge_en.set_low().ok();
                discharge_enabled = true;
            }
            long_press_detected = false;
        }


        let intf = mcp.interrupt_flags();
        //writeln!(rtt, "{:?}", intf);
        let _ = mcp.error_flags();
        //writeln!(rtt, "{:?}", errf);

        if intf.rx0if_is_set() {
            let _ = mcp.receive(McpReceiveBuffer::Buffer0);
            //writeln!(rtt, "rx0:{:?}", message);
        }
        if intf.rx1if_is_set() {
            let _ = mcp.receive(McpReceiveBuffer::Buffer1);
            //writeln!(rtt, "rx1:{:?}", message);
        }
        //let tec = mcp.tec();
        //let rec = mcp.rec();
        //writeln!(rtt, "tec:{}, rec:{}", tec, rec);

        mcp.reset_interrupt_flags(intf.bits);
        mcp.reset_error_flags();

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
            //writeln!(rtt, "{} tx buffers free", empty_tx_bufs);
        }


        let _ = mcp.send(CanAddress::extended(0x1769A5F3), &[0xaa, 0xbb, 0xcc, 0xdd], McpPriority::Low);
        //writeln!(rtt, "send: {:?}", r);

        let mut bytes = [0u8; 64];
        let bytes_readed = rtt_try_read(&mut bytes);
        if bytes_readed != 0 {
            let _ = writeln!(rtt, "");
            let cmd = [&(bytes[0] as char), &(bytes[1] as char)];
            match cmd {
                ['r', 'a'] => {
                    if !(bytes_readed == 8 || bytes_readed == 9) {
                        let _ = writeln!(rtt, "Syntax: ra fa 10");
                    } else {
                        let mut lnibble = bytes[4] - '0' as u8;
                        if lnibble > 9 {
                            lnibble = lnibble - 39;
                        }
                        let mut hnibble = bytes[3] - '0' as u8;
                        if hnibble > 9 {
                            hnibble = hnibble - 39;
                        }
                        let reg_addr = (hnibble << 4) | (lnibble);
                        let len = if bytes_readed == 8 {
                            bytes[6] - '0' as u8
                        } else {
                            (bytes[6] - '0' as u8) * 10 + bytes[7] - '0' as u8
                        };
                        if len > 8 {
                            let _ = writeln!(rtt, "Len must be <=8");
                        } else {
                            let mut buf = [0u8; 8];
                            let r = bq76920.read_raw(&mut i2c, reg_addr, &mut buf[0..len as usize]);
                            if r.is_ok() {
                                for i in 0..len as usize {
                                    let _ = writeln!(rtt, "{:#04x}={:#04x}={:08b}", reg_addr + i as u8, buf[i], buf[i]);
                                }
                            } else {
                                let _ = writeln!(rtt, "Read err={:?}", r.err());
                            }
                        }
                    }
                },
                ['a', 'v'] => {
                    match bq76920.cell_voltages(&mut i2c) {
                        Ok(cells) => {
                            for (i, cell) in cells.iter().enumerate() {
                                let _ = writeln!(rtt, "Cell {}: {}mV", i+1, cell);
                            }
                        }
                        Err(e) => {
                            let _ = writeln!(rtt, "Read err={:?}", e);
                        }
                    }
                },
                ['i', 'i'] => {
                    match bq76920.current(&mut i2c) {
                        Ok(i) => {
                            let _ = writeln!(rtt, "Current: {}", i);
                        }
                        Err(e) => {
                            let _ = writeln!(rtt, "Read err={:?}", e);
                        }
                    }
                },
                ['v', 'v'] => {
                    match bq76920.voltage(&mut i2c) {
                        Ok(v) => {
                            let _ = writeln!(rtt, "Voltage: {}", v);
                        }
                        Err(e) => {
                            let _ = writeln!(rtt, "Read err={:?}", e);
                        }
                    }
                },
                ['p', 'p'] => {
                    let i = bq76920.current(&mut i2c);
                    let v = bq76920.voltage(&mut i2c);
                    let p: Result<i32, BQ769x0Error> = i.and_then(|i| {
                        v.map(|v| i.0 * (v.0 as i32) / 1000)
                    });
                    match p {
                        Ok(p) => {
                            let intpart = p / 1000;
                            let fractpart = p - intpart * 1000;
                            let _ = writeln!(rtt, "Power: {}.{}W", intpart, fractpart.abs());
                        }
                        Err(e) => {
                            let _ = writeln!(rtt, "Read err={:?}", e);
                        }
                    }
                },
                ['t', 't'] => {
                    let _ = writeln!(rtt, "Now: {:?}", rtc.now());
                },
                ['d', 'v'] => {
                    vdiv_en.set_high().ok();
                    delay.delay_ms(10u16);
                    // k=0.02006 =approx 997/20
                    let vchg_raw = adc.read(&mut vcharge_div) as Result<u32, _>;
                    let vchg_raw = vchg_raw.unwrap_or(0);
                    delay.delay_ms(10u16);
                    let vdsg_raw = adc.read(&mut vdischarge_div) as Result<u32, _>;
                    let vdsg_raw = vdsg_raw.unwrap_or(0);
                    vdiv_en.set_low().ok();
                    let vchg = (vchg_raw * 997 * 3300) / 20 / 4095;
                    let vdsg = (vdsg_raw * 997 * 3300) / 20 / 4095;
                    let _ = writeln!(rtt, "Vchg: {}mV {}, Vdsg: {}mV {}", vchg, vchg_raw, vdsg, vdsg_raw);
                },
                ['d', 'x'] => {
                    vdiv_en.set_low().ok();
                    let _ = writeln!(rtt, "divs dis");
                },
                ['p', 'e'] => {
                    let _ = writeln!(rtt, "Pre DSG EN");
                    predischarge_en.set_high().ok();
                },
                ['p', 'd'] => {
                    let _ = writeln!(rtt, "Pre DSG low");
                    predischarge_en.set_low().ok();
                },
                ['s', 't'] => {
                    let mut buf = [0u8; 1];
                    // match i2c.write_read(0x08, &[0x00u8], &mut buf) {
                    //     Ok(_) => {
                    //         let _ = writeln!(rtt, "SYS_STAT: {:08b}", buf[0]);
                    //     },
                    //     Err(e) => {
                    //         let _ = writeln!(rtt, "err: {:?}", e);
                    //     }
                    // }
                    let r = bq76920.read_raw(&mut i2c, 0x00, &mut buf);
                    let _ = writeln!(rtt, "{:?} {:08b}", r, buf[0]);
                },
                ['s', 'r'] => {
                    let mut buf = [0u8; 4];
                    buf[0] = 0x08 << 1;
                    buf[1] = 0x00;
                    buf[2] = 0b0001_1111;
                    crc8.reset();
                    crc8.digest(&buf[0..3]);
                    buf[3] = crc8.get_crc();
                    match i2c.write(0x08, &buf[1..]) {
                        Ok(_) => {
                            let _ = writeln!(rtt, "STAT reset");
                        },
                        Err(e) => {
                            let _ = writeln!(rtt, "err: {:?}", e);
                        }
                    }
                }
                ['c', 'p'] => {
                    let r = bq76920.init(&mut i2c, &bq769x0_config);
                    let _ = writeln!(rtt, "ReApply config: {}", r.is_ok());
                }
                ['d', 'e'] => {
                    predischarge_en.set_high().ok();
                    watchdog.feed();
                    delay.delay_ms(500u16);
                    watchdog.feed();
                    let r = bq76920.discharge(&mut i2c, true);
                    let _ = writeln!(rtt, "DSG EN: {:?}", r);
                    predischarge_en.set_low().ok();
                },
                ['d', 'd'] => {
                    let r = bq76920.discharge(&mut i2c, false);
                    let _ = writeln!(rtt, "DSG DIS: {:?}", r);
                },
                ['c', 'e'] => {
                    let r = bq76920.charge(&mut i2c, true);
                    let _ = writeln!(rtt, "CHG EN: {:?}", r);
                },
                ['c', 'd'] => {
                    let r = bq76920.charge(&mut i2c, false);
                    let _ = writeln!(rtt, "CHG DIS: {:?}", r);
                },
                ['s','h'] => {
                    match bq76920.ship_enter(&mut i2c) {
                        Ok(_) => {
                            let _ = writeln!(rtt, "SHIP ok");
                        },
                        Err(e) => {
                            let _ = writeln!(rtt, "err: {:?}", e);
                        }
                    }
                },
                ['w', 'k'] => {
                    afe_wake.set_high().ok();
                    delay.delay_ms(100_u16);
                    afe_wake.set_low().ok();
                    let _ = writeln!(rtt, "afe wake?");
                },
                ['s', 'l'] => {
                    exti.wait_for_irq(
                        user_button.pin_number(),
                        pwr.stop_mode(
                            &mut scb,
                            &mut rcc,
                            pwr::StopModeConfig {
                                ultra_low_power: true,
                            },
                        ),
                    );
                },
                ['f', 'o'] => {
                    let _ = writeln!(rtt, "disable dc_dc");
                    dcdc_en.set_low().ok();
                },
                _ => {

                }
            }
        }

    }
}
