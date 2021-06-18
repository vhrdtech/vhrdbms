use crate::{config, tasks};
use bq769x0::{MilliVolts, MilliAmperes};
use core::fmt::Write;
use mcu_helper::tim_cyccnt::U32Ext;
//use crate::hal::prelude::_embedded_hal_adc_OneShot;
// use jlink_rtt::NonBlockingOutput;
use stm32l0xx_hal::prelude::*;
//use mcu_helper::color;
use mcu_helper::color;
use bq769x0::{SCDDelay, Amperes, OCDDelay, MicroOhms, UVDelay, OVDelay};
use bq769x0::Config as BQ769x0Config;
// use crate::power_block::PowerBlockId;
use crate::util::EventSource;

// #[derive(Debug)]
// pub struct PowerRailCommand {
//     pub rail: PowerBlockId,
//     pub powered: bool,
//     pub delay_after: Option<MicroSeconds>,
// }

#[derive(Debug)]
pub enum BmsEvent {
    TogglePower(EventSource),
    PowerOff(EventSource),
    PowerOn(EventSource),
    CheckAfe,
    CheckBalancing,
    Halt,
    // PowerRailControl(PowerRailCommand)
}

#[derive(Default)]
pub struct BmsState {
    power_enabled: bool,
    precharge_enabled: bool,
    charge_enabled: bool,

    //charge_enabled: bool,
    //vchg_prev_mv: u32,

    // balancing_state: BalancingState,
    afe_fault_count: u8,
    power_on_fault_count: u8
}

#[derive(Debug)]
pub enum Error {
    AfeError(bq769x0::Error),
    DischargeFailed

}
impl From<bq769x0::Error> for Error {
    fn from(afe_err: bq769x0::Error) -> Self {
        Error::AfeError(afe_err)
    }
}

pub struct AfeIo {
    pub afe_wake_pin: config::AfeWakePin,
    pub dcdc_en_pin: config::DcDcEnPin,
    pub pack_div_en_pin: config::PackDivEnPin,
    pub pack_div_pin: config::PackDivPin,
    pub bat_div_en_pin: config::BatDivEnPin,
    pub bat_div_pin: config::BatDivPin,
    pub afe_chg_override: config::AfeChgOverridePin,
    pub afe_dsg_override: config::AfeDsgOverridePin,
    pub precharge_control_pin: config::ZvchgDisablePin,
    pub switch_5v0_s0_en: config::Switch5V0S0Pin,
    pub switch_3v3_s0_en: config::Switch3V3S0Pin,
    pub switch_5v0_aux_en: config::Switch5V0AuxPin,
}

impl AfeIo {
    pub fn enable_voltage_dividers(&mut self) {
        self.pack_div_en_pin.set_high().ok();
        self.bat_div_en_pin.set_high().ok();
    }

    pub fn disable_voltage_dividers(&mut self) {
        self.pack_div_en_pin.set_low().ok();
        self.bat_div_en_pin.set_low().ok();
    }

    pub fn enable_s0_switches(&mut self) {
        self.switch_3v3_s0_en.set_high().ok();
        self.switch_5v0_s0_en.set_high().ok();
    }

    pub fn is_s0_switches_enabled(&self) -> bool {
        self.switch_3v3_s0_en.is_high().unwrap()
    }

    pub fn disable_s0_switches(&mut self) {
        self.switch_3v3_s0_en.set_low().ok();
        self.switch_5v0_s0_en.set_low().ok();
    }
}

// enum BalancingStage {
//     /// Find cells with delta from lower >= config::BALANCE_START_DELTA_MV
//     /// If at least one found -> Phase1, else -> CheckStart
//     CheckStart,
//     /// Enable balancing for the first set of cells
//     Phase1,
//     // Enable balancing for the second set of cells (if not empty)
//     // Phase2,
// }
// impl Default for BalancingStage {
//     fn default() -> Self {
//         BalancingStage::CheckStart
//     }
// }
//
// struct BalancingState {
//     stage: BalancingStage,
//     phase1: u16,
//     phase2: u16,
// }
// impl Default for BalancingState {
//     fn default() -> Self {
//         BalancingState {
//             stage: BalancingStage::default(),
//             phase1: 0,
//             phase2: 0
//         }
//     }
// }

pub enum CellCount {
    _3S = 3,
    _4S = 4,
    _5S = 5,
}
impl CellCount {
    pub fn cells_mask(&self) -> u16 {
        match self {
            CellCount::_3S => { 0b10011 }
            CellCount::_4S => { 0b10111 }
            CellCount::_5S => { 0b11111 }
        }
    }
}

pub fn afe_init(
    i2c: &mut config::InternalI2c,
    bq769x0: &mut config::BQ769x0,
) -> Result<bq769x0::CalculatedValues, Error> {
    let bq769x0_config = BQ769x0Config {
        shunt: MicroOhms(2000),
        scd_delay: SCDDelay::_400uS,
        scd_threshold: Amperes(100),
        ocd_delay: OCDDelay::_640ms,
        ocd_threshold: Amperes(50),
        uv_delay: UVDelay::_4s,
        uv_threshold: config::CELL_UV_THRESHOLD,
        ov_delay: OVDelay::_4s,
        ov_threshold: config::CELL_OV_THRESHOLD
    };
    let values = bq769x0.init(i2c, &bq769x0_config).map_err(|e| Error::AfeError(e))?;
    bq769x0.enable_adc(i2c, true)?;
    bq769x0.coulomb_counter_mode(i2c, bq769x0::CoulombCounterMode::Continuous)?;
    bq769x0.discharge(i2c, false)?;
    bq769x0.charge(i2c, false)?;
    Ok((values))
}

pub fn afe_discharge(
    i2c: &mut config::InternalI2c,
    bq769x0: &mut config::BQ769x0,
    enable: bool
) -> Result<(), Error> {
    if !enable {
        bq769x0.discharge(i2c, false)?;
        return Ok(());
    }
    for _ in 0..10 {
        bq769x0.sys_stat_reset(i2c, bq769x0::SysStat::SHORTCIRCUIT | bq769x0::SysStat::OVERCURRENT)?;
        bq769x0.discharge(i2c, true)?;
        cortex_m::asm::delay(400_000);
        let stat = bq769x0.sys_stat(i2c)?;

        if !stat.scd_is_set() {
            return Ok(());
        }

        cortex_m::asm::delay(100_000);
    }
    Err(Error::DischargeFailed)
}

pub fn bms_event(cx: crate::bms_event::Context, e: tasks::bms::BmsEvent) {
    let bms_state: &mut BmsState = cx.resources.bms_state;
    let i2c: &mut config::InternalI2c = cx.resources.i2c;
    let bq769x0: &mut config::BQ769x0 = cx.resources.bq76920;
    let rtt = cx.resources.rtt;
    let clocks = cx.resources.clocks;
    let rcc = cx.resources.rcc;
    let adc = cx.resources.adc;
    let afe_io: &mut AfeIo = cx.resources.afe_io;
    let can_tx = cx.resources.can_tx;
    let mcp25625_state = cx.resources.mcp25625_state;

    writeln!(rtt, "p_en: {}", bms_state.power_enabled).ok();

    match e {
        BmsEvent::TogglePower(source) => {
            if bms_state.power_enabled {
                if config::SOFTOFF_TIMEOUT_MS == 0 {
                    cx.spawn.bms_event(BmsEvent::PowerOff(source)).ok();
                } else {
                    writeln!(rtt, "Softoff spawned").ok();
                    cx.spawn.softoff().ok();
                }
            } else {
                cx.spawn.bms_event(BmsEvent::PowerOn(source)).ok();
            }
        },
        BmsEvent::PowerOff(source) => {
            // if !bms_state.power_enabled {
            //     return;
            // }
            if bms_state.charge_enabled && config::IS_SEPARATE_CHG_PATH == false {
                writeln!(rtt, "Ignoring off due to charge").ok();
                bms_state.power_enabled = false;
                return;
            }
            let _ = afe_discharge(i2c, bq769x0, false); // TODO: bb
            if config::IS_SEPARATE_CHG_PATH == false {
                match bq769x0.charge(i2c, false) {
                    Ok(_) => { writeln!(rtt, "CHG fet off").ok(); }
                    Err(_) => {}
                }
            }
            bms_state.power_enabled = false;
            writeln!(rtt, "Power disabled").ok();
            cx.spawn.blinker(crate::tasks::led::Event::OffMode).ok();
            if source.need_to_forward() {
                crate::tasks::api::broadcast_power_control_frame(can_tx, false);
            }
            cx.schedule.canctrl_event(cx.scheduled + ms2cycles!(clocks, 100), crate::tasks::canbus::Event::BringDownWithPeriodicBringUp).ok();
        },
        BmsEvent::PowerOn(source) => {
            if bms_state.power_enabled {
                writeln!(rtt, "Already on").ok();
                return;
            }
            // crate::board_components::imx_prepare_boot(i2c, rtt);
            let r = afe_discharge(i2c, bq769x0, true);
            let dsg_successful = r.is_ok();
            writeln!(rtt, "DSG successful?: {:?}", r).ok();
            if dsg_successful {
                // crate::power_block::enable_all(power_blocks);
                if config::IS_SEPARATE_CHG_PATH == false {
                    match bq769x0.charge(i2c, true) {
                        Ok(_) => { writeln!(rtt, "CHG fet on").ok(); }
                        Err(_) => {}
                    }
                }

                if source != EventSource::LocalNoStateChangeNoForward {
                    bms_state.power_enabled = true;
                }
                bms_state.power_on_fault_count = 0;
                cx.spawn.blinker(crate::tasks::led::Event::OnMode).ok();
                cx.spawn.canctrl_event(crate::tasks::canbus::Event::BringUp).ok();
                if source.need_to_forward() {
                    use crate::tasks::api::{Event, PowerBurstContext};
                    cx.spawn.api(Event::SendPowerOnBurst(PowerBurstContext::new())).ok();
                }
            } else {
                bms_state.power_on_fault_count += 1;
                if bms_state.power_on_fault_count > 5 {
                    bms_state.power_enabled = false;
                }
            }
        }
        BmsEvent::CheckAfe => {
            let sys_stat = match bq769x0.sys_stat(i2c) {
                Ok(s) => { s },
                Err(e) => {
                    use bq769x0::Error;
                    match e {
                        Error::CRCMismatch | Error::Uninitialized | Error::I2CError | Error::VerifyError(_) => {
                            writeln!(rtt, "{}AFE problem detected: {:?}{}", color::RED, e, color::DEFAULT).ok();
                            afe_io.afe_wake_pin.set_high().ok();
                            delay_ms!(clocks, 300);
                            afe_io.afe_wake_pin.set_low().ok();
                            match afe_init(i2c, bq769x0) {
                                Ok(_) => {
                                    writeln!(rtt, "AFE reinit ok").ok();
                                    bms_state.afe_fault_count = 0;
                                },
                                Err(e) => {
                                    writeln!(rtt, "AFE reinit fail: {:?}", e).ok();
                                    bms_state.afe_fault_count += 1;
                                    if bms_state.afe_fault_count > config::AFE_FAULT_COUNT_TO_HALT {
                                        writeln!(rtt, "Giving up").ok();
                                        cx.spawn.bms_event(BmsEvent::Halt).ok();
                                    }
                                }
                            }
                        },
                        _ => {}
                    }
                    return;
                }
            };
            // match bq769x0.is_charge_enabled(i2c) { // TODO: maybe remove this and check for OV directly
            //     Ok(chg) => {
            //         if !chg {
            //             match bq769x0.sys_stat(i2c) {
            //                 Ok(stat) => {
            //                     if stat.overvoltage_is_set() {
            //                         let sufficiently_charged_cells = find_cells(i2c, bq769x0, |cell, _| {
            //                             cell > config::CELL_OV_CLEAR
            //                         }).unwrap_or(0);
            //                         if sufficiently_charged_cells == 0 { // when highly unbalanced, this check is better than total pack value.
            //                             bq769x0.sys_stat_reset(i2c, bq769x0::SysStat::OVERVOLTAGE).ok();
            //                         }
            //                     } else {
            //                         bq769x0.charge(i2c, true).ok();
            //                         bq769x0.sys_stat_reset(i2c, bq769x0::SysStat::UNDERVOLTAGE).ok();
            //                     }
            //                 },
            //                 Err(_) => {}
            //             }
            //         }
            //     },
            //     Err(_) => {}
            // }
            if sys_stat.scd_is_set() | sys_stat.ocd_is_set() { // TODO: add max amount of restarts here?
                writeln!(rtt, "{}SCD/OCD detected{}", color::YELLOW, color::DEFAULT).ok();
                bq769x0.sys_stat_reset(i2c, bq769x0::SysStat::SHORTCIRCUIT | bq769x0::SysStat::OVERCURRENT).ok();
                if bms_state.power_enabled {
                    cx.spawn.bms_event(BmsEvent::PowerOn(EventSource::LocalNoForward)).ok();
                }
            }
            if sys_stat.undervoltage_is_set() {
                writeln!(rtt, "{}UV detected, shutting off{}", color::YELLOW, color::DEFAULT).ok();
                bq769x0.sys_stat_reset(i2c, bq769x0::SysStat::UNDERVOLTAGE).ok();
                // cx.spawn.bms_event(BmsEvent::PowerOff).ok();
                cx.spawn.bms_event(BmsEvent::Halt).ok();
            }
            if sys_stat.device_xready_is_set() {
                writeln!(rtt, "{}Xready detected, clearing{}", color::RED, color::DEFAULT).ok();
                bq769x0.sys_stat_reset(i2c, bq769x0::SysStat::DEVICE_XREADY).ok();
                bq769x0.sys_stat_reset(i2c, bq769x0::SysStat::DEVICE_XREADY).ok();
            }

            let (afe_pack_voltage, pack_current, cell_voltages) = match get_v_i_cells(i2c, bq769x0) {
                Ok((pack_voltage, pack_current, cell_voltages)) => {
                    (pack_voltage, pack_current, cell_voltages)
                },
                Err(e) => {
                    writeln!(rtt, "{}V/I/C read err {:?}{}", color::RED, e, color::DEFAULT).ok();
                    return;
                }
            };

            let min_cell = *cell_voltages.iter().min().unwrap_or(&bq769x0::MilliVolts(0));
            let max_cell = *cell_voltages.iter().max().unwrap_or(&bq769x0::MilliVolts(0));
            if min_cell < config::CELL_SOFT_UV_THRESHOLD {
                writeln!(rtt, "{}Soft undervoltage{}", color::YELLOW, color::DEFAULT).ok();
                cx.spawn.softoff().ok();
            }

            let telemetry = crate::tasks::api::Telemetry {
                pack_voltage: afe_pack_voltage,
                pack_current,
                cell_voltages
            };
            crate::tasks::api::send_telemetry(can_tx, &telemetry);

            // Enable voltage dividers and give some time for input to stabilise
            afe_io.enable_voltage_dividers();
            delay_ms!(clocks, 10);
            use crate::util::{MilliVolts, resistor_divider_inverse};
            let rdiv_bat_voltage = (adc.read(&mut afe_io.bat_div_pin) as Result<u16, _>).map(|v| adc.to_millivolts(v)).unwrap_or(0);
            let rdiv_bat_voltage = resistor_divider_inverse(config::BAT_DIV_RT, config::BAT_DIV_RB, MilliVolts(rdiv_bat_voltage as i32));

            let rdiv_pack_voltage = (adc.read(&mut afe_io.pack_div_pin) as Result<u16, _>).map(|v| adc.to_millivolts(v)).unwrap_or(0);
            let rdiv_pack_voltage = resistor_divider_inverse(config::PACK_DIV_RT, config::PACK_DIV_RB, MilliVolts(rdiv_pack_voltage as i32));
            let pack_bat_diff = rdiv_pack_voltage - rdiv_bat_voltage;
            writeln!(rtt, "b:{} p:{} d:{}", rdiv_bat_voltage, rdiv_pack_voltage, pack_bat_diff).ok();
            // afe_io.disable_voltage_dividers();

            if max_cell >= config::CELL_SOFT_OV_THRESHOLD {
                writeln!(rtt, "{}Soft overvoltage{}", color::RED, color::DEFAULT).ok();
                #[cfg(feature = "zvhcg-fet-present")]
                afe_io.precharge_control_pin.set_high().ok(); // disable zvchg depletion fet
                #[cfg(feature = "precharge-fet-present")]
                afe_io.precharge_control_pin.set_low().ok(); // disable precharge fet
                /// TODO: set SOV flag, keep CHG fet only if current is negative
                bq769x0.charge(i2c, false); // chg fet will overheat if too much current is flowing due to body diode conducting
                bms_state.precharge_enabled = false;
                bms_state.charge_enabled = false;
            } else if pack_bat_diff >= config::CHARGER_DETECTION_THRESHOLD && max_cell < config::CHARGE_START_MIN_VOLTAGE && bms_state.power_enabled == false {
                if min_cell < config::PRECHARGE_THRESHOLD {
                    writeln!(rtt, "Precharge (too low v)").ok();
                    #[cfg(feature = "precharge-fet-present")]
                    afe_io.precharge_control_pin.set_high().ok();
                    bms_state.precharge_enabled = true;
                } else {
                    if bms_state.charge_enabled == false {
                        writeln!(rtt, "Turning on due to charger present").ok();
                        cx.spawn.bms_event(BmsEvent::PowerOn(EventSource::LocalNoStateChangeNoForward)).ok();
                        bms_state.charge_enabled = true;
                    }
                }
            } else {
                if bms_state.charge_enabled && pack_current < config::CHARGE_STOP_CURRENT {
                    writeln!(rtt, "Charger removed?").ok();
                    if !bms_state.power_enabled {
                        cx.spawn.bms_event(BmsEvent::PowerOff(EventSource::LocalNoStateChangeNoForward)).ok();
                    }
                    bms_state.charge_enabled = false;
                    bms_state.precharge_enabled = false;
                }
            }

            // if bms_state.charge_enabled {
            //     bms_state.charge_enabled = bq769x0.is_charging(i2c).unwrap_or(false);
            //     if !bms_state.charge_enabled {
            //         writeln!(rtt, "OV reached").ok();
            //     }
            //     match bq769x0.current(i2c) {
            //         Ok(i) => {
            //             if i.0 < 100 {
            //                 let r = bq769x0.charge(i2c, false);
            //                 writeln!(rtt, "Charge finish: {:?}", r).ok();
            //                 bms_state.charge_enabled = false;
            //             }
            //         },
            //         Err(_) => {}
            //     }
            // } else {
            //     tca9534.write_config(i2c, tca9535::tca9534::Port::empty()).ok(); // Fix glitched reset?
            //     let r =     tca9534.modify_outputs(i2c, config::TCA_VHCG_DIV_EN_PIN, config::TCA_VHCG_DIV_EN_PIN);
            //     if r.is_err() {
            //         writeln!(rtt, "{}vchg tca err{}", color::RED, color::DEFAULT).ok();
            //     }
            //     cortex_m::asm::delay(100_000);
            //     let vchg_raw = adc.read(&mut afe_io.vchg_div_pin) as Result<u32, _>;
            //     let vchg_raw = vchg_raw.unwrap_or(0);
            //     tca9534.modify_outputs(i2c, config::TCA_VHCG_DIV_EN_PIN, Port::empty()).ok();
            //     let vchg = (vchg_raw * 8 * 3300) / 1 / 4095;
            //     writeln!(rtt, "vchg: {}mV", vchg).ok();
            //     if bms_state.vchg_prev_mv != 0 {
            //         let dvchg_mv = vchg as i32 - bms_state.vchg_prev_mv as i32;
            //         if dvchg_mv > 200 {
            //             let r = bq769x0.charge(i2c, true);
            //             writeln!(rtt, "Charger plugged, en:{:?}", r).ok();
            //             bms_state.charge_enabled = true;
            //         }
            //     }
            //     bms_state.vchg_prev_mv = vchg;
            // }
            cx.schedule.bms_event(cx.scheduled + ms2cycles!(clocks, config::CHARGER_CHECK_INTERVAL_MS), BmsEvent::CheckAfe).ok();
        },
        BmsEvent::CheckBalancing => {
            // let _ = check_balancing(i2c, bq769x0, &mut bms_state.balancing_state, rtt);
            // cx.schedule.bms_event(cx.scheduled + ms2cycles!(clocks, config::BALANCING_CHECK_INTERVAL_MS), BmsEvent::CheckBalancing).ok();
        },
        BmsEvent::Halt => {
            match bq769x0.ship_enter(i2c) {
                Ok(_) => {
                    writeln!(rtt, "Halt: ok").ok();
                    afe_io.dcdc_en_pin.set_low().ok();
                },
                Err(e) => {
                    writeln!(rtt, "Halt: {:?}", e).ok();
                }
            }
        },
        // BmsEvent::PowerRailControl(cmd) => {
        //     if !bms_state.power_enabled {
        //         return;
        //     }
        //     let pb = power_blocks.get_mut(&cmd.rail).expect("I1");
        //     if cmd.powered {
        //         pb.enable();
        //     } else {
        //         pb.disable();
        //     }
        //     match cmd.delay_after {
        //         Some(delay) => {
        //             cortex_m::asm::delay(us2cycles_raw!(clocks, delay.0) as u32);
        //         },
        //         None => {}
        //     }
        // }
    }
}

fn get_v_i_cells<'a>(i2c: &mut config::InternalI2c, bq769x0: &'a mut config::BQ769x0)
    -> Result<(MilliVolts, MilliAmperes, &'a [MilliVolts]), bq769x0::Error>
{
    let v = bq769x0.voltage(i2c)?;
    let i = bq769x0.current(i2c)?;
    let cells = bq769x0.cell_voltages(i2c)?;
    Ok((v, i, cells))
}

fn find_cells<F: Fn(MilliVolts, MilliVolts) -> bool>(
    // i2c: &mut config::InternalI2c,
    // bq769x0: &mut config::BQ769x0,
    cells: &[MilliVolts],
    f: F
) -> u16 {
    // let cells = bq769x0.cell_voltages(i2c)?;
    let low_cell = *cells.iter().min().unwrap();
    let cells = cells
        .iter()
        .map(|cell| f(*cell, low_cell))
        .enumerate().map(|(i, c)| (c as u16) << i)
        .fold(0u16, |acc, cell_mask| acc | cell_mask);
    cells
}

// fn check_balancing(
//     i2c: &mut config::InternalI2c,
//     bq769x0: &mut config::BQ769x0,
//     state: &mut BalancingState,
//     rtt: &mut NonBlockingOutput
// ) -> Result<(), Error> {
//     state.stage = match state.stage {
//         BalancingStage::CheckStart => {
//             let balance_phases = find_cells(i2c, bq769x0, |cell, low_cell| {
//                 (cell.0 - low_cell.0) > config::BALANCE_START_DELTA_MV
//             })?;
//             let balance_phases = (balance_phases & 0b00101010_10101010, balance_phases & 0b01010101_01010101);
//             if balance_phases.0 != 0 || balance_phases.1 != 0 {
//                 writeln!(rtt, "CheckStart {:05b} {:05b}", balance_phases.0, balance_phases.1).ok();
//                 state.phase1 = balance_phases.0;
//                 state.phase2 = balance_phases.1;
//                 BalancingStage::Phase1
//             } else {
//                 writeln!(rtt, "Balanced").ok();
//                 BalancingStage::CheckStart
//             }
//         }
//         BalancingStage::Phase1 => {
//             if state.phase1 != 0 {
//                 // writeln!(rtt, "Phase1 {:05b}", state.phase1).ok();
//                 bq769x0.enable_balancing(i2c, state.phase1 as u8)?;
//             }
//             BalancingStage::CheckStart
//         }
//         // BalancingStage::Phase2 => {
//         //     if state.phase2 != 0 {
//         //         // writeln!(rtt, "Phase2 {:05b}", state.phase2).ok();
//         //         bq769x0.enable_balancing(i2c, state.phase2 as u8)?;
//         //     }
//         //     let stop_balancing = find_cells(i2c, bq769x0, |cell, low_cell| {
//         //         (cell.0 - low_cell.0) < config::BALANCE_STOP_DELTA_MV
//         //     })?;
//         //     // writeln!(rtt, "CheckStop {:05b} {:05b}", stop_balancing.0, stop_balancing.1).ok();
//         //     state.phase1 &= !stop_balancing.0;
//         //     state.phase2 &= !stop_balancing.1;
//         //     if state.phase1 != 0 || state.phase2 != 0 {
//         //         BalancingStage::Phase1
//         //     } else {
//         //         bq769x0.enable_balancing(i2c, 0)?;
//         //         BalancingStage::CheckStart
//         //     }
//         // }
//     };
//
//     //let max_delta = cells.iter().map(|cell| *cell - low_cell).max().unwrap();
//
//     // let mut i = 1;
//     // for needs_balancing in unbalanced_cells {
//     //     writeln!(rtt, "{}: {}", i, needs_balancing);
//     //     i += 1;
//     // }
//     // writeln!(rtt, "max_delta: {}", max_delta).ok();
//     Ok(())
// }