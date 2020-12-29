use crate::{config, tasks};
use bq769x0::{BQ769x0, MilliVolts};
use core::fmt::Write;
use mcu_helper::tim_cyccnt::U32Ext;
use tca9535::tca9534::{Tca9534, Port};
use crate::hal::prelude::_embedded_hal_adc_OneShot;
use jlink_rtt::NonBlockingOutput;
use stm32l0xx_hal::prelude::OutputPin;
use mcu_helper::color;

#[derive(Debug)]
pub enum BmsEvent {
    TogglePower,
    CheckCharger,
    CheckBalancing,
    Halt,
}

#[derive(Default)]
pub struct BmsState {
    power_enabled: bool,

    charge_enabled: bool,
    vchg_prev_mv: u32,

    balancing_state: BalancingState
}

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
    pub vchg_div_pin: config::VchgDivPin,
    pub dcdc_en_pin: config::DcDcEnPin,
}

enum BalancingStage {
    /// Find cells with delta from lower >= config::BALANCE_START_DELTA_MV
    /// If at least one found -> Phase1, else -> CheckStart
    CheckStart,
    /// Enable balancing for the first set of cells
    Phase1,
    // Enable balancing for the second set of cells (if not empty)
    // Phase2,
}
impl Default for BalancingStage {
    fn default() -> Self {
        BalancingStage::CheckStart
    }
}

struct BalancingState {
    stage: BalancingStage,
    phase1: u16,
    phase2: u16,
}
impl Default for BalancingState {
    fn default() -> Self {
        BalancingState {
            stage: BalancingStage::default(),
            phase1: 0,
            phase2: 0
        }
    }
}

pub fn afe_discharge(
    i2c: &mut config::InternalI2c,
    bq769x0: &mut BQ769x0,
    enable: bool
) -> Result<(), Error> {
    if !enable {
        bq769x0.discharge(i2c, false)?;
        return Ok(());
    }
    for _ in 0..10 {
        bq769x0.sys_stat_reset(i2c)?;
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
    let bq769x0: &mut BQ769x0 = cx.resources.bq76920;
    let rtt = cx.resources.rtt;
    let power_blocks = cx.resources.power_blocks;
    let tca9534: &mut Tca9534<config::InternalI2c> = cx.resources.tca9534;
    let clocks = cx.resources.clocks;
    let adc = cx.resources.adc;
    let afe_io: &mut AfeIo = cx.resources.afe_io;

    match e {
        BmsEvent::TogglePower => {
            if bms_state.power_enabled {
                crate::power_block::disable_all(power_blocks);
                let _ = afe_discharge(i2c, bq769x0, false); // TODO: bb
                bms_state.power_enabled = false;
                writeln!(rtt, "Power disabled").ok();
            } else {
                crate::board_components::imx_prepare_boot(i2c, tca9534, rtt);
                bms_state.power_enabled = afe_discharge(i2c, bq769x0, true).is_ok(); // TODO: bb
                writeln!(rtt, "Power enabled?: {}", bms_state.power_enabled).ok();
                if bms_state.power_enabled {
                    crate::power_block::enable_all(power_blocks);
                }
            }
        },
        BmsEvent::CheckCharger => {
            if bms_state.charge_enabled {
                bms_state.charge_enabled = bq769x0.is_charging(i2c).unwrap_or(false);
                if !bms_state.charge_enabled {
                    writeln!(rtt, "OV reached").ok();
                }
                match bq769x0.current(i2c) {
                    Ok(i) => {
                        if i.0 < 100 {
                            let r = bq769x0.charge(i2c, false);
                            writeln!(rtt, "Charge finish: {:?}", r).ok();
                            bms_state.charge_enabled = false;
                        }
                    },
                    Err(_) => {}
                }
            } else {
                tca9534.write_config(i2c, tca9535::tca9534::Port::empty()).ok(); // Fix glitched reset?
                let r =     tca9534.modify_outputs(i2c, config::TCA_VHCG_DIV_EN_PIN, config::TCA_VHCG_DIV_EN_PIN);
                if r.is_err() {
                    writeln!(rtt, "{}vchg tca err{}", color::RED, color::DEFAULT).ok();
                }
                cortex_m::asm::delay(100_000);
                let vchg_raw = adc.read(&mut afe_io.vchg_div_pin) as Result<u32, _>;
                let vchg_raw = vchg_raw.unwrap_or(0);
                tca9534.modify_outputs(i2c, config::TCA_VHCG_DIV_EN_PIN, Port::empty()).ok();
                let vchg = (vchg_raw * 8 * 3300) / 1 / 4095;
                writeln!(rtt, "vchg: {}mV", vchg).ok();
                if bms_state.vchg_prev_mv != 0 {
                    let dvchg_mv = vchg as i32 - bms_state.vchg_prev_mv as i32;
                    if dvchg_mv > 200 {
                        let r = bq769x0.charge(i2c, true);
                        writeln!(rtt, "Charger plugged, en:{:?}", r).ok();
                        bms_state.charge_enabled = true;
                    }
                }
                bms_state.vchg_prev_mv = vchg;
            }
            cx.schedule.bms_event(cx.scheduled + ms2cycles!(clocks, config::CHARGER_CHECK_INTERVAL_MS), BmsEvent::CheckCharger).ok();
        },
        BmsEvent::CheckBalancing => {
            let _ = check_balancing(i2c, bq769x0, &mut bms_state.balancing_state, rtt);
            cx.schedule.bms_event(cx.scheduled + ms2cycles!(clocks, config::BALANCING_CHECK_INTERVAL_MS), BmsEvent::CheckBalancing).ok();
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
        }
    }
}

fn find_cells<F: Fn(MilliVolts, MilliVolts) -> bool>(
    i2c: &mut config::InternalI2c,
    bq769x0: &mut BQ769x0,
    f: F
) -> Result<(u16, u16), Error> {
    let cells = bq769x0.cell_voltages(i2c)?;
    let low_cell = *cells.iter().min().unwrap();
    let unbalanced_cells = cells
        .iter()
        .map(|cell| f(*cell, low_cell))
        .enumerate().map(|(i, c)| (c as u16) << i)
        .fold(0u16, |acc, cell_mask| acc | cell_mask);
    let phase_1 = unbalanced_cells & 0b00101010_10101010;
    let phase_2 = unbalanced_cells & 0b01010101_01010101;
    Ok((phase_1, phase_2))
}

fn check_balancing(
    i2c: &mut config::InternalI2c,
    bq769x0: &mut BQ769x0,
    state: &mut BalancingState,
    rtt: &mut NonBlockingOutput
) -> Result<(), Error> {
    state.stage = match state.stage {
        BalancingStage::CheckStart => {
            let balance_phases = find_cells(i2c, bq769x0, |cell, low_cell| {
                (cell.0 - low_cell.0) > config::BALANCE_START_DELTA_MV
            })?;
            if balance_phases.0 != 0 || balance_phases.1 != 0 {
                writeln!(rtt, "CheckStart {:05b} {:05b}", balance_phases.0, balance_phases.1).ok();
                state.phase1 = balance_phases.0;
                state.phase2 = balance_phases.1;
                BalancingStage::Phase1
            } else {
                writeln!(rtt, "Balanced").ok();
                BalancingStage::CheckStart
            }
        }
        BalancingStage::Phase1 => {
            if state.phase1 != 0 {
                // writeln!(rtt, "Phase1 {:05b}", state.phase1).ok();
                bq769x0.enable_balancing(i2c, state.phase1 as u8)?;
            }
            BalancingStage::CheckStart
        }
        // BalancingStage::Phase2 => {
        //     if state.phase2 != 0 {
        //         // writeln!(rtt, "Phase2 {:05b}", state.phase2).ok();
        //         bq769x0.enable_balancing(i2c, state.phase2 as u8)?;
        //     }
        //     let stop_balancing = find_cells(i2c, bq769x0, |cell, low_cell| {
        //         (cell.0 - low_cell.0) < config::BALANCE_STOP_DELTA_MV
        //     })?;
        //     // writeln!(rtt, "CheckStop {:05b} {:05b}", stop_balancing.0, stop_balancing.1).ok();
        //     state.phase1 &= !stop_balancing.0;
        //     state.phase2 &= !stop_balancing.1;
        //     if state.phase1 != 0 || state.phase2 != 0 {
        //         BalancingStage::Phase1
        //     } else {
        //         bq769x0.enable_balancing(i2c, 0)?;
        //         BalancingStage::CheckStart
        //     }
        // }
    };

    //let max_delta = cells.iter().map(|cell| *cell - low_cell).max().unwrap();

    // let mut i = 1;
    // for needs_balancing in unbalanced_cells {
    //     writeln!(rtt, "{}: {}", i, needs_balancing);
    //     i += 1;
    // }
    // writeln!(rtt, "max_delta: {}", max_delta).ok();
    Ok(())
}