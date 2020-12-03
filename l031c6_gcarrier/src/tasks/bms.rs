use crate::{config, tasks};
use bq769x0::BQ769x0;
use core::fmt::Write;
use mcu_helper::tim_cyccnt::U32Ext;
use tca9535::tca9534::{Tca9534, Port};
use crate::hal::prelude::_embedded_hal_adc_OneShot;

pub enum BmsEvent {
    TogglePower,
    CheckCharger,
}

#[derive(Default)]
pub struct BmsState {
    power_enabled: bool,

    charge_enabled: bool,
    vchg_prev_mv: u32,
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
    pub vchg_div_pin: config::VchgDivPin
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
                crate::board_components::imx_prepare_boot(i2c, tca9534);
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
                tca9534.modify_outputs(i2c, config::TCA_VHCG_DIV_EN_PIN, config::TCA_VHCG_DIV_EN_PIN).ok();
                cortex_m::asm::delay(100_000);
                let vchg_raw = adc.read(&mut afe_io.vchg_div_pin) as Result<u32, _>;
                let vchg_raw = vchg_raw.unwrap_or(0);
                tca9534.modify_outputs(i2c, config::TCA_VHCG_DIV_EN_PIN, Port::empty()).ok();
                let vchg = (vchg_raw * 8 * 3300) / 1 / 4095;
                //writeln!(rtt, "vchg: {}mV", vchg).ok();
                if bms_state.vchg_prev_mv != 0 {
                    let dvchg_mv = vchg as i32 - bms_state.vchg_prev_mv as i32;
                    if dvchg_mv > 100 {
                        let r = bq769x0.charge(i2c, true);
                        writeln!(rtt, "Charger plugged, en:{:?}", r).ok();
                        bms_state.charge_enabled = true;
                    }
                }
                bms_state.vchg_prev_mv = vchg;
            }
            cx.schedule.bms_event(cx.scheduled + ms2cycles!(clocks, config::CHARGER_CHECK_INTERVAL_MS), BmsEvent::CheckCharger).ok();
        }
    }
}