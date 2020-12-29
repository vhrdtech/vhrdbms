use crate::config;
use tca9535::tca9534::Tca9534;
use mcu_helper::color;

pub fn imx_prepare_boot(
    i2c: &mut config::InternalI2c,
    tca9534: &mut Tca9534<config::InternalI2c>,
    fmt: &mut dyn core::fmt::Write
) {
    let r = tca9534.modify_outputs(i2c, config::TCA_MXM_BOOT_SRC_PIN, config::TCA_MXM_BOOT_SRC_PIN); // 1 = uSD
    if r.is_err() {
        writeln!(fmt, "{}Eipb{}", color::RED, color::DEFAULT).ok();
    }
}