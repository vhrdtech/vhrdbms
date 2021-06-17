use crate::config;
use stm32l0xx_hal::adc::Error;
use core::fmt::Write;

pub fn adc_dma1_channel1(cx: crate::adc_dma1_channel1::Context) {
    let adc: &mut config::Adc = cx.resources.adc;
    let rtt = cx.resources.rtt;

    rtt.write_bytes(&[0xff, '2' as u8]);
    match adc.read_available() {
        Ok(values) => {
            for v in values {
                if let Ok(sample) = v {
                    writeln!(rtt, "{}", sample).ok();
                }
            }
        }
        Err(_) => {}
    }
    rtt.write_bytes(&[0xff, '0' as u8]);
}