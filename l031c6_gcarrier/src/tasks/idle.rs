use cfg_if::cfg_if;

#[allow(unused_variables)]
#[allow(unused_mut)]
pub fn idle(mut cx: crate::idle::Context) -> ! {
    cfg_if! {
        if #[cfg(feature = "cli")] {
            use rtic::Mutex;

            let bq769x0 = cx.resources.bq76920;
            let i2c = cx.resources.i2c;
            let power_blocks = cx.resources.power_blocks;
            let tca9534 = cx.resources.tca9534;
        }
    }

    loop {
        cfg_if! {
            if #[cfg(feature = "cli")] {
                cx.resources.rtt.lock(|rtt| super::cli::cli(rtt, i2c, bq769x0, power_blocks, tca9534));
            }
        }
        cortex_m::asm::delay(100_000);
    }
}


