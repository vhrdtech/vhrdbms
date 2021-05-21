use cfg_if::cfg_if;

#[allow(unused_variables)]
#[allow(unused_mut)]
pub fn idle(mut cx: crate::idle::Context) -> ! {
    cfg_if! {
        if #[cfg(feature = "cli")] {
            use rtic::Mutex;

            let mut bq769x0 = cx.resources.bq76920;
            let mut i2c = cx.resources.i2c;
            // let mut power_blocks = cx.resources.power_blocks;
            let mut afe_io = cx.resources.afe_io;
            let mut mcp25625 = cx.resources.mcp25625;
            let spawn = cx.spawn;
        }
    }

    loop {
        cfg_if! {
            if #[cfg(feature = "cli")] {
                cx.resources.rtt.lock(|rtt| {
                    i2c.lock(|i2c| {
                        bq769x0.lock(|bq769x0| {
                            // power_blocks.lock(|power_blocks| {
                                afe_io.lock(|afe_io| {
                                    mcp25625.lock(|mcp25625| {
                                        super::cli::cli(rtt, i2c, bq769x0, spawn, afe_io, mcp25625);
                                    })
                                })
                            // })
                        });
                    });

                });
            }
        }
        cortex_m::asm::delay(100_000);
    }
}


