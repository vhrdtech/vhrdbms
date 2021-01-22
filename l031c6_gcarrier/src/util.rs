#[allow(unused_macros)]
macro_rules! ms2cycles {
    ($clocks:expr, $amount:expr) => {
        ($clocks.sys_clk().0 / 1_000 * $amount).cycles()
    };
}

macro_rules! ms2cycles_raw {
    ($clocks:expr, $amount:expr) => {
        ($clocks.sys_clk().0 / 1_000 * $amount)
    };
}

macro_rules! us2cycles_raw {
    ($clocks:expr, $amount:expr) => {
        (($clocks.sys_clk().0 as u64) * ($amount as u64) / 1_000_000)
    };
}

macro_rules! delay_ms {
   ($clocks:expr, $amount:expr) => {
       cortex_m::asm::delay(ms2cycles_raw!($clocks, $amount));
   };
}