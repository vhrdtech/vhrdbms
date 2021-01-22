#[allow(unused_macros)]
macro_rules! ms2cycles {
    ($clocks:expr, $amount:expr) => {
        ($clocks.sys_clk().0 / 1_000 * $amount).cycles()
    };
}

macro_rules! us2cycles_raw {
    ($clocks:expr, $amount:expr) => {
        (($clocks.sys_clk().0 as u64) * ($amount as u64) / 1_000_000)
    };
}