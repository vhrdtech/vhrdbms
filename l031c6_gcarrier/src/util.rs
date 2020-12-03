#[allow(unused_macros)]
macro_rules! ms2cycles {
    ($clocks:expr, $amount:expr) => {
        ($clocks.sys_clk().0 / 1_000 * $amount).cycles()
    };
}