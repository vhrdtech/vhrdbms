use core::fmt::Formatter;
#[allow(unused_macros)]
macro_rules! ms2cycles {
    ($clocks:expr, $amount:expr) => {
        ($clocks.sys_clk().0 / 1_000 * $amount).cycles()
    };
}

#[allow(unused_macros)]
macro_rules! ms2cycles_raw {
    ($clocks:expr, $amount:expr) => {
        ($clocks.sys_clk().0 / 1_000 * $amount)
    };
}

#[allow(unused_macros)]
macro_rules! us2cycles_raw {
    ($clocks:expr, $amount:expr) => {
        (($clocks.sys_clk().0 as u64) * ($amount as u64) / 1_000_000)
    };
}

#[allow(unused_macros)]
macro_rules! delay_ms {
   ($clocks:expr, $amount:expr) => {
       cortex_m::asm::delay(ms2cycles_raw!($clocks, $amount));
   };
}

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum EventSource {
    LocalNoForward,
    LocalForward,
    RemoteNoForward,
    RemoteForward,
}

impl EventSource {
    pub fn need_to_forward(&self) -> bool {
        *self == EventSource::LocalForward || *self == EventSource::RemoteForward
    }
}

extern "C" {
    pub static mut __sheap: usize;
    pub static _stack_pointer_upper: usize;
}

pub const STACK_PROBE_MAGICWORD: u32 = 0x5ca1ab1e;
pub fn stack_lower_bound() -> usize {
    unsafe {
        let used_ram_end = &__sheap as *const usize;
        used_ram_end as usize
    }
}

pub fn stack_upper_bound() -> usize {
    unsafe {
        let upper_bound = &_stack_pointer_upper as *const usize;
        upper_bound as usize
    }
}

pub fn current_stack_pointer() -> usize {
    // Grab a copy of the stack pointer
    let x: usize;
    unsafe {
        asm!("mov {}, sp", out(reg) x);
    }
    x
}

pub struct Ohms(pub u32);

pub struct MilliVolts(pub i32);
impl core::fmt::Display for MilliVolts {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}mV", self.0)
    }
}

pub fn resistor_divider_inverse(rt: Ohms, rb: Ohms, vin: MilliVolts) -> MilliVolts {
    MilliVolts(vin.0 * (rt.0 as i32 + rb.0 as i32) / rb.0 as i32)
}