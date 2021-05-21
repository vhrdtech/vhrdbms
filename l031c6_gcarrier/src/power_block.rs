use hash32::{Hash, Hasher};
use core::convert::TryFrom;
use crate::config;

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum PowerBlockId {
    //DcDc5V0Bms,
    Switch5V0Syscan,
    Switch3V3S0,
    Switch5V0Uwb,
}

impl Hash for PowerBlockId {
    fn hash<H>(&self, state: &mut H) where
        H: Hasher {
        (*self as u8).hash(state)
    }
}

impl TryFrom<&str> for PowerBlockId {
    type Error = ();

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        use PowerBlockId::*;
        match value {
            "s5v0_syscan" => Ok(Switch5V0Syscan),
            "s3v3_s0" => Ok(Switch3V3S0),
            "s5v0_uwb" => Ok(Switch5V0Uwb),
            _ => Err(())
        }
    }
}

impl Into<&'static str> for PowerBlockId {
    fn into(self) -> &'static str {
        use PowerBlockId::*;
        match self {
            Switch5V0Syscan => "s5v0_syscan",
            Switch5V0Uwb => "s5v0_uwb",
            Switch3V3S0 => "s3v3_s0",
        }
    }
}

pub fn enable_all(_power_blocks: &mut config::PowerBlocksMap,) {
    // power_blocks.get_mut(&PowerBlockId::Switch5V0Syscan).expect("I1").enable();
}

pub fn disable_all(_power_blocks: &mut config::PowerBlocksMap) {
}