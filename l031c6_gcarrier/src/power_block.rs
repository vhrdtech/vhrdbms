use hash32::{Hash, Hasher};
use core::convert::TryFrom;

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum PowerBlockId {
    DcDc5V0Bms,
    DcDc5V0Hc,
    DcDc3V3Hc,
    DcDc3V8Imx,
    Switch5V0Syscan,
    Switch5V0M12,
    Switch5V0Flex,
    Switch5V0Usb,
    Switch3V3UWBLc,
    Switch3V3UWBHc,
    Switch3V3Tms,
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
            "d5v0_bms" => Ok(DcDc5V0Bms),
            "d5v0_hc" => Ok(DcDc5V0Hc),
            "d3v3_hc" => Ok(DcDc3V3Hc),
            "d3v8_imx" => Ok(DcDc3V8Imx),
            "s5v0_syscan" => Ok(Switch5V0Syscan),
            "s5v0_m12" => Ok(Switch5V0M12),
            "s5v0_flex" => Ok(Switch5V0Flex),
            "s5v0_usb" => Ok(Switch5V0Usb),
            "s3v3_uwblc" => Ok(Switch3V3UWBLc),
            "s3v3_uwbhc" => Ok(Switch3V3UWBHc),
            "s3v3_tms" => Ok(Switch3V3Tms),
            _ => Err(())
        }
    }
}

impl Into<&'static str> for PowerBlockId {
    fn into(self) -> &'static str {
        use PowerBlockId::*;
        match self {
            DcDc5V0Bms => "d5v0_bms",
            DcDc5V0Hc => "d5v0_hc",
            DcDc3V3Hc => "d3v3_hc",
            DcDc3V8Imx => "d3v8_imx",
            Switch5V0Syscan => "s5v0_syscan",
            Switch5V0M12 => "s5v0_m12",
            Switch5V0Flex => "s5v0_flex",
            Switch5V0Usb => "s5v0_usb",
            Switch3V3UWBLc => "s3v3_uwblc",
            Switch3V3UWBHc => "s3v3_uwbhc",
            Switch3V3Tms => "s3v3_tms",
        }
    }
}