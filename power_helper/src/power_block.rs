use embedded_hal::digital::v2::{OutputPin, InputPin};
use core::fmt;
use mcu_helper::color;

pub enum PowerBlockType {
    DcDc,
    Switch
}

pub struct PowerBlock<EnPin, StatPin> {
    pub r#type: PowerBlockType,
    pub en_pin: EnPin,
    pub stat_pin: Option<StatPin>,
    enabled: bool
}

pub trait PowerBlockControl {
    fn enable(&mut self);
    fn disable(&mut self);
    fn is_enabled(&self) -> bool;
    fn status_is_ok(&self) -> Option<bool>;
}

impl<EnPin: OutputPin, StatPin: InputPin> PowerBlockControl for PowerBlock<EnPin, StatPin> {
    fn enable(&mut self) {
        let _ = self.en_pin.set_high();
        self.enabled = true;
    }

    fn disable(&mut self) {
        let _ = self.en_pin.set_low().ok();
        self.enabled = false;
    }

    fn is_enabled(&self) -> bool {
        self.enabled
    }

    fn status_is_ok(&self) -> Option<bool> {
        if self.stat_pin.is_none() {
            None
        } else {
            if !self.enabled {
                Some(true)
            } else {
                Some(self.stat_pin.as_ref().unwrap().is_high().unwrap_or(false))
            }
        }
    }
}

pub struct DummyInputPin;
impl InputPin for DummyInputPin {
    type Error = ();
    fn is_high(&self) -> Result<bool, Self::Error> { unreachable!() }
    fn is_low(&self) -> Result<bool, Self::Error> { unreachable!() }
}

impl<EnPin: OutputPin, StatPin: InputPin> PowerBlock<EnPin, StatPin> {
    pub fn new(r#type: PowerBlockType, mut en_pin: EnPin, stat_pin: Option<StatPin>) -> Self {
        en_pin.set_low().ok();
        Self { r#type, en_pin, stat_pin, enabled: false }
    }
}

impl fmt::Debug for dyn PowerBlockControl {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_enabled() {
            let _ = write!(f, "{}Enabled", color::GREEN);
            match self.status_is_ok() {
                Some(status) => {
                    if status {
                        let _ = write!(f, ", Good");
                    } else {
                        let _ = write!(f, "{}, Bad", color::RED);
                    }
                },
                None => {}
            }
            write!(f, "{}", color::DEFAULT)
        } else {
            write!(f, "{}Disabled{}", color::YELLOW, color::DEFAULT)
        }
    }
}