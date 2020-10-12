use embedded_hal::digital::v2::{OutputPin, InputPin};

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
    fn status_is_ok(&self) -> bool;
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

    fn status_is_ok(&self) -> bool {
        if !self.enabled {
            true
        } else {
            match &self.stat_pin {
                Some(stat_pin) => {
                    stat_pin.is_high().unwrap_or(false)
                },
                None => { true }
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