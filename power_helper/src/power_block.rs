use embedded_hal::digital::v2::{OutputPin, InputPin};

pub enum PowerBlockType {
    DcDc,
    Switch
}

pub struct PowerBlock<EnPin, StatPin> {
    pub r#type: PowerBlockType,
    pub en_pin: EnPin,
    pub stat_pin: Option<StatPin>,
    pub name: &'static str
}

pub trait PowerBlockControl {
    fn enable(&mut self);
 //   fn disable(&mut self);
  //  fn name(&self) -> &'static str;
   // fn status_is_ok(&self) -> bool;
}

impl<EnPin: OutputPin, StatPin: InputPin> PowerBlockControl for PowerBlock<EnPin, StatPin> {
    fn enable(&mut self) {
        let _ = self.en_pin.set_high();
    }
}

pub struct DummyInputPin;
impl InputPin for DummyInputPin {
    type Error = ();
    fn is_high(&self) -> Result<bool, Self::Error> { unreachable!() }
    fn is_low(&self) -> Result<bool, Self::Error> { unreachable!() }
}

impl<EnPin: OutputPin, StatPin: InputPin> PowerBlock<EnPin, StatPin> {
    pub fn new(r#type: PowerBlockType, en_pin: EnPin, stat_pin: Option<StatPin>, name: &'static str) -> Self {
        Self { r#type, en_pin, stat_pin, name }
    }
}