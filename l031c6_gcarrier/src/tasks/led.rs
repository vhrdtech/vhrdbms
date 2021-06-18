use stm32l0xx_hal::prelude::{ToggleableOutputPin, OutputPin};
use mcu_helper::tim_cyccnt::U32Ext;
use stm32l0xx_hal::gpio::gpiob::{PB12, PB13, PB14, PB11, PB15};
use stm32l0xx_hal::gpio::{PushPull, Output};
use stm32l0xx_hal::gpio::gpioa::PA8;
use stm32l0xx_hal::pac::Interrupt;
// use core::fmt::Write;

#[derive(Debug)]
pub enum Event {
    Toggle,
    OnMode,
    OffMode
}

pub struct ChargeIndicator {
    pub led1: PB12<Output<PushPull>>,
    pub led2: PB13<Output<PushPull>>,
    pub led3: PB14<Output<PushPull>>,
    pub led4: PB11<Output<PushPull>>,
    pub led5: PA8<Output<PushPull>>,
    pub led6: PB15<Output<PushPull>>,
}

impl ChargeIndicator {
    pub fn off(&mut self) {
        self.led1.set_low().ok();
        self.led2.set_low().ok();
        self.led3.set_low().ok();
        self.led4.set_low().ok();
        self.led5.set_low().ok();
        self.led6.set_low().ok();
    }

    pub fn on_one(&mut self, idx: u8) {
        self.off();
        match idx {
            0 => { self.led1.set_high().ok(); },
            1 => { self.led2.set_high().ok(); },
            2 => { self.led3.set_high().ok(); },
            3 => { self.led4.set_high().ok(); },
            4 => { self.led5.set_high().ok(); },
            5 => { self.led6.set_high().ok(); },
            _ => {}
        }
    }
}

pub fn blinker(cx: crate::blinker::Context, e: Event, on: &mut bool, counter: &mut u8) {
    match e {
        Event::Toggle => {
            cx.resources.status_led.toggle().ok();
            let schedule_at = cx.scheduled + if *on {
                cx.resources.charge_indicator.on_one(*counter);
                *counter += 1;
                if *counter == 6 {
                    *counter = 0;
                }
                ms2cycles!(cx.resources.clocks, 150)
            } else {
                cx.resources.charge_indicator.off();
                ms2cycles!(cx.resources.clocks, 2000)
            };
            cx.schedule.blinker(schedule_at, Event::Toggle).ok();
            rtic::pend(Interrupt::DMA1_CHANNEL1);
            // writeln!(cx.resources.rtt, ".").ok();
        }
        Event::OnMode => {
            *on = true;
        }
        Event::OffMode => {
            *on = false;
        }
    }
}