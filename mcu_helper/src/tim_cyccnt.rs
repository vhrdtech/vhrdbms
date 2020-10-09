use core::ops;
use core::cmp::Ordering;
use core::convert::{TryInto, Infallible};
use rtic::Fraction;
use crate::hal as hal;

#[derive(Clone, Copy, Eq, PartialEq)]
pub struct TimInstant {
    inner: i32,
}

impl core::fmt::Debug for TimInstant {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_tuple("Instant")
            .field(&(self.inner as u32))
            .finish()
    }
}

impl TimInstant {
    pub fn now() -> Self {
        TimInstant {
            inner: 0
        }
    }

    /// Returns the amount of time elapsed since this instant was created.
    pub fn elapsed(&self) -> TimDuration {
        let diff = TimInstant::now().inner.wrapping_sub(self.inner);
        assert!(diff >= 0, "instant now is earlier than self");
        TimDuration { inner: diff as u32 }
    }

    /// Returns the amount of time elapsed from another instant to this one.
    pub fn duration_since(&self, earlier: TimInstant) -> TimDuration {
        let diff = self.inner.wrapping_sub(earlier.inner);
        assert!(diff >= 0, "second instant is later than self");
        TimDuration { inner: diff as u32 }
    }
}

impl ops::AddAssign<TimDuration> for TimInstant {
    fn add_assign(&mut self, dur: TimDuration) {
        // NOTE this is a debug assertion because there's no foolproof way to detect a wrap around;
        // the user may write `(instant + dur) + dur` where `dur` is `(1<<31)-1` ticks.
        debug_assert!(dur.inner < (1 << 31));
        self.inner = self.inner.wrapping_add(dur.inner as i32);
    }
}

impl ops::Add<TimDuration> for TimInstant {
    type Output = Self;

    fn add(mut self, dur: TimDuration) -> Self {
        self += dur;
        self
    }
}

impl ops::SubAssign<TimDuration> for TimInstant {
    fn sub_assign(&mut self, dur: TimDuration) {
        // NOTE see the NOTE in `<Instant as AddAssign<Duration>>::add_assign`
        debug_assert!(dur.inner < (1 << 31));
        self.inner = self.inner.wrapping_sub(dur.inner as i32);
    }
}

impl ops::Sub<TimDuration> for TimInstant {
    type Output = Self;

    fn sub(mut self, dur: TimDuration) -> Self {
        self -= dur;
        self
    }
}

impl ops::Sub<TimInstant> for TimInstant {
    type Output = TimDuration;

    fn sub(self, other: TimInstant) -> TimDuration {
        self.duration_since(other)
    }
}

impl Ord for TimInstant {
    fn cmp(&self, rhs: &Self) -> Ordering {
        self.inner.wrapping_sub(rhs.inner).cmp(&0)
    }
}

impl PartialOrd for TimInstant {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        Some(self.cmp(rhs))
    }
}


#[derive(Clone, Copy, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct TimDuration {
    inner: u32,
}

impl TimDuration {
    /// Creates a new `Duration` from the specified number of clock cycles
    pub fn from_cycles(cycles: u32) -> Self {
        TimDuration { inner: cycles }
    }

    /// Returns the total number of clock cycles contained by this `Duration`
    pub fn as_cycles(&self) -> u32 {
        self.inner
    }
}

impl TryInto<u32> for TimDuration {
    type Error = Infallible;

    fn try_into(self) -> Result<u32, Infallible> {
        Ok(self.as_cycles())
    }
}

impl ops::AddAssign for TimDuration {
    fn add_assign(&mut self, dur: TimDuration) {
        self.inner += dur.inner;
    }
}

impl ops::Add<TimDuration> for TimDuration {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        TimDuration {
            inner: self.inner + other.inner,
        }
    }
}

impl ops::SubAssign for TimDuration {
    fn sub_assign(&mut self, rhs: TimDuration) {
        self.inner -= rhs.inner;
    }
}

impl ops::Sub<TimDuration> for TimDuration {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        TimDuration {
            inner: self.inner - rhs.inner,
        }
    }
}

pub trait U32Ext {
    /// Converts the `u32` value into clock cycles
    fn cycles(self) -> TimDuration;
}

impl U32Ext for u32 {
    fn cycles(self) -> TimDuration {
        TimDuration { inner: self }
    }
}

pub struct TimCyccnt;
impl rtic::Monotonic for TimCyccnt {
    type Instant = TimInstant;

    fn ratio() -> Fraction {
        Fraction {
            numerator: 1,
            denominator: 1,
        }
    }

    fn now() -> Self::Instant {
        let device = unsafe { hal::pac::Peripherals::steal() };
        let lsb_tim = device.TIM22;
        let msb_tim = device.TIM21;
        let ticks = msb_tim.cnt.read().bits() << 16 | lsb_tim.cnt.read().bits();
        TimInstant { inner: ticks as i32 }
    }

    unsafe fn reset() {
        let device = hal::pac::Peripherals::steal();
        let rcc = device.RCC;
        // Enable and reset TIM21 and TIM22
        rcc.apb2enr.modify(|_, w| w.tim21en().set_bit().tim22en().set_bit());
        rcc.apb2rstr.modify(|_, w| w.tim21rst().set_bit().tim22rst().set_bit());
        rcc.apb2rstr.modify(|_, w| w.tim21rst().clear_bit().tim22rst().clear_bit());

        let lsb_tim = device.TIM22; // Master timer, drives msb_tim clock
        lsb_tim.cr1.modify(|_, w| w.cen().clear_bit());
        lsb_tim.cr2.modify(|_, w| w.mms().update()); // update event as trigger ouput (TRGO)
        lsb_tim.cnt.reset();
        lsb_tim.psc.write(|w| w.psc().bits(0)); // Run at the same freq as SysClk
        lsb_tim.cr1.modify(|_, w| w.urs().set_bit());
        lsb_tim.egr.write(|w| w.ug().set_bit());

        let msb_tim = device.TIM21;
        msb_tim.cr1.modify(|_, w| w.cen().clear_bit());
        msb_tim.smcr.modify(|_, w| w.ts().itr1().sms().ext_clock_mode()); // clock from lsb_tim

        lsb_tim.cr1.modify(|_, w| w.cen().set_bit());
        msb_tim.cr1.modify(|_, w| w.cen().set_bit());
    }

    fn zero() -> Self::Instant {
        TimInstant { inner: 0 }
    }
}
