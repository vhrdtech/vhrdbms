use crate::{hal, util};

use hal::gpio::{
    gpioa::*,
    // gpiob::*,
    // gpioc::*,
};
use hal::gpio::{Analog, PushPull, Output, OpenDrain};

//
pub const CHARGER_CHECK_INTERVAL_MS: u32 = 2000;
pub const BALANCING_CHECK_INTERVAL_MS: u32 = 5000;

// Mcp25625
pub type Mcp25625Sck = PB3<Analog>;
pub type Mcp25625Miso = PA11<Analog>;
pub type Mcp25625Mosi = PA12<Analog>;
pub type Mcp25625IrqUninit = PA10<Analog>;
pub type Mcp25625Cs = PA9<Output<PushPull>>;
pub type Mcp25625Spi = hal::pac::SPI1;
pub type Mcp25625Instance = mcp25625::MCP25625<hal::spi::Spi<Mcp25625Spi, (Mcp25625Sck, Mcp25625Miso, Mcp25625Mosi)>, Mcp25625Cs>;
pub type Mcp25625Irq = PA10<Input<PullUp>>;
pub type CanTX = vhrdcan::FrameHeap<vhrdcan::heapless::consts::U16>;
pub type CanRX = vhrdcan::FrameHeap<vhrdcan::heapless::consts::U8>;
pub const CAN_TX_HANDLER: Interrupt = Interrupt::EXTI4_15;

// DCDCs and Switches
// Make sure size is enough, since it is not easily possible to check that unfortunately
// use alloc::boxed::Box;
// use power_helper::power_block::PowerBlockControl;
// use crate::power_block::PowerBlockId;
// pub type PowerBlocksMap =  heapless::FnvIndexMap::<PowerBlockId, Box<dyn PowerBlockControl + Send>, heapless::consts::U16>;

// Internal i2c
#[cfg(not(feature = "bitbang-i2c"))]
pub type InternalI2c = hal::i2c::I2c<hal::pac::I2C1, PA10<Output<OpenDrain>>, PA9<Output<OpenDrain>>>;
#[cfg(feature = "bitbang-i2c")]
pub type InternalI2c = bitbang_hal::i2c::I2cBB<PB6<Output<OpenDrain>>, PB7<Output<OpenDrain>>, hal::timer::Timer<hal::pac::TIM6>>;

use stm32l0xx_hal::gpio::{Input, Floating, PullUp};

// Power button
pub type ButtonPin = PA15<Input<Floating>>;
pub const BUTTON_SHORT_PRESS_MS: u32 = 200;
pub const BUTTON_LONG_PRESS_MS: u32 = 2000;
pub const BUTTON_IRQ: Interrupt = Interrupt::EXTI4_15; // keep in sync with button task!

// Afe
const BQ769X0_VARIANT: usize = bq769x0::BQ76920;
pub type BQ769x0 = bq769x0::BQ769x0<{ BQ769X0_VARIANT }>;
pub const CELL_COUNT: CellCount = CellCount::_4S;
/// Halt after one of the cells drops below this threshold (HW)
pub const CELL_UV_THRESHOLD: MilliVolts = MilliVolts(2950);
/// Power off after one of the cells drops below this threshold (SW)
pub const CELL_SOFT_UV_THRESHOLD: MilliVolts = MilliVolts(3150);
/// Stop charging and disable zero voltage charge fet (if present) at this threshold (SW)
pub const CELL_SOFT_OV_THRESHOLD: MilliVolts = MilliVolts(4210);
/// HW overvoltage limit
pub const CELL_OV_THRESHOLD: MilliVolts = MilliVolts(4250);
/// Clear OV flag when highest cell drop below this threshold
pub const CELL_OV_CLEAR: MilliVolts = MilliVolts(4100);
pub const AFE_FAULT_COUNT_TO_HALT: u8 = 20;
/// Stop precharge and stort normal charge at this cell voltage.
/// Lowest cell in the stack is used.
pub const PRECHARGE_THRESHOLD: MilliVolts = MilliVolts(3300);
pub const PRECHARGE_HYSTERESIS: MilliVolts = MilliVolts(50);
/// Charger detection threshold (difference between pack and bat voltages)
pub const CHARGER_DETECTION_THRESHOLD: util::MilliVolts = util::MilliVolts(50);
/// Power path configuration
pub const IS_SEPARATE_CHG_PATH: bool = false;
/// Stop charging and disable CHG & DSG mosfets when current drop below this.
pub const CHARGE_STOP_CURRENT: MilliAmperes = MilliAmperes(100);
/// If min cell is above this threshold and charger is present, do not start charging to prevent endless turn-on-off cycle.
pub const CHARGE_START_MIN_VOLTAGE: MilliVolts = MilliVolts(4000);

// Voltage dividers
pub const BAT_DIV_RT: Ohms = Ohms(102_000);
pub const BAT_DIV_RB: Ohms = Ohms(16_200);

pub const PACK_DIV_RT: Ohms = Ohms(270_000);
pub const PACK_DIV_RB: Ohms = Ohms(43_000);

// Afe IO
pub type AfeWakePin = PB4<Output<PushPull>>;
pub type PackDivEnPin = PB9<Output<PushPull>>;
pub type PackDivPin = PA6<Analog>;
pub type BatDivEnPin = PA2<Output<PushPull>>;
pub type BatDivPin = PA7<Analog>;
pub type DcDcEnPin = PA3<Output<PushPull>>;
pub type AfeChgOverridePin = PH1<Output<PushPull>>;
pub type AfeDsgOverridePin = PH0<Output<PushPull>>;
pub type ZvchgDisablePin = PB8<Output<PushPull>>;
pub type Switch5V0S0Pin = PA5<Output<PushPull>>;
pub type Switch3V3S0Pin = PA4<Output<PushPull>>;
pub type Switch5V0AuxPin = PB0<Output<PushPull>>;

// Balancing
pub const BALANCE_START_DELTA_MV: u32 = 300;
// pub const BALANCE_STOP_DELTA_MV: u32 = 5;

// CanBus Protocol
use stm32l0xx_hal::pac::Interrupt;
use vhrdcan::FrameId;
use bq769x0::{MilliVolts, MilliAmperes};
use stm32l0xx_hal::gpio::gpiob::{PB9, PB4, PB6, PB7, PB3, PB8, PB0};
use stm32l0xx_hal::gpio::gpioh::{PH0, PH1};
use crate::tasks::bms::CellCount;
use crate::util::Ohms;

pub const HEARTBEAT_INTERVAL_MS: u32 = 1000;

// Soft off
#[cfg(feature = "softoff")]
pub const SOFTOFF_TIMEOUT_MS: u32 = 20_000;
#[cfg(not(feature = "softoff"))]
pub const SOFTOFF_TIMEOUT_MS: u32 = 0;
pub const SOFTOFF_NOTIFY_INTERVAL_MS: u32 = 1_000;
pub const SOFTOFF_NOTIFY_FRAME_ID: FrameId = FrameId::new_extended(0x15E).unwrap();
pub const POWER_CONTROL_FRAME_ID: FrameId = FrameId::new_standard(0x7).unwrap();

pub const POWER_ON_BURST_INTERVAL_MS: u32 = 25;
pub const POWER_ON_BURST_DURATION_MS: u32 = 10_000;

pub const CANCTRL_OFF_DURATION_MS: u32 = 4800;
pub const CANCTRL_ON_DURATION_MS: u32 = 200;

pub const UAVCAN_NODE_ID: u8 = 50;