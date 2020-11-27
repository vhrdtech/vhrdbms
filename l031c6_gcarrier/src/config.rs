use crate::hal;

use hal::gpio::{
    gpioa::*,
    // gpiob::*,
};
use hal::gpio::{Analog, PushPull, Output, OpenDrain};

// Mcp25625
pub type Mcp25625Sck = PA5<Analog>;
pub type Mcp25625Miso = PA6<Analog>;
pub type Mcp25625Mosi = PA7<Analog>;
pub type Mcp25625Cs = PA4<Output<PushPull>>;
pub type Mcp25625Instance = mcp25625::MCP25625<hal::spi::Spi<hal::pac::SPI1, (Mcp25625Sck, Mcp25625Miso, Mcp25625Mosi)>, Mcp25625Cs>;

// DCDCs and Switches
// Make sure size is enough, since it is not easily possible to check that unfortunately
use alloc::boxed::Box;
use power_helper::power_block::PowerBlockControl;
use crate::power_block::PowerBlockId;
pub type PowerBlocksMap =  heapless::FnvIndexMap::<PowerBlockId, Box<dyn PowerBlockControl>, heapless::consts::U16>;

// Internal i2c
#[cfg(not(feature = "bitbang-i2c"))]
pub type InternalI2c = hal::i2c::I2c<hal::pac::I2C1, PA10<Output<OpenDrain>>, PA9<Output<OpenDrain>>>;
#[cfg(feature = "bitbang-i2c")]
pub type InternalI2c = bitbang_hal::i2c::I2cBB<PA9<Output<OpenDrain>>, PA10<Output<OpenDrain>>, hal::timer::Timer<hal::pac::TIM2>>;

// TCA9534 I2C GPIO expander pins
use tca9535::tca9534::Port;
pub const TCA_TMS_TDO_PIN: Port = Port::P00;
pub const TCA_MXM_ON_OFF_PIN: Port = Port::P01;
pub const TCA_MXM_BOOT_SRC_PIN: Port = Port::P02;
pub const TCA_VHCG_DIV_EN_PIN: Port = Port::P03; // Do not forgot to solder on rev.B boards
