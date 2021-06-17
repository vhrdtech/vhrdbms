pub mod init;
pub mod idle;
#[cfg(feature = "cli")]
mod cli;
pub mod button;
pub mod canbus;
pub mod bms;
pub mod api;
pub mod softoff;
pub mod led;
pub mod beeper;
pub mod adc;