use core::fmt::Write;
use mcp25625::{McpReceiveBuffer, McpPriority, FiltersConfigBuffer0, FiltersMask, FiltersConfigBuffer1, FiltersConfig, MCP25625Config, McpOperationMode, MCP25625, McpErrorKind};
use crate::config;
use cfg_if::cfg_if;
use mcu_helper::color;

use config::{Mcp25625Cs, Mcp25625Sck, Mcp25625Miso, Mcp25625Mosi, Mcp25625Spi, Mcp25625Instance, Mcp25625IrqUninit};
use stm32l0xx_hal::time::U32Ext;
use stm32l0xx_hal::spi::SpiExt;
use stm32l0xx_hal::prelude::OutputPin;
use crate::config::Mcp25625Irq;
use stm32l0xx_hal::exti::{GpioLine, Exti, ExtiLine};
use vhrdcan::FrameId;

pub struct Mcp25625Parts<SPI> {
    pub cs: Mcp25625Cs,
    pub sck: Mcp25625Sck,
    pub miso: Mcp25625Miso,
    pub mosi: Mcp25625Mosi,
    pub spi: SPI,
    pub irq: Mcp25625IrqUninit,
}

pub enum Mcp25625State {
    Operational(Option<(Mcp25625Instance, Mcp25625Irq)>),
    PoweredDown(Option<Mcp25625Parts<Mcp25625Spi>>)
}

macro_rules! write_if_cps {
    ($rtt: expr, $($arg:tt)*) => {
        cfg_if! {
            if #[cfg(feature = "canbus-printstat")] {
                writeln!($rtt, $($arg)*).ok();
            }
        }
    };
}

#[derive(Debug)]
pub enum Error {
    CodeFault,
    McpError(McpErrorKind)
}

impl From<McpErrorKind> for Error {
    fn from(e: McpErrorKind) -> Self {
        Error::McpError(e)
    }
}

fn mcp25625_bringup(mcp25625_state: &mut Mcp25625State, rcc: &mut crate::hal::rcc::Rcc) -> Result<(), Error> {
    let mut mcp_parts = match mcp25625_state {
        Mcp25625State::Operational(_) => {
            return Ok(());
        }
        Mcp25625State::PoweredDown(p) => {
            match p.take() {
                Some(parts) => parts,
                None => { return Err(Error::CodeFault); }
            }
        }
    };
    // SPI init
    let mcp_freq = 1.mhz();
    let spi = mcp_parts.spi.spi(
        (mcp_parts.sck, mcp_parts.miso, mcp_parts.mosi),
        crate::hal::spi::MODE_0,
        mcp_freq,
        rcc
    );
    // MCP25625 init
    mcp_parts.cs.set_high().ok();
    let irq = mcp_parts.irq.into_pull_up_input();
    cortex_m::asm::delay(100_000);
    let mut mcp25625 = MCP25625::new(spi, mcp_parts.cs, mcp_freq.0 * 1_000_000, rcc.clocks.sys_clk().0);
    match mcp25625_configure(&mut mcp25625) {
        Ok(()) => {

        },
        Err(e) => {
            *mcp25625_state = Mcp25625State::PoweredDown(Some(mcp25625_free(mcp25625, irq, rcc)));
            return Err(e.into());
        }
    }

    *mcp25625_state = Mcp25625State::Operational(Some((mcp25625, irq)));
    Ok(())
}

fn mcp25625_free(mcp25625: Mcp25625Instance, irq: Mcp25625Irq, rcc: &mut crate::hal::rcc::Rcc) -> Mcp25625Parts<Mcp25625Spi> {
    let (spi, mut cs) = mcp25625.release();
    let (spi, (sck, miso, mosi)) = spi.free(rcc);
    cs.set_low().ok();
    let mut sck = sck.into_push_pull_output();
    sck.set_low().ok();
    let mut mosi = mosi.into_push_pull_output();
    mosi.set_low().ok();

    Mcp25625Parts {
        cs,
        sck: sck.into_analog(),
        miso,
        mosi: mosi.into_analog(),
        spi,
        irq: irq.into_analog()
    }
}

fn mcp25625_bringdown(mcp25625_state: &mut Mcp25625State, rcc: &mut crate::hal::rcc::Rcc) -> Result<(), Error> {
    let (mcp25625, irq) = match mcp25625_state {
        Mcp25625State::Operational(mcp25625) => {
            match mcp25625.take() {
                Some((mcp25625, irq)) => (mcp25625, irq),
                None => { return Err(Error::CodeFault); }
            }
        }
        Mcp25625State::PoweredDown(_) => {
            return Err(Error::CodeFault);
        }
    };
    *mcp25625_state = Mcp25625State::PoweredDown(Some(mcp25625_free(mcp25625, irq, rcc)));

    Ok(())
}

fn mcp25625_configure(mcp25625: &mut config::Mcp25625Instance) -> Result<(), McpErrorKind> {
    let filters_buffer0 = FiltersConfigBuffer0 {
        mask: FiltersMask::AllExtendedIdBits,
        filter0: config::SOFTOFF_NOTIFY_FRAME_ID,
        filter1: None
    };
    let filters_buffer1 = FiltersConfigBuffer1 {
        mask: FiltersMask::OnlyStandardIdBits,
        filter2: config::POWER_CONTROL_FRAME_ID,
        filter3: None,
        filter4: None,
        filter5: None,
    };
    let filters_config = FiltersConfig::Filter(filters_buffer0, Some(filters_buffer1));
    let mcp_config = MCP25625Config {
        brp: 0, // Fosc=16MHz
        prop_seg: 3,
        ph_seg1: 2,
        ph_seg2: 2,
        sync_jump_width: 2,
        rollover_to_buffer1: true,
        filters_config,
        // filters_config: FiltersConfig::ReceiveAll,
        operation_mode: McpOperationMode::Normal
    };
    mcp25625.apply_config(mcp_config)?;
    mcp25625.enable_interrupts(0b0001_1111);
    Ok(())
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Event {
    /// Turn off completely
    BringDown,
    /// Turn on for indefinite amount of time
    BringUp,
    /// Turn on and then turn off, if already on - ignore
    BringUpThenBringDown,
    /// Turn off and schedule periodic turn on
    BringDownWithPeriodicBringUp,
    _BringDownNoStateChange,
}

#[derive(Copy, Clone, PartialEq)]
enum CtrlState {
    Down,
    Up,
    PeriodicUp
}

#[derive(Copy, Clone, PartialEq)]
pub struct State {
    ctrl_state: CtrlState
}
impl State {
    pub const fn new() -> Self {
        State {
            ctrl_state: CtrlState::Down
        }
    }
}

pub fn canctrl_event(cx: crate::canctrl_event::Context, e: Event, s: &mut State) {
    let mcp25625_state = cx.resources.mcp25625_state;
    let rcc = cx.resources.rcc;
    let can_tx = cx.resources.can_tx;
    let afe_io = cx.resources.afe_io;
    let rtt = cx.resources.rtt;
    let clocks = cx.resources.clocks;
    use mcu_helper::tim_cyccnt::U32Ext;

    writeln!(rtt, "{:?}", e).ok();
    match e {
        Event::BringDown | Event::_BringDownNoStateChange | Event::BringDownWithPeriodicBringUp => {
            match e {
                Event::BringDown => {
                    s.ctrl_state = CtrlState::Down;
                }
                Event::BringDownWithPeriodicBringUp => {
                    if s.ctrl_state != CtrlState::PeriodicUp {
                        cx.schedule.canctrl_event(cx.scheduled + ms2cycles!(clocks, config::CANCTRL_OFF_DURATION_MS), Event::BringUpThenBringDown).ok();
                        s.ctrl_state = CtrlState::PeriodicUp;
                    }
                }
                Event::_BringDownNoStateChange => {
                    if s.ctrl_state == CtrlState::PeriodicUp {
                        cx.schedule.canctrl_event(cx.scheduled + ms2cycles!(clocks, config::CANCTRL_OFF_DURATION_MS), Event::BringUpThenBringDown).ok();
                    } else {
                        return;
                    }
                }
                _ => unreachable!()
            }
            afe_io.disable_s0_switches();
            let r = mcp25625_bringdown(mcp25625_state, rcc);
            writeln!(rtt, "McpDown {}", r.is_ok()).ok();

        }
        Event::BringUp | Event::BringUpThenBringDown => {
            if e == Event::BringUpThenBringDown && s.ctrl_state == CtrlState::PeriodicUp {
                cx.schedule.canctrl_event(cx.scheduled + ms2cycles!(clocks, config::CANCTRL_ON_DURATION_MS), Event::_BringDownNoStateChange).ok();
            }
            if e == Event::BringUp {
                s.ctrl_state = CtrlState::Up;
            }
            if let Mcp25625State::Operational(_) = mcp25625_state {
                return;
            }
            if s.ctrl_state == CtrlState::Down {
                return;
            }
            afe_io.enable_s0_switches();
            can_tx.clear();
            match mcp25625_bringup(mcp25625_state, rcc) {
                Ok(()) => {
                    writeln!(rtt, "McpOk").ok();
                },
                Err(e) => {
                    writeln!(rtt, "McpErr: {:?}", e).ok();
                    afe_io.disable_s0_switches();
                }
            }
        }
    }
}

pub fn canctrl_irq(cx: &mut crate::exti4_15::Context) {
    let mcp25625_state: &mut Mcp25625State = cx.resources.mcp25625_state;
    let (mut mcp25625, irq) = match mcp25625_state {
        Mcp25625State::Operational(mcp25625) => {
            match mcp25625.take() {
                Some(mcp25625) => mcp25625,
                None => {
                    // write_if_cps!(rtt, "can down");
                    return;
                }
            }
        }
        Mcp25625State::PoweredDown(parts) => {
            let irq_line = GpioLine::from_raw_line(parts.as_ref().unwrap().irq.pin_number()).unwrap();
            Exti::unpend(irq_line);
            return;
        }
    };
    let irq_line = GpioLine::from_raw_line(irq.pin_number()).unwrap();
    Exti::unpend(irq_line);
    // if !Exti::is_pending(irq_line) {
    //     return;
    // }

    cfg_if! {
        if #[cfg(feature = "canbus-printstat")] {
            let rtt: &mut jlink_rtt::NonBlockingOutput = &mut cx.resources.rtt;
            rtt.write_bytes(&[0xff, '1' as u8]);
        }
    }

    let can_tx: &mut config::CanTX = cx.resources.can_tx;
    let can_rx: &mut config::CanRX = cx.resources.can_rx;

    let intf = mcp25625.interrupt_flags();
    write_if_cps!(rtt, "INTF: {:?}", intf);
    let errf = mcp25625.error_flags();
    write_if_cps!(rtt, "{:?}", errf);

    let mut buffers = [None, None];
    buffers[0] = if intf.rx0if_is_set() {
        Some(McpReceiveBuffer::Buffer0)
    } else {
        None
    };
    buffers[1] = if intf.rx1if_is_set() {
        Some(McpReceiveBuffer::Buffer1)
    } else {
        None
    };
    let mut new_frames = false;
    for b in buffers.iter() {
        if b.is_none() {
            continue;
        }
        let frame = mcp25625.receive(b.unwrap());
        match can_rx.push(frame, ()) {
            Ok(_) => {
                write_if_cps!(rtt, "{}RX: {:?}{}", color::GREEN, frame, color::DEFAULT);
            },
            Err(_) => {
                write_if_cps!(rtt, "{}RX overflow{}", color::YELLOW, color::DEFAULT);
            }
        }
        new_frames = true;
    }
    if new_frames {
        cx.spawn.can_rx().ok();
    }

    let tec = mcp25625.tec();
    let rec = mcp25625.rec();
    write_if_cps!(rtt, "TEC: {}, REC: {}", tec, rec);

    for _ in 0..3 {
        match can_tx.pop() {
            Some(frame) => {
                // Treat extended id frames as uavcan, use only one buffer for them to avoid priority inversion
                // If standard id frame is placed after extended one it will have to wait with this implementation!
                let buffer_choice = match frame.0.id {
                    FrameId::Standard(_) => { mcp25625::TxBufferChoice::Any }
                    FrameId::Extended(_) => { mcp25625::TxBufferChoice::OnlyOne(0) }
                };
                match mcp25625.send(frame.0.as_frame_ref(), buffer_choice, McpPriority::Highest) {
                    Ok(_) => {
                        // let _ = can_tx.heap.pop();
                        write_if_cps!(rtt, "{}TX: {:?}{}", color::CYAN, frame, color::DEFAULT);
                    }
                    Err(e) => {
                        write_if_cps!(rtt, "TX error: {:?}", e);
                        break;
                    }
                }
            },
            None => {
                break;
            }
        }
    }

    if errf.is_err() {
        mcp25625.reset_error_flags();
    }
    mcp25625.reset_interrupt_flags(0xFF);

    cfg_if! {
        if #[cfg(feature = "canbus-printstat")] {
            writeln!(rtt, "\n").ok();
            rtt.write_bytes(&[0xff, '0' as u8]);
        }
    }

    *mcp25625_state = Mcp25625State::Operational(Some((mcp25625, irq)));
}