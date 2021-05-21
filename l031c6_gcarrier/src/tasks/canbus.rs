use core::fmt::Write;
use stm32l0xx_hal::exti::{Exti, GpioLine, ExtiLine};
use mcp25625::{McpReceiveBuffer, McpPriority, };
use crate::config;
use cfg_if::cfg_if;
use mcu_helper::color;

macro_rules! write_if_cps {
    ($rtt: expr, $($arg:tt)*) => {
        cfg_if! {
            if #[cfg(feature = "canbus-printstat")] {
                writeln!($rtt, $($arg)*).ok();
            }
        }
    };
}

pub fn canctrl_irq(cx: &mut crate::exti4_15::Context) {
    let irq_line = GpioLine::from_raw_line(cx.resources.mcp25625_irq.pin_number()).unwrap();
    // if !Exti::is_pending(irq_line) {
    //     return;
    // }
    Exti::unpend(irq_line);

    cfg_if! {
        if #[cfg(feature = "canbus-printstat")] {
            let rtt: &mut jlink_rtt::NonBlockingOutput = &mut cx.resources.rtt;
            rtt.write_bytes(&[0xff, '1' as u8]);
        }
    }
    let mcp25625: &mut config::Mcp25625Instance = cx.resources.mcp25625;
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
        match can_rx.pool.new_frame_from_raw(frame) {
            Ok(frame) => {
                match can_rx.heap.push(frame) {
                    Ok(_) => {
                        write_if_cps!(rtt, "{}RX: {:?}{}", color::GREEN, frame, color::DEFAULT);
                    },
                    Err(_) => {
                        write_if_cps!(rtt, "{}RX overflow{}", color::YELLOW, color::DEFAULT);
                    }
                }
                new_frames = true;
            },
            Err(_) => {}
        }
    }
    if new_frames {
        cx.spawn.can_rx().ok();
    }

    let tec = mcp25625.tec();
    let rec = mcp25625.rec();
    write_if_cps!(rtt, "TEC: {}, REC: {}", tec, rec);

    for _ in 0..3 {
        match can_tx.heap.pop() {
            Some(frame) => {
                match mcp25625.send(frame.as_raw_frame_ref(), McpPriority::Highest) {
                    Ok(_) => {
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
}