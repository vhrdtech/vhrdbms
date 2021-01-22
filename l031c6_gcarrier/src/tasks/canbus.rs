// use core::fmt::Write;
use stm32l0xx_hal::exti::{Exti, GpioLine, ExtiLine};
use mcp25625::{McpReceiveBuffer, McpPriority, };
use crate::config;

pub fn canctrl_irq(cx: crate::canctrl_irq::Context) {
    Exti::unpend(GpioLine::from_raw_line(cx.resources.mcp25625_irq.pin_number()).unwrap());

    let _rtt = cx.resources.rtt;
    let mcp25625: &mut config::Mcp25625Instance = cx.resources.mcp25625;
    let can_tx: &mut config::CanTX = cx.resources.can_tx;
    let can_rx: &mut config::CanRX = cx.resources.can_rx;

    let intf = mcp25625.interrupt_flags();
    // writeln!(rtt, "{:?}", intf).ok();
    let errf = mcp25625.error_flags();
    // writeln!(rtt, "{:?}", errf).ok();

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
                can_rx.heap.push(frame).ok();
                new_frames = true;
            },
            Err(_) => {}
        }
    }
    if new_frames {
        cx.spawn.can_rx().ok();
    }
    // let tec = mcp25625.tec();
    // let rec = mcp25625.rec();
    // writeln!(rtt, "tec:{}, rec:{}", tec, rec).ok();
    //

    for _ in 0..3 {
        match can_tx.heap.peek() {
            Some(frame) => {
                match mcp25625.send(frame.as_raw_frame_ref(), McpPriority::Highest) {
                    Ok(_) => {
                        let _ = can_tx.heap.pop();
                    }
                    Err(_) => {
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
}