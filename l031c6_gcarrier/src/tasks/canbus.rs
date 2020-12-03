use core::fmt::Write;
use stm32l0xx_hal::exti::{Exti, GpioLine, ExtiLine};
use mcp25625::{McpReceiveBuffer, CanAddress, McpPriority};
use crate::config;

pub fn canctrl_irq(cx: crate::canctrl_irq::Context) {
    Exti::unpend(GpioLine::from_raw_line(cx.resources.mcp25625_irq.pin_number()).unwrap());

    let rtt = cx.resources.rtt;
    let mcp25625: &mut config::Mcp25625Instance = cx.resources.mcp25625;

    let intf = mcp25625.interrupt_flags();
    // writeln!(rtt, "{:?}", intf).ok();
    let errf = mcp25625.error_flags();
    // writeln!(rtt, "{:?}", errf).ok();

    if intf.rx0if_is_set() {
        let message = mcp25625.receive(McpReceiveBuffer::Buffer0);
        writeln!(rtt, "rx0:{:?}", message).ok();
    }
    if intf.rx1if_is_set() {
        let message = mcp25625.receive(McpReceiveBuffer::Buffer1);
        writeln!(rtt, "rx1:{:?}", message).ok();
    }
    // let tec = mcp25625.tec();
    // let rec = mcp25625.rec();
    // writeln!(rtt, "tec:{}, rec:{}", tec, rec).ok();
    //


    //
    // let mut empty_tx_bufs = 0u8;
    // if intf.tx0if_is_set() {
    //     empty_tx_bufs = empty_tx_bufs + 1
    // }
    // if intf.tx1if_is_set() {
    //     empty_tx_bufs = empty_tx_bufs + 1
    // }
    // if intf.tx2if_is_set() {
    //     empty_tx_bufs = empty_tx_bufs + 1
    // }
    // if empty_tx_bufs > 0 {
    //     writeln!(rtt, "{} tx buffers free", empty_tx_bufs).ok();
    // }
    //
    //
    let r = mcp25625.send(CanAddress::extended(0x1769A5F3), &[0xaa, 0xbb, 0xcc, 0xdd], McpPriority::Low);
    writeln!(rtt, "tx: {:?}", r).ok();

    if errf.is_err() {
        mcp25625.reset_error_flags();
    }
    mcp25625.reset_interrupt_flags(0xFF);
}