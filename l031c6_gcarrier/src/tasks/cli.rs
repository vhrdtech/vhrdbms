use core::fmt::Write;
use crate::{config, tasks};
use mcu_helper::color;
// use crate::power_block::PowerBlockId;
use core::convert::TryFrom;
use crate::tasks::bms::BmsEvent;
// use power_helper::power_block::PowerBlockControl;
// use stm32l0xx_hal::prelude::OutputPin;
use btoi::{btoi, ParseIntegerError, btoi_radix};
use no_std_compat::prelude::v1::*;
use crate::hal::prelude::*;

macro_rules! ok_or_return {
    ($e: expr, $fmt: ident, $message: expr) => {
        match $e {
            Ok(x) => x,
            Err(_) => {
                writeln!($fmt, "{}{}{}", color::YELLOW, $message, color::DEFAULT).ok();
                return;
            }
        }
    }
}

macro_rules! some_or_return {
    ($e: expr, $fmt: ident, $message: expr) => {
        match $e {
            Some(x) => x,
            None => {
                writeln!($fmt, "{}{}{}", color::YELLOW, $message, color::DEFAULT).ok();
                return;
            }
        }
    }
}

fn print_str_array(strs: &[&str], fmt: &mut dyn core::fmt::Write) {
    write!(fmt, "{}", color::YELLOW).ok();
    for i in 0..strs.len() {
        write!(fmt, "{}", strs[i]).ok();
        if i != strs.len() - 1 {
            write!(fmt, "/").ok();
        }
    }
    writeln!(fmt, "{}", color::DEFAULT).ok();
}

pub fn cli(
    rtt: &mut jlink_rtt::NonBlockingOutput,
    i2c: &mut config::InternalI2c,
    bq769x0: &mut config::BQ769x0,
    spawn: crate::idle::Spawn,
    afe_io: &mut tasks::bms::AfeIo,
    mcp25625: &mut config::Mcp25625Instance
) {
    let mut rtt_down = [0u8; 128];
    let rtt_down_len = jlink_rtt::try_read(&mut rtt_down);
    if rtt_down_len > 0 {
        //let mut space_idxs = [0u8; 8];
        let args = core::str::from_utf8(&rtt_down);
        match args {
            Ok(args) => {
                let mut args = args.split_ascii_whitespace();
                let cmd = args.next();
                if cmd.is_some() {
                    let cmd = cmd.unwrap();
                    match cmd {
                        "help" => {
                            writeln!(rtt, "cmd [a] [a]...").ok();
                            writeln!(rtt, "cmds: afe, pb, imx, reset, halt, tms").ok();
                        },
                        "afe" => {
                            afe_command(&mut args, i2c, bq769x0, afe_io, rtt);
                        },
                        // "pb" => {
                        //     power_blocks_command(&mut args, power_blocks, rtt);
                        // },
                        "reset" => {
                            cortex_m::peripheral::SCB::sys_reset(); // -> !
                        },
                        "halt" => {
                            spawn.bms_event(BmsEvent::Halt).ok();
                        },
                        "i2c" => {
                            i2c_command(&mut args, i2c, rtt);
                        },
                        "mcp" => {
                            mcp_command(&mut args, mcp25625, rtt);
                        },
                        "stack" => {
                            print_stack_usage(rtt);
                        },
                        _ => {
                            writeln!(rtt, "{}UC{}", color::YELLOW, color::DEFAULT).ok();
                        }
                    }
                }
            },
            Err(_) => {
                writeln!(rtt, "{}BC{}", color::YELLOW, color::DEFAULT).ok();
            }
        }

        writeln!(rtt, "{}> {}", color::GREEN, color::DEFAULT).ok();
    }
}

fn one_of(arg: Option<&str>, choices: &[&'static str], fmt: &mut dyn core::fmt::Write) -> Option<usize> {
    let arg = match arg {
        Some(arg) => arg,
        None => {
            print_str_array(choices, fmt);
            return None;
        }
    };
    match choices.iter().position(|&c| c == arg) {
        Some(choice) => Some(choice),
        None => {
            print_str_array(choices, fmt);
            None
        }
    }
}

fn afe_command(
    args: &mut core::str::SplitAsciiWhitespace,
    i2c: &mut config::InternalI2c,
    bq769x0: &mut config::BQ769x0,
    afe_io: &mut tasks::bms::AfeIo,
    fmt: &mut dyn core::fmt::Write,
) {
    let afe_part = args.next();
    match afe_part {
        Some(afe_part) => {
            let part_cmd = args.next();
            match afe_part {
                "dsg" => {
                    match one_of(part_cmd, &["off", "on", "ovrd"], fmt) {
                        Some(offon) => {
                            match offon {
                                0 => {
                                    let r = crate::tasks::bms::afe_discharge(i2c, bq769x0, false);
                                    writeln!(fmt, "DSG en: {:?}", r).ok();
                                    afe_io.afe_dsg_override.set_low().ok();
                                },
                                1 => {
                                    let r = crate::tasks::bms::afe_discharge(i2c, bq769x0, true);
                                    writeln!(fmt, "DSG en: {:?}", r).ok();
                                },
                                2 => {
                                    writeln!(fmt, "{}DSG OVRD{}", color::RED, color::DEFAULT).ok();
                                    afe_io.afe_dsg_override.set_high().ok();
                                },
                                _ => {}
                            }
                        },
                        None => {}
                    }
                    return;
                },
                "chg" => {
                    match one_of(part_cmd, &["off", "on", "ovrd"], fmt) {
                        Some(offon) => {
                            match offon {
                                0 => {
                                    let r = bq769x0.charge(i2c, false);
                                    writeln!(fmt, "{:?}", r).ok();
                                    afe_io.afe_chg_override.set_low().ok();
                                },
                                1 => {
                                    let r = bq769x0.charge(i2c, true);
                                    writeln!(fmt, "{:?}", r).ok();
                                },
                                2 => {
                                    afe_io.afe_chg_override.set_high().ok();
                                    writeln!(fmt, "{}CHG OVRD{}", color::RED, color::DEFAULT).ok();
                                },
                                _ => {},
                            }
                        },
                        None => {}
                    }
                    return;
                },
                "stat" => {
                    match one_of(part_cmd, &["show", "reset"], fmt) {
                        Some(showreset) => {
                            if showreset == 0 {
                                let stat = bq769x0.sys_stat(i2c);
                                writeln!(fmt, "{:?}", stat).ok();
                            } else if showreset == 1 {
                                let r = bq769x0.sys_stat_reset(i2c, bq769x0::SysStat::ALL);
                                writeln!(fmt, "SR: {:?}", r).ok();
                            }
                        },
                        None => {}
                    }
                    return;
                },
                "cells" => {
                    let balancing_state = bq769x0.balancing_state(i2c).unwrap_or(0);
                    match bq769x0.cell_voltages(i2c) {
                        Ok(cells) => {
                            for (i, cell) in cells.iter().enumerate() {
                                let is_balancing = balancing_state & (1 << i) != 0;
                                let is_balancing = if is_balancing { " [BAL]" } else { "" };
                                let _ = writeln!(fmt, "C{}: {}{}", i + 1, cell, is_balancing);
                            }
                        }
                        Err(e) => {
                            let _ = writeln!(fmt, "RE={:?}", e);
                        }
                    }
                    return;
                },
                "vi" => {
                    let i = bq769x0.current(i2c);
                    let v = bq769x0.voltage(i2c);
                    let p: Result<i32, bq769x0::Error> = i.and_then(|i| {
                        v.map(|v| i.0 * (v.0 as i32) / 1000)
                    });
                    match p {
                        Ok(p) => {
                            let intpart = p / 1000;
                            let fractpart = p - intpart * 1000;
                            let _ = writeln!(fmt, "V: {}, I:{}, Power: {}.{}W", v.unwrap(), i.unwrap(), intpart, fractpart.abs());
                        }
                        Err(e) => {
                            let _ = writeln!(fmt, "Read err={:?}", e);
                        }
                    }
                    return;
                },
                "bal" => {
                    let subcmd = some_or_return!(part_cmd, fmt, "bal en <n>/show/off");
                    match subcmd {
                        "en" => {
                            let cells = some_or_return!(args.next(), fmt, "bal en 10101");
                            let cells: Result<u8, ParseIntegerError> = btoi_radix(cells.as_bytes(), 2);
                            let cells = ok_or_return!(cells, fmt, "not a binary number");
                            if (cells & 0b10101 != 0) && (cells & 0b01010 != 0) {
                                writeln!(fmt, "Cannot balance consecutive cells!").ok();
                                return;
                            }
                            writeln!(fmt, "Enabling balance: {:?}", bq769x0.enable_balancing(i2c, cells)).ok();
                        },
                        "off" => {
                            let r = bq769x0.enable_balancing(i2c, 0);
                            writeln!(fmt, "Balancing off {:?}", r).ok();
                        },
                        "show" => {
                            let r = bq769x0.balancing_state(i2c);
                            writeln!(fmt, "Balancing (C5:C1): {:05b}", r.unwrap_or(0)).ok();
                        },
                        _ => {
                            writeln!(fmt, "Unknown command").ok();
                        }
                    }
                    return;
                }
                _ => { writeln!(fmt, "Unknown command").ok(); }
            }
        },
        None => {}
    }
    writeln!(fmt, "{}dsg, chg, stat, cells, vi, bal!{}", color::YELLOW, color::DEFAULT).ok();
}

// fn power_blocks_command(
//     args: &mut core::str::SplitAsciiWhitespace,
//     power_blocks: &mut config::PowerBlocksMap,
//     fmt: &mut dyn core::fmt::Write
// ) {
//     let cmd = match args.next() {
//         Some(cmd) => cmd,
//         None => {
//             writeln!(fmt, "{}pb <c> [b]{}", color::YELLOW, color::DEFAULT).ok();
//             return;
//         }
//     };
//     match cmd {
//         "list" => {
//             for (block, _) in power_blocks {
//                 let block_name: &'static str = (*block).into();
//                 writeln!(fmt, "{}", block_name).ok();
//             }
//         },
//         cmd @ "off" | cmd @ "on" | cmd @ "stat" => {
//             let block_name = match args.next() {
//                 Some(block_name) => block_name,
//                 None => {
//                     writeln!(fmt, "{}pb <b> <c>{}", color::YELLOW, color::DEFAULT).ok();
//                     return;
//                 }
//             };
//             let block_id = match PowerBlockId::try_from(block_name) {
//                 Ok(id) => id,
//                 Err(_) => {
//                     writeln!(fmt, "{}wrong arg{}", color::YELLOW, color::DEFAULT).ok();
//                     return;
//                 }
//             };
//             let block = power_blocks.get_mut(&block_id).expect("PB");
//
//             match one_of(Some(cmd), &["off", "on", "stat"], fmt).unwrap() {
//                 0 => {
//                     block.disable();
//                 },
//                 1 => {
//                     block.enable();
//                 },
//                 2 => {
//                     writeln!(fmt, "{}: {:?}", block_name, block).ok();
//                 },
//                 _ => unreachable!()
//             }
//         },
//         _ => { writeln!(fmt, "Unknown command").ok(); }
//     }
// }

fn i2c_command(
    args: &mut core::str::SplitAsciiWhitespace,
    i2c: &mut config::InternalI2c,
    fmt: &mut dyn core::fmt::Write
) {
    const MAX_TRANSFER: usize = 64;

    let subcmd = some_or_return!(args.next(), fmt, "i2c read/scan");
    let mut buf = [0u8; MAX_TRANSFER];
    match subcmd {
        "read" => {
            let addr = some_or_return!(args.next(), fmt, "no addr");
            let addr: Result<u8, ParseIntegerError> = btoi_radix(addr.as_bytes(), 16);
            let addr = ok_or_return!(addr, fmt, "wrong addr");

            let len = some_or_return!(args.next(), fmt, "no len");
            let len: Result<u8, ParseIntegerError> = btoi(len.as_bytes());
            let len = ok_or_return!(len, fmt, "wrong len");
            let len = if len <= MAX_TRANSFER as u8 {
                len as usize
            } else {
                writeln!(fmt, "len>{}", MAX_TRANSFER).ok();
                return;
            };
            let r = i2c.read(addr, &mut buf[..len]);
            match r {
                Ok(()) => {
                    writeln!(fmt, "{}ok:{}", color::GREEN, color::DEFAULT).ok();
                    for i in 0..len {
                        writeln!(fmt, "\t{:02x}: {:02x}={:08b}", addr + i as u8, buf[i], buf[i]).ok();
                    }
                },
                Err(e) => {
                    writeln!(fmt, "{}err:{:?}{}", color::RED, e, color::DEFAULT).ok();
                }
            };
        },
        "scan" => {
            for i in 0..=127u8 {
                if i % 8 == 0 {
                    writeln!(fmt, "").ok();
                }
                match i2c.read(i, &mut buf[0..1]) {
                    Ok(_) => {
                        write!(fmt, "{:02x} ", i).ok();
                    },
                    Err(_) => {
                        write!(fmt, ".. ").ok();
                    }
                }

            }
        },
        _ => { writeln!(fmt, "Unknown command").ok(); }
    };
}

fn mcp_command(
    args: &mut core::str::SplitAsciiWhitespace,
    mcp25625: &mut config::Mcp25625Instance,
    fmt: &mut dyn core::fmt::Write
) {
    let subcmd = some_or_return!(args.next(), fmt, "mcp dump");
    match subcmd {
        "dump" => {
            writeln!(fmt, "\n\nRXF0-RXF2:").ok();
            for addr in 0b0000_0000..=0b0000_1011 {
                let reg = mcp25625.read_reg(addr);
                writeln!(fmt, "{:08b} = {:08b}", addr, reg).ok();
            }
            writeln!(fmt, "\n\nRXF3-RXF5:").ok();
            for addr in 0b0001_0000..=0b0001_1011 {
                let reg = mcp25625.read_reg(addr);
                writeln!(fmt, "{:08b} = {:08b}", addr, reg).ok();
            }
            writeln!(fmt, "\n\nRXM0-RXM1:").ok();
            for addr in 0b0010_0000..=0b0010_0111 {
                let reg = mcp25625.read_reg(addr);
                writeln!(fmt, "{:08b} = {:08b}", addr, reg).ok();
            }
            writeln!(fmt, "\n\nTXB0:").ok();
            for addr in 0b0011_0000..=0b0011_1101 {
                let reg = mcp25625.read_reg(addr);
                writeln!(fmt, "{:08b} = {:08b}", addr, reg).ok();
            }
            writeln!(fmt, "\n\nTXB1:").ok();
            for addr in 0b0100_0000..=0b100_1101 {
                let reg = mcp25625.read_reg(addr);
                writeln!(fmt, "{:08b} = {:08b}", addr, reg).ok();
            }
            writeln!(fmt, "\n\nTXB2:").ok();
            for addr in 0b0101_0000..=0b0101_1101 {
                let reg = mcp25625.read_reg(addr);
                writeln!(fmt, "{:08b} = {:08b}", addr, reg).ok();
            }
        },
        _ => { writeln!(fmt, "Unknown command").ok(); }
    }
}

fn print_stack_usage(
    fmt: &mut dyn core::fmt::Write
) {
    use crate::util::{stack_lower_bound, stack_upper_bound, STACK_PROBE_MAGICWORD};
    let stack_size_words = (stack_upper_bound() - stack_lower_bound()) / 4;
    let p = stack_lower_bound() as *mut u32;
    let mut untouched_words = 0;
    for i in 0..stack_size_words {
        if unsafe { p.offset(i as isize).read() } == STACK_PROBE_MAGICWORD {
            untouched_words += 1;
        } else {
            writeln!(fmt, "Min free: {}B", untouched_words * 4).ok();
            break;
        }
    }
}