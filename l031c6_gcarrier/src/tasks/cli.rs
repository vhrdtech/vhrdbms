use bq769x0::BQ769x0;
use core::fmt::Write;
use crate::config;
use mcu_helper::color;
use crate::power_block::PowerBlockId;
use core::convert::TryFrom;
// use power_helper::power_block::PowerBlockControl;
// use stm32l0xx_hal::prelude::OutputPin;

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
    bq769x0: &mut BQ769x0,
    power_blocks: &mut config::PowerBlocksMap
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
                            writeln!(rtt, "cmds: afe, pb, tms").ok();
                        },
                        "afe" => {
                            afe_command(&mut args, i2c, bq769x0, rtt);
                        },
                        "pb" => {
                            power_blocks_command(&mut args, power_blocks, rtt);
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
    bq769x0: &mut BQ769x0,
    fmt: &mut dyn core::fmt::Write
) {
    let afe_part = args.next();
    match afe_part {
        Some(afe_part) => {
            let part_cmd = args.next();
            match afe_part {
                "dsg" => {
                    match one_of(part_cmd, &["off", "on"], fmt) {
                        Some(offon) => {
                            let r = bq769x0.discharge(i2c, offon != 0);
                            writeln!(fmt, "{:?}", r).ok();
                        },
                        None => {}
                    }
                    return;
                },
                "chg" => {
                    match one_of(part_cmd, &["off", "on"], fmt) {
                        Some(offon) => {
                            let r = bq769x0.charge(i2c, offon != 0);
                            writeln!(fmt, "{:?}", r).ok();
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
                                let r = bq769x0.sys_stat_reset(i2c);
                                writeln!(fmt, "SR: {:?}", r).ok();
                            }
                        },
                        None => {}
                    }
                    return;
                },
                "cells" => {
                    match bq769x0.cell_voltages(i2c) {
                        Ok(cells) => {
                            for (i, cell) in cells.iter().enumerate() {
                                let _ = writeln!(fmt, "C{}: {}mV", i + 1, cell);
                            }
                        }
                        Err(e) => {
                            let _ = writeln!(fmt, "RE={:?}", e);
                        }
                    }
                    return;
                }
                _ => {}
            }
        },
        None => {}
    }
    writeln!(fmt, "{}dsg, chg, stat, cells!{}", color::YELLOW, color::DEFAULT).ok();
}

fn power_blocks_command(
    args: &mut core::str::SplitAsciiWhitespace,
    power_blocks: &mut config::PowerBlocksMap,
    fmt: &mut dyn core::fmt::Write
) {
    let cmd = match args.next() {
        Some(cmd) => cmd,
        None => {
            writeln!(fmt, "{}pb <c> [b]{}", color::YELLOW, color::DEFAULT).ok();
            return;
        }
    };
    match cmd {
        "list" => {
            for (block, _) in power_blocks {
                let block_name: &'static str = (*block).into();
                writeln!(fmt, "{}", block_name).ok();
            }
        },
        cmd @ "off" | cmd @ "on" | cmd @ "stat" => {
            let block_name = match args.next() {
                Some(block_name) => block_name,
                None => {
                    writeln!(fmt, "{}pb <b> <c>{}", color::YELLOW, color::DEFAULT).ok();
                    return;
                }
            };
            let block_id = match PowerBlockId::try_from(block_name) {
                Ok(id) => id,
                Err(_) => {
                    writeln!(fmt, "{}wrong arg{}", color::YELLOW, color::DEFAULT).ok();
                    return;
                }
            };
            let block = power_blocks.get_mut(&block_id).expect("PB");

            match one_of(Some(cmd), &["off", "on", "stat"], fmt).unwrap() {
                0 => {
                    block.disable();
                },
                1 => {
                    block.enable();
                },
                2 => {
                    writeln!(fmt, "{}: en?:{} ok?:{}", block_name, block.is_enabled(), block.status_is_ok()).ok();
                },
                _ => unreachable!()
            }
        },
        _ => {}
    }
}
