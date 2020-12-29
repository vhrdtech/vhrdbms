use bq769x0::{BQ769x0};
use core::fmt::Write;
use crate::config;
use mcu_helper::color;
use crate::power_block::PowerBlockId;
use core::convert::TryFrom;
use tca9535::tca9534::Tca9534;
use crate::config::TCA_MXM_BOOT_SRC_PIN;
use crate::tasks::bms::BmsEvent;
// use power_helper::power_block::PowerBlockControl;
// use stm32l0xx_hal::prelude::OutputPin;
use btoi::{btoi, ParseIntegerError, btoi_radix};
use no_std_compat::prelude::v1::*;
use crate::hal::prelude::*;

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
    power_blocks: &mut config::PowerBlocksMap,
    tca9534: &mut Tca9534<config::InternalI2c>,
    spawn: crate::idle::Spawn
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
                            afe_command(&mut args, i2c, bq769x0, rtt);
                        },
                        "pb" => {
                            power_blocks_command(&mut args, power_blocks, rtt);
                        },
                        "imx" => {
                            imx_command(&mut args, i2c, tca9534, rtt);
                        },
                        "tms" => {
                            tms_command(&mut args, i2c, tca9534, power_blocks, rtt);
                        },
                        "reset" => {
                            cortex_m::peripheral::SCB::sys_reset(); // -> !
                        },
                        "halt" => {
                            spawn.bms_event(BmsEvent::Halt).ok();
                        },
                        "i2c" => {
                            i2c_command(&mut args, i2c, rtt);
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
                            for _ in 0..10 {
                                let r = bq769x0.sys_stat_reset(i2c);
                                writeln!(fmt, "st_res: {:?}", r).ok();
                                let _ = bq769x0.discharge(i2c, offon != 0);
                                cortex_m::asm::delay(400_000);
                                match bq769x0.sys_stat(i2c) {
                                    Ok(stat) => {
                                        if !stat.scd_is_set() {
                                            let _ = writeln!(fmt, "no scd, done");
                                            break;
                                        }
                                    }
                                    Err(_) => {}
                                }
                                cortex_m::asm::delay(50_000);
                            }
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
                                let _ = writeln!(fmt, "C{}: {}", i + 1, cell);
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
                }
                _ => {}
            }
        },
        None => {}
    }
    writeln!(fmt, "{}dsg, chg, stat, cells, vi!{}", color::YELLOW, color::DEFAULT).ok();
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
                    writeln!(fmt, "{}: {:?}", block_name, block).ok();
                },
                _ => unreachable!()
            }
        },
        _ => {}
    }
}

fn imx_command(
    args: &mut core::str::SplitAsciiWhitespace,
    i2c: &mut config::InternalI2c,
    tca9534: &mut Tca9534<config::InternalI2c>,
    fmt: &mut dyn core::fmt::Write
) {
    let subcmd = match args.next() {
        Some(subcmd) => subcmd,
        None => {
            writeln!(fmt, "{}imx push/rel/usd/emmc{}", color::YELLOW, color::DEFAULT).ok();
            return;
        }
    };
    use tca9535::tca9534::Port;
    let r = match subcmd {
        "rel" => {
            tca9534.modify_outputs(i2c, config::TCA_MXM_ON_OFF_PIN, Port::empty()) // 0 = release on/off button
        },
        "push" => {
            tca9534.modify_outputs(i2c, config::TCA_MXM_ON_OFF_PIN, config::TCA_MXM_ON_OFF_PIN) // 1 = push on/off button
        },
        "emmc" => {
            tca9534.modify_outputs(i2c, config::TCA_MXM_BOOT_SRC_PIN, Port::empty()) // 0 = eMMC
        },
        "usd" => {
            tca9534.modify_outputs(i2c, config::TCA_MXM_BOOT_SRC_PIN, TCA_MXM_BOOT_SRC_PIN) // 1 = uSD
        },
        _ => {
            writeln!(fmt, "{}wrong command{}", color::YELLOW, color::DEFAULT).ok();
            return;
        }
    };
    match r {
        Ok(_) => {
            writeln!(fmt, "{}ok{}", color::GREEN, color::DEFAULT).ok();
        },
        Err(_) => {
            writeln!(fmt, "{}error{}", color::RED, color::DEFAULT).ok();
        }
    }
}

fn tms_command(
    args: &mut core::str::SplitAsciiWhitespace,
    i2c: &mut config::InternalI2c,
    tca9534: &mut Tca9534<config::InternalI2c>,
    power_blocks: &mut config::PowerBlocksMap,
    fmt: &mut dyn core::fmt::Write
) {
    let subcmd = match args.next() {
        Some(subcmd) => subcmd,
        None => {
            writeln!(fmt, "{}tms run/jtag/tdo0/tdoz{}", color::YELLOW, color::DEFAULT).ok();
            return;
        }
    };
    use tca9535::tca9534::Port;
    let r = match subcmd {
        "run" => {
            power_blocks.get_mut(&PowerBlockId::Switch3V3Tms).unwrap().disable();
            let r = tca9534.modify_outputs(i2c, config::TCA_TMS_TDO_PIN, Port::empty());
            cortex_m::asm::delay(500_000);
            power_blocks.get_mut(&PowerBlockId::DcDc3V3Hc).unwrap().enable();
            power_blocks.get_mut(&PowerBlockId::Switch3V3Tms).unwrap().enable();
            r
        },
        "jtag" => {
            power_blocks.get_mut(&PowerBlockId::Switch3V3Tms).unwrap().disable();
            let _ = tca9534.modify_outputs(i2c, config::TCA_TMS_TDO_PIN, config::TCA_TMS_TDO_PIN);
            cortex_m::asm::delay(500_000);
            power_blocks.get_mut(&PowerBlockId::DcDc3V3Hc).unwrap().enable();
            power_blocks.get_mut(&PowerBlockId::Switch3V3Tms).unwrap().enable();
            cortex_m::asm::delay(500_000);
            let r = tca9534.modify_outputs(i2c, config::TCA_TMS_TDO_PIN, Port::empty());
            r
        },
        "tdo0" => {
            tca9534.modify_outputs(i2c, config::TCA_TMS_TDO_PIN, config::TCA_TMS_TDO_PIN)
        },
        "tdoz" => {
            tca9534.modify_outputs(i2c, config::TCA_TMS_TDO_PIN, Port::empty())
        },
        _ => {
            writeln!(fmt, "{}wrong command{}", color::YELLOW, color::DEFAULT).ok();
            return;
        }
    };
    match r {
        Ok(_) => {
            writeln!(fmt, "{}ok{}", color::GREEN, color::DEFAULT).ok();
        },
        Err(_) => {
            writeln!(fmt, "{}error{}", color::RED, color::DEFAULT).ok();
        }
    }
}

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
        _ => {}
    };
}