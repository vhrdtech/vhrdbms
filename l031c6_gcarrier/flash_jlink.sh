arm-none-eabi-objcopy -O binary ../target/thumbv6m-none-eabi/release/l031c6_gcarrier ../target/thumbv6m-none-eabi/release/l031c6_gcarrier.bin
JLinkExe -commanderscript ./flash.jlink
