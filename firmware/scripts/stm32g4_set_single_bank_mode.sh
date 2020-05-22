#!/bin/bash
set -o verbose
set -o errexit

openocd -d3 -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/olimex-arm-jtag-swd.cfg -f target/stm32g4x.cfg  -c "reset_config srst_only srst_nogate connect_assert_srst" -c "init; reset halt; stm32l4x option_write 0 0x20 0 0x00400000"
