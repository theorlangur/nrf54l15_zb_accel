#!/bin/sh
openocd -f interface/cmsis-dap.cfg -f target/nordic/nrf54l.cfg -c "gdb flash_program enable" -c "flash init"
