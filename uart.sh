#!/bin/sh
#/dev/ttyUSB0 - depending on where the CH340 USB-to-UART will be detected
minicom -b 115200 -D /dev/ttyUSB0
