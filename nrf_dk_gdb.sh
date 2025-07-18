#!/bin/bash
/opt/SEGGER/JLink/JLinkGDBServerCLExe -if SWD -speed 4000 -device cortex-m33 -select usb=1057749201 -port 44477 -rtos GDBServer/RTOSPlugin_Zephyr  -singlerun -nogui -halt -noir -silent
