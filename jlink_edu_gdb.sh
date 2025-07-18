#!/bin/bash
/opt/SEGGER/JLink/JLinkGDBServerCLExe -if SWD -speed 4000 -device cortex-m33 -select usb=802005000 -port 44477 -rtos GDBServer/RTOSPlugin_Zephyr  -singlerun -nogui -halt -noir -silent
