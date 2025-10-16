#!/bin/bash
west build --build-dir build_ezurio_llvm . --pristine \
    --board orlangur_ezurio_nrf54l15/nrf54l15/cpuapp -- \
    -DBOARD_ROOT=~/myapps/cpp/nrf \
    -DCONF_FILE="prj.conf config/cpp.conf config/zb.conf config/no_log.conf" \
    -DZEPHYR_TOOLCHAIN_VARIANT=llvm \
    -DCONFIG_LLVM_USE_LLD=y \
    -DCONFIG_COMPILER_RT_RTLIB=y \
    -DCMAKE_TOOLCHAIN_FILE=${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
