#!/bin/bash
# west build --build-dir build . --pristine --sysbuild --board orlangur_ezurio_nrf54l15/nrf54l15/cpuapp -- -DBOARD_ROOT=$(pwd)/.. -DCONF_FILE=prj.conf --toolchain ${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
# west build --build-dir build_conv . --pristine --board nrf54l15dk/nrf54l15/cpuapp -- -DCONF_FILE=prj.conf --toolchain ${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
west build --build-dir build_ezurio_llvm . --pristine \
    --board orlangur_ezurio_nrf54l15/nrf54l15/cpuapp -- \
    -DBOARD_ROOT=~/myapps/cpp/nrf \
    -DCONF_FILE="prj.conf config/cpp.conf config/zb.conf config/log.conf" \
    -DZEPHYR_TOOLCHAIN_VARIANT=llvm \
    -DCONFIG_LLVM_USE_LLD=y \
    -DCONFIG_COMPILER_RT_RTLIB=y \
    -DCMAKE_TOOLCHAIN_FILE=${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
# /home/orlangur/ncs/toolchains/llvm/nrf_llvm_pico_7cbc0036f4_m33.cmake
# --toolchain ${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
# west build --build-dir build_conv . --pristine --board nrf54l15dk/nrf54l15/cpuapp -- -DCONF_FILE=prj.conf
