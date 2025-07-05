#!/bin/sh
# west build --build-dir build . --pristine --sysbuild --board orlangur_ezurio_nrf54l15/nrf54l15/cpuapp -- -DBOARD_ROOT=$(pwd)/.. -DCONF_FILE=prj.conf --toolchain ${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
# west build --build-dir build_conv . --pristine --board nrf54l15dk/nrf54l15/cpuapp -- -DCONF_FILE=prj.conf --toolchain ${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
# west build --build-dir build_conv . --pristine --board nrf54l15dk/nrf54l15/cpuapp -- -DCONF_FILE=prj.conf -DZEPHYR_TOOLCHAIN_VARIANT=llvm --toolchain ${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
west build --build-dir build_conv . --pristine --board nrf54l15dk/nrf54l15/cpuapp -- -DCONF_FILE=prj.conf 
