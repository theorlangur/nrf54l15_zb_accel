#!/bin/sh
west build --build-dir build_dk . --pristine --sysbuild --board nrf54l15dk/nrf54l15/cpuapp -- -DCONF_FILE=prj.conf --toolchain ${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
# west build --build-dir build_dk . --pristine --sysbuild --board nrf54l15dk/nrf54l15/cpuapp -- -DCONF_FILE=prj.conf --toolchain ${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
# west build --build-dir build . --pristine --board nrf54l15dk/nrf54l15/cpuapp -- -DCONF_FILE=prj.conf --toolchain ${ZEPHYR_SDK_CMAKE_TOOLCHAIN_LLVM_PICO}
