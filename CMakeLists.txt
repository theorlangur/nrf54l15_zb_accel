# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(EXTRA_DTC_OVERLAY_FILE "dts.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf54l15_breakout_test)

target_sources(app PRIVATE 
        src/main.cpp
        submodules/esp_generic_lib/src/lib_linked_list.cpp
)

target_include_directories(app PRIVATE include)
target_include_directories(app PRIVATE submodules/esp_generic_lib/include)
if (${ZEPHYR_TOOLCHAIN_VARIANT} STREQUAL  "llvm")
        target_compile_options(app PRIVATE -fmacro-backtrace-limit=0)
        target_compile_options(app PUBLIC -D__cpp_concepts=202002L -Wno-builtin-macro-redefined -Wno-invalid-offsetof)
endif()
