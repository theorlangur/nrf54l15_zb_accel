#!/bin/bash
west build --build-dir build_dk_gcc . --pristine \
    --board nrf54l15dk/nrf54l15/cpuapp -- \
    -DCONF_FILE=prj.conf \
