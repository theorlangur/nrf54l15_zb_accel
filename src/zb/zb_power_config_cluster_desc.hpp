#ifndef ZB_POWER_CONFIG_CLUSTER_DESC_HPP_
#define ZB_POWER_CONFIG_CLUSTER_DESC_HPP_

#include "zb_main.hpp"

extern "C"
{
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zb_nrf_platform.h>
}

namespace zb
{
    static constexpr uint16_t kZB_ZCL_CLUSTER_ID_POWER_CFG = 0x0001;
    struct zb_zcl_power_cfg_mains_info_t
    {
        uint16_t mains_voltage;
        uint8_t mains_freq;
    };

    template<> constexpr auto get_cluster_description<zb_zcl_power_cfg_mains_info_t>()
    {
        using T = zb_zcl_power_cfg_mains_info_t;
        return cluster_struct_desc_t<
            {.id = kZB_ZCL_CLUSTER_ID_POWER_CFG},
            cluster_attributes_desc_t<
                cluster_mem_desc_t{.m = &T::mains_voltage,.id = 0x0000, .a=Access::Read}
                ,cluster_mem_desc_t{.m = &T::mains_freq,.id = 0x0001, .a=Access::Read}
            >{}
        >{};
    }

    struct zb_mains_alarm_mask_t
    {
        static constexpr Type TypeId() { return Type::Map8; }

        uint8_t voltage_low  : 1 = 0;
        uint8_t voltage_high : 1 = 0;
        uint8_t supply_lost  : 1 = 0;
    };

    struct zb_zcl_power_cfg_mains_settings_t: zb_zcl_power_cfg_mains_info_t
    {
        zb_mains_alarm_mask_t mains_alarm_mask;
        uint16_t mains_voltage_min_threshold;
        uint16_t mains_voltage_max_threshold;
        uint16_t mains_voltage_dwell_trip_point;
    };

    template<> constexpr auto get_cluster_description<zb_zcl_power_cfg_mains_settings_t>()
    {
        using T = zb_zcl_power_cfg_mains_settings_t;
        return get_cluster_description<zb_zcl_power_cfg_mains_info_t>() + cluster_struct_desc_t<
            {.id = kZB_ZCL_CLUSTER_ID_POWER_CFG},
            cluster_attributes_desc_t<
                 cluster_mem_desc_t{.m = &T::mains_alarm_mask,              .id = 0x0010, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::mains_voltage_min_threshold,   .id = 0x0011, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::mains_voltage_max_threshold,   .id = 0x0012, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::mains_voltage_dwell_trip_point,.id = 0x0013, .a=Access::RW}
            >{}
        >{};
    }

    struct zb_zcl_power_cfg_battery_info_t
    {
        uint8_t batt_voltage;
        uint8_t batt_percentage_remaining;
    };

    template<> constexpr auto get_cluster_description<zb_zcl_power_cfg_battery_info_t>()
    {
        using T = zb_zcl_power_cfg_battery_info_t;
        return cluster_struct_desc_t<
            {.id = kZB_ZCL_CLUSTER_ID_POWER_CFG},
            cluster_attributes_desc_t<
                cluster_mem_desc_t{.m = &T::batt_voltage,              .id = 0x0020, .a=Access::RP}
                ,cluster_mem_desc_t{.m = &T::batt_percentage_remaining,.id = 0x0021, .a=Access::RP}
            >{}
        >{};
    }

    enum class BatterySize: uint8_t
    {
        NoBattery = 0,
        BuiltIn = 1,
        Other = 2,
        AA = 3,
        AAA = 4,
        C = 5,
        D = 6,
        CR2 = 7,
        CR123A= 8,
        Unkown = 0xff
    };

    struct zb_battery_alarm_mask_t
    {
        static constexpr Type TypeId() { return Type::Map8; }

        uint8_t low    : 1;
        uint8_t alarm1 : 1;
        uint8_t alarm2 : 1;
        uint8_t alarm3 : 1;
    };

    struct zb_battery_alarm_state_t
    {
        static constexpr Type TypeId() { return Type::Map32; }

        uint32_t low_src1    : 1;
        uint32_t alarm1_src1 : 1;
        uint32_t alarm2_src1 : 1;
        uint32_t alarm3_src1 : 1;
        uint32_t unused1     : 6;
        uint32_t low_src2    : 1;
        uint32_t alarm1_src2 : 1;
        uint32_t alarm2_src2 : 1;
        uint32_t alarm3_src2 : 1;
        uint32_t unused2     : 6;
        uint32_t low_src3    : 1;
        uint32_t alarm1_src3 : 1;
        uint32_t alarm2_src3 : 1;
        uint32_t alarm3_src3 : 1;
        uint32_t unused3     : 6;
        uint32_t mains_lost  : 1;
    };

    struct zb_zcl_power_cfg_battery_settings_t: zb_zcl_power_cfg_battery_info_t
    {
        ZigbeeStr<16> batt_manufacture;
        BatterySize batt_size;
        uint16_t batt_AHr_rating;
        uint8_t batt_quantity;
        uint8_t batt_rated_voltage;
        zb_battery_alarm_mask_t batt_alarm_mask;
        uint8_t batt_voltage_min_threshold;
        uint8_t batt_voltage_threshold1;
        uint8_t batt_voltage_threshold2;
        uint8_t batt_voltage_threshold3;
        uint8_t batt_percentage_min_threshold;
        uint8_t batt_percentage_threshold1;
        uint8_t batt_percentage_threshold2;
        uint8_t batt_percentage_threshold3;
        zb_battery_alarm_state_t batt_alarm_state;
    };

    template<> constexpr auto get_cluster_description<zb_zcl_power_cfg_battery_settings_t>()
    {
        using T = zb_zcl_power_cfg_battery_settings_t;
        return get_cluster_description<zb_zcl_power_cfg_battery_info_t>() + cluster_struct_desc_t<
            {.id = kZB_ZCL_CLUSTER_ID_POWER_CFG},
            cluster_attributes_desc_t<
                 cluster_mem_desc_t{.m = &T::batt_manufacture,              .id = 0x0030, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_size,                     .id = 0x0031, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_AHr_rating,               .id = 0x0032, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_quantity,                 .id = 0x0033, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_rated_voltage,            .id = 0x0034, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_alarm_mask,               .id = 0x0035, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_voltage_min_threshold,    .id = 0x0036, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_voltage_threshold1,       .id = 0x0037, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_voltage_threshold2,       .id = 0x0038, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_voltage_threshold3,       .id = 0x0039, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_percentage_min_threshold, .id = 0x003a, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_percentage_threshold1,    .id = 0x003b, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_percentage_threshold2,    .id = 0x003c, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_percentage_threshold3,    .id = 0x003d, .a=Access::RW}
                ,cluster_mem_desc_t{.m = &T::batt_alarm_state,              .id = 0x003e, .a=Access::RW}
            >{}
        >{};
    }


DEFINE_NULL_CLUSTER_INIT_FOR(kZB_ZCL_CLUSTER_ID_POWER_CFG);
}
#endif
