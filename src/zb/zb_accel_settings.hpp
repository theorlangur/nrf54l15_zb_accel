#ifndef ZB_ACCEL_SETTINGS_H_
#define ZB_ACCEL_SETTINGS_H_

#include <nrfzbcpp/zb_main.hpp>

namespace zb
{
    static constexpr uint16_t kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS = 0xfc01;
    static constexpr uint16_t kZB_ATTR_ID_MAIN_SETTINGS = 0x0000;
    static constexpr uint16_t kZB_ATTR_ID_WAKE_SLEEP_THRESHOLD = 0x0001;
    static constexpr uint16_t kZB_ATTR_ID_SLEEP_DURATION = 0x0002;
    static constexpr uint16_t kZB_ATTR_ID_SLEEP_TRACKING_RATE = 0x0003;
    static constexpr uint16_t kZB_ATTR_ID_ACTIVE_TRACKING_RATE = 0x0004;

    enum class inactive_odr_t: uint8_t
    {
        Same = 0,//same as active
        _1_6Hz = 1,
        _3Hz = 2,
        _6Hz = 3,
    };

    struct zb_zcl_accel_settings_t
    {
        union{
            struct{
                uint32_t enable_x      : 1 = 0;
                uint32_t enable_y      : 1 = 0;
                uint32_t enable_z      : 1 = 1;
                uint32_t track_wake_up : 1 = 0;
                uint32_t track_sleep   : 1 = 0;
                uint32_t track_flip    : 1 = 1;
            }flags{};
            uint32_t flags_dw;
        };
        uint8_t wake_sleep_threshold = 1;
        uint8_t sleep_duration = 0;
        inactive_odr_t sleep_odr = inactive_odr_t::Same;
        uint8_t  active_odr = 0;/*lis2du12_odr_t*/ //0 - whatever is provided by dts
    };

    template<>
    struct zcl_description_t<zb_zcl_accel_settings_t>{
        static constexpr auto get()
        {
            using T = zb_zcl_accel_settings_t;
            return cluster_struct_desc_t<
                cluster_info_t{.id = kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS},
                cluster_attributes_desc_t<
                    cluster_mem_desc_t{.m = &T::flags_dw,.id = kZB_ATTR_ID_MAIN_SETTINGS, .a=Access::RW, .type=Type::Map32}
                    ,cluster_mem_desc_t{.m = &T::wake_sleep_threshold,.id = kZB_ATTR_ID_WAKE_SLEEP_THRESHOLD, .a=Access::RW}
                    ,cluster_mem_desc_t{.m = &T::sleep_duration,.id = kZB_ATTR_ID_SLEEP_DURATION, .a=Access::RW}
                    ,cluster_mem_desc_t{.m = &T::sleep_odr,.id = kZB_ATTR_ID_SLEEP_TRACKING_RATE, .a=Access::RW, .type=Type::E8}
                    ,cluster_mem_desc_t{.m = &T::active_odr,.id = kZB_ATTR_ID_ACTIVE_TRACKING_RATE, .a=Access::RW, .type=Type::E8}
                >{}
            >{};
        }
    };
}
#endif
