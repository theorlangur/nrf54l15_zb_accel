#ifndef ZB_ACCEL_SETTINGS_H_
#define ZB_ACCEL_SETTINGS_H_

#include <nrfzbcpp/zb_main.hpp>

extern "C"
{
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zb_nrf_platform.h>
}

namespace zb
{
    static constexpr uint16_t kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS = 0xfc01;
    static constexpr uint16_t kZB_ATTR_ID_MAIN_SETTINGS = 0x0000;

    struct zb_zcl_accel_settings_t
    {
        union{
            struct{
                uint32_t enable_x : 1 = 1;
                uint32_t enable_y : 1 = 1;
                uint32_t enable_z : 1 = 1;
            }flags{};
            uint32_t flags_dw;
        };
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
                >{}
            >{};
        }
    };

DEFINE_NULL_CLUSTER_INIT_FOR(kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS);
}
#endif
