#ifndef ZB_ACCEL_CLUSTER_DESC_HPP_
#define ZB_ACCEL_CLUSTER_DESC_HPP_

#include "zb_main.hpp"

extern "C"
{
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zb_nrf_platform.h>
}

namespace zb
{
    static constexpr uint16_t kZB_ZCL_CLUSTER_ID_ACCEL = 0xfc00;
    struct zb_zcl_accel_basic_t
    {
        float x;//1.f == 1G
        float y;//1.f == 1G
        float z;//1.f == 1G
    };

    template<> constexpr auto get_cluster_description<zb_zcl_accel_basic_t>()
    {
        using T = zb_zcl_accel_basic_t;
        return cluster_struct_desc_t<
            cluster_info_t{.id = kZB_ZCL_CLUSTER_ID_ACCEL},
            cluster_attributes_desc_t<
                cluster_mem_desc_t{.m = &T::x,.id = 0x0000, .a=Access::RP},
                cluster_mem_desc_t{.m = &T::y,.id = 0x0001, .a=Access::RP},
                cluster_mem_desc_t{.m = &T::z,.id = 0x0002, .a=Access::RP}
            >{}
        >{};
    }

DEFINE_NULL_CLUSTER_INIT_FOR(kZB_ZCL_CLUSTER_ID_ACCEL);
}
#endif
