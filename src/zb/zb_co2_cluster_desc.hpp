#ifndef ZB_CO2_CLUSTER_DESC_HPP_
#define ZB_CO2_CLUSTER_DESC_HPP_

#include "zb_main.hpp"

extern "C"
{
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zb_nrf_platform.h>
}

namespace zb
{
    static constexpr uint16_t kZB_ZCL_CLUSTER_ID_CO2 = 0x040D;
    struct zb_zcl_co2_basic_t
    {
        float measured_value;
    };

    template<> constexpr auto get_cluster_description<zb_zcl_co2_basic_t>()
    {
        using T = zb_zcl_co2_basic_t;
        return cluster_struct_desc_t<
            {.id = kZB_ZCL_CLUSTER_ID_CO2},
            cluster_attributes_desc_t<
                cluster_mem_desc_t{.m = &T::measured_value,.id = 0x0000, .a=Access::RP}
            >{}
        >{};
    }

    struct zb_zcl_co2_t: zb_zcl_co2_basic_t
    {
        float min_measured_value;
        float max_measured_value;
    };

    template<> constexpr auto get_cluster_description<zb_zcl_co2_t>()
    {
        using T = zb_zcl_co2_t;
        return get_cluster_description<zb_zcl_co2_basic_t>() 
            + cluster_struct_desc_t<
            {.id = kZB_ZCL_CLUSTER_ID_CO2},
            cluster_attributes_desc_t<
                cluster_mem_desc_t{.m = &T::min_measured_value,.id = 0x0001},
                cluster_mem_desc_t{.m = &T::max_measured_value,.id = 0x0002}
            >{}
        >{};
    }

    struct zb_zcl_co2_ext_t: zb_zcl_co2_t
    {
        float tolerance;
    };

    template<> constexpr auto get_cluster_description<zb_zcl_co2_ext_t>()
    {
        using T = zb_zcl_co2_ext_t;
        return get_cluster_description<zb_zcl_co2_t>() 
            + cluster_struct_desc_t<
            {.id = kZB_ZCL_CLUSTER_ID_CO2},
            cluster_attributes_desc_t<
                cluster_mem_desc_t{.m = &T::tolerance,.id = 0x0003}
            >{}
        >{};
    }

DEFINE_NULL_CLUSTER_INIT_FOR(kZB_ZCL_CLUSTER_ID_CO2);
}
#endif
