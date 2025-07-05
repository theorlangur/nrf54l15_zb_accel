#ifndef ZB_POLL_CTRL_CLUSTER_DESC_HPP_
#define ZB_POLL_CTRL_CLUSTER_DESC_HPP_

#include "zb_main.hpp"

extern "C"
{
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zb_nrf_platform.h>
}

namespace zb
{
    namespace literals
    {
        constexpr uint32_t operator""_sec_to_qs(unsigned long long v) { return v * 4; }
        constexpr uint32_t operator""_min_to_qs(unsigned long long v) { return v * 4 * 60; }
        constexpr uint32_t operator""_h_to_qs(unsigned long long v) { return v * 4 * 60 * 60; }
    }

    constexpr uint32_t qs_to_s(uint32_t v) { return v / 4; }

    using namespace literals;
    static constexpr uint16_t kZB_ZCL_CLUSTER_ID_POLL_CTRL = 0x0020;
    struct zb_zcl_poll_ctrl_basic_t
    {
        uint32_t check_in_interval = 1_h_to_qs;//1h
        uint32_t long_poll_interval = 5_sec_to_qs;//5sec
        uint16_t short_poll_interval = 2;//2quater-sec
        uint16_t fast_poll_timeout = 10_sec_to_qs;//10sec
        zb_zcl_poll_control_srv_cfg_data_t srv_cfg =                                         \
        { ZB_ZCL_POLL_CTRL_INVALID_ADDR, ZB_ZCL_POLL_INVALID_EP, 0, 0 };        \
    };

    struct zb_zcl_poll_ctrl_t: zb_zcl_poll_ctrl_basic_t
    {
        uint32_t check_in_interval_min = 0;
        uint32_t long_poll_interval_min = 0;
        uint16_t fast_poll_timeout_max = 0;
    };

    template<> constexpr auto get_cluster_description<zb_zcl_poll_ctrl_basic_t>()
    {
        using T = zb_zcl_poll_ctrl_basic_t;
        return cluster_struct_desc_t<
            {.id = kZB_ZCL_CLUSTER_ID_POLL_CTRL},
            cluster_attributes_desc_t<
                cluster_mem_desc_t{.m = &T::check_in_interval,  .id = 0x0000, .a=Access::RW},
                cluster_mem_desc_t{.m = &T::long_poll_interval, .id = 0x0001, .a=Access::Read},
                cluster_mem_desc_t{.m = &T::short_poll_interval,.id = 0x0002, .a=Access::Read},
                cluster_mem_desc_t{.m = &T::fast_poll_timeout,  .id = 0x0003, .a=Access::RW},
                cluster_mem_desc_t{.m = &T::srv_cfg,  .id = 0xeffeU, .a=Access::Internal, .type=Type::Null}
            >{}
        >{};
    }

    template<> constexpr auto get_cluster_description<zb_zcl_poll_ctrl_t>()
    {
        using T = zb_zcl_poll_ctrl_t;
        return get_cluster_description<zb_zcl_poll_ctrl_basic_t>() 
            + cluster_struct_desc_t<
            {.id = kZB_ZCL_CLUSTER_ID_POLL_CTRL},
            cluster_attributes_desc_t<
                cluster_mem_desc_t{.m = &T::check_in_interval_min, .id = 0x0004, .a=Access::Read},
                cluster_mem_desc_t{.m = &T::long_poll_interval_min,.id = 0x0005, .a=Access::Read},
                cluster_mem_desc_t{.m = &T::fast_poll_timeout_max, .id = 0x0006, .a=Access::Read}
            >{}
        >{};
    }

DEFINE_NULL_CLUSTER_INIT_FOR(kZB_ZCL_CLUSTER_ID_POLL_CTRL);
}
#endif
