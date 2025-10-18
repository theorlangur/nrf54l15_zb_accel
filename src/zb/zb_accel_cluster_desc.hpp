#ifndef ZB_ACCEL_CLUSTER_DESC_HPP_
#define ZB_ACCEL_CLUSTER_DESC_HPP_

#include <nrfzbcpp/zb_main.hpp>

namespace zb
{
    static constexpr uint16_t kZB_ZCL_CLUSTER_ID_ACCEL = 0xfc00;
    static constexpr uint8_t kZB_ZCL_ACCEL_CMD_WAKE_UP_EVENT = 100;
    static constexpr uint8_t kZB_ZCL_ACCEL_CMD_SLEEP_EVENT = 101;
    static constexpr uint8_t kZB_ZCL_ACCEL_CMD_FLIP_EVENT = 102;
    struct EventArgs
    {
        uint32_t tap : 1;
        uint32_t dbl_tap : 1;
        uint32_t sixD_x_hi : 1;
        uint32_t sixD_x_lo : 1;
        uint32_t sixD_y_hi : 1;
        uint32_t sixD_y_lo : 1;
        uint32_t sixD_z_hi : 1;
        uint32_t sixD_z_lo : 1;
        uint32_t wake : 1;
        uint32_t sleep : 1;
        uint32_t flip_x : 1;
        uint32_t flip_y : 1;
        uint32_t flip_z : 1;
    };

    struct MeasuredAccelValues
    {
        float x;//1.f == 1G
        float y;//1.f == 1G
        float z;//1.f == 1G
    };

    struct accel_config_t
    {
        uint8_t on_event_pool_size = 0;
    };

    struct flip_event_arg_t
    {
        uint8_t flip_x : 1 = 0;
        uint8_t flip_y : 1 = 0;
        uint8_t flip_z : 1 = 0;
        uint8_t x_neg  : 1 = 0;
        uint8_t y_neg  : 1 = 0;
        uint8_t z_neg  : 1 = 0;
        uint8_t unused : 2 = 0;

        explicit operator bool() const { return flip_x || flip_y || flip_z; }
    };

    template<accel_config_t cfg = {}>
    struct zb_zcl_accel_basic_t
    {
        float x = {};//1.f == 1G
        float y = {};//1.f == 1G
        float z = {};//1.f == 1G
        [[no_unique_address]]cluster_std_cmd_desc_with_pool_size_t<kZB_ZCL_ACCEL_CMD_WAKE_UP_EVENT, cfg.on_event_pool_size, MeasuredAccelValues> on_wake_up;
        [[no_unique_address]]cluster_std_cmd_desc_with_pool_size_t<kZB_ZCL_ACCEL_CMD_SLEEP_EVENT, cfg.on_event_pool_size, MeasuredAccelValues> on_sleep;
        [[no_unique_address]]cluster_std_cmd_desc_with_pool_size_t<kZB_ZCL_ACCEL_CMD_FLIP_EVENT, cfg.on_event_pool_size, flip_event_arg_t> on_flip;
    };

    template<accel_config_t cfg> struct zcl_description_t<zb_zcl_accel_basic_t<cfg>> {
        static constexpr auto get()
        {
            using T = zb_zcl_accel_basic_t<cfg>;
            return cluster_struct_desc_t<
                cluster_info_t{.id = kZB_ZCL_CLUSTER_ID_ACCEL},
                cluster_attributes_desc_t<
                    cluster_mem_desc_t{.m = &T::x,.id = 0x0000, .a=Access::RP},
                    cluster_mem_desc_t{.m = &T::y,.id = 0x0001, .a=Access::RP},
                    cluster_mem_desc_t{.m = &T::z,.id = 0x0002, .a=Access::RP}
                >{}
                ,cluster_commands_desc_t<
                     &T::on_wake_up
                     ,&T::on_sleep
                     ,&T::on_flip
                >{}
            >{};
        }
    };
}
#endif
