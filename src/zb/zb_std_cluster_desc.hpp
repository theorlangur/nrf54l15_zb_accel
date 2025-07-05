#ifndef ZB_STD_CLUSTER_DESC_HPP_
#define ZB_STD_CLUSTER_DESC_HPP_

#include "zb_main.hpp"
#include "zb_str.hpp"

extern "C"
{
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zb_nrf_platform.h>
}

namespace zb
{

struct zb_zcl_basic_min_t
{
    enum PowerSource: zb_uint8_t
    {
        Unknown               = 0x00,
        Mains1Phase           = 0x01,
        Mains3Phase           = 0x02,
        Battery               = 0x03,
        DC                    = 0x04,
        EmergencyConst        = 0x05,
        EmergencySwitch       = 0x06,
        BackupUnknown         = 0x80,
        BackupMains1Phase     = 0x81,
        BackupMains3Phase     = 0x82,
        BackupBattery         = 0x83,
        BackupDC              = 0x84,
        BackupEmergencyConst  = 0x85,
        BackupEmergencySwitch = 0x86,
    };
    zb_uint8_t zcl_version;
    PowerSource power_source;
} ;

struct zb_zcl_on_off_attrs_client_t
{
    [[no_unique_address]]cluster_std_cmd_desc_t<ZB_ZCL_CMD_ON_OFF_OFF_ID> off;
    [[no_unique_address]]cluster_std_cmd_desc_t<ZB_ZCL_CMD_ON_OFF_ON_ID> on;
    [[no_unique_address]]cluster_std_cmd_desc_t<ZB_ZCL_CMD_ON_OFF_ON_WITH_TIMED_OFF_ID, uint8_t, uint16_t, uint16_t> on_with_timed_off;
};

template<> constexpr auto get_cluster_description<zb_zcl_basic_min_t>()
{
	using T = zb_zcl_basic_min_t;
	return cluster_struct_desc_t<
		{.id = ZB_ZCL_CLUSTER_ID_BASIC},
		cluster_attributes_desc_t<
			cluster_mem_desc_t{.m = &T::zcl_version , .id = ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID },
			cluster_mem_desc_t{.m = &T::power_source, .id = ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID}
		>{}
	>{};
}

struct zb_zcl_basic_names_t: zb_zcl_basic_min_t
{
    ZigbeeStr<33> manufacturer;
    ZigbeeStr<33> model;
};

template<> constexpr auto get_cluster_description<zb_zcl_basic_names_t>()
{
	using T = zb_zcl_basic_names_t;
	return get_cluster_description<zb_zcl_basic_min_t>() + 
        cluster_struct_desc_t<
		{.id = ZB_ZCL_CLUSTER_ID_BASIC},
		cluster_attributes_desc_t<
			cluster_mem_desc_t{.m = &T::manufacturer, .id = ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID},
			cluster_mem_desc_t{.m = &T::model       , .id = ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID }
		>{}
	>{};
}

template<> constexpr auto get_cluster_description<zb_zcl_basic_attrs_ext_t>()
{
	using T = zb_zcl_basic_attrs_ext_t;
	return cluster_struct_desc_t<
		{.id = ZB_ZCL_CLUSTER_ID_BASIC},
		cluster_attributes_desc_t<
			cluster_mem_desc_t{.m = &T::zcl_version,   .id = ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID},
			cluster_mem_desc_t{.m = &T::app_version,   .id = ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID},
			cluster_mem_desc_t{.m = &T::stack_version, .id = ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID},
			cluster_mem_desc_t{.m = &T::hw_version,    .id = ZB_ZCL_ATTR_BASIC_HW_VERSION_ID},
			cluster_mem_desc_t{.m = &T::mf_name,       .id = ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,    .a = Access::Read, .type=Type::CharStr},
			cluster_mem_desc_t{.m = &T::model_id,      .id = ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,     .a = Access::Read, .type=Type::CharStr},
			cluster_mem_desc_t{.m = &T::date_code,     .id = ZB_ZCL_ATTR_BASIC_DATE_CODE_ID,            .a = Access::Read, .type=Type::CharStr},
			cluster_mem_desc_t{.m = &T::power_source,  .id = ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID,         .a = Access::Read, .type=Type::E8},
			cluster_mem_desc_t{.m = &T::location_id,   .id = ZB_ZCL_ATTR_BASIC_LOCATION_DESCRIPTION_ID, .a = Access::RW,   .type=Type::CharStr},
			cluster_mem_desc_t{.m = &T::ph_env,        .id = ZB_ZCL_ATTR_BASIC_PHYSICAL_ENVIRONMENT_ID, .a = Access::RW,   .type=Type::E8},
			cluster_mem_desc_t{.m = &T::sw_ver,        .id = ZB_ZCL_ATTR_BASIC_SW_BUILD_ID,             .a = Access::Read, .type=Type::CharStr}
		>{}
	>{};
}

template<> constexpr auto get_cluster_description<zb_zcl_identify_attrs_t>()
{
	using T = zb_zcl_identify_attrs_t;
	return cluster_struct_desc_t<
		{.id = ZB_ZCL_CLUSTER_ID_IDENTIFY},
		cluster_attributes_desc_t<
			cluster_mem_desc_t{.m = &T::identify_time, .id = ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, .a = Access::RW}
		>{}
	>{};
}

template<> constexpr auto get_cluster_description<zb_zcl_on_off_attrs_t>()
{
	using T = zb_zcl_on_off_attrs_t;
	return cluster_struct_desc_t<
		{.id = ZB_ZCL_CLUSTER_ID_ON_OFF},
		cluster_attributes_desc_t<
			cluster_mem_desc_t{.m = &T::on_off, .id = ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, .a = Access::RPS, .type=Type::Bool}
		>{}
	>{};
}

template<> constexpr auto get_cluster_description<zb_zcl_on_off_attrs_client_t>()
{
	return cluster_struct_desc_t<
		{.id = ZB_ZCL_CLUSTER_ID_ON_OFF, .role = Role::Client},
		cluster_attributes_desc_t<>{},
		cluster_commands_desc_t<
		     &zb_zcl_on_off_attrs_client_t::on
		    ,&zb_zcl_on_off_attrs_client_t::off
		    ,&zb_zcl_on_off_attrs_client_t::on_with_timed_off
		>{}
	>{};
}

template<> constexpr auto get_cluster_description<zb_zcl_groups_attrs_t>()
{
	using T = zb_zcl_groups_attrs_t;
	return cluster_struct_desc_t<
		{.id = ZB_ZCL_CLUSTER_ID_GROUPS},
		cluster_attributes_desc_t<
			cluster_mem_desc_t{.m = &T::name_support, .id = ZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, .a = Access::Read, .type=Type::Map8}
		>{}
	>{};
}

template<> constexpr auto get_cluster_description<zb_zcl_scenes_attrs_t>()
{
	using T = zb_zcl_scenes_attrs_t;
	return cluster_struct_desc_t<
		{.id = ZB_ZCL_CLUSTER_ID_SCENES},
		cluster_attributes_desc_t<
			  cluster_mem_desc_t{.m = &T::scene_count,   .id = ZB_ZCL_ATTR_SCENES_SCENE_COUNT_ID,   .a = Access::Read}
			, cluster_mem_desc_t{.m = &T::current_scene, .id = ZB_ZCL_ATTR_SCENES_CURRENT_SCENE_ID, .a = Access::Read}
			, cluster_mem_desc_t{.m = &T::scene_valid,   .id = ZB_ZCL_ATTR_SCENES_SCENE_VALID_ID,   .a = Access::Read, .type=Type::Bool}
			, cluster_mem_desc_t{.m = &T::name_support,  .id = ZB_ZCL_ATTR_SCENES_NAME_SUPPORT_ID,  .a = Access::Read, .type=Type::Map8}
			, cluster_mem_desc_t{.m = &T::current_group, .id = ZB_ZCL_ATTR_SCENES_CURRENT_GROUP_ID, .a = Access::Read}
		>{}
	>{};
}

template<> constexpr auto get_cluster_description<zb_zcl_level_control_attrs_t>()
{
	using T = zb_zcl_level_control_attrs_t;
	return cluster_struct_desc_t<
		{.id = ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL},
		cluster_attributes_desc_t<
			  cluster_mem_desc_t{.m = &T::current_level, .id = ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID, .a = Access::RPS}
			, cluster_mem_desc_t{.m = &T::remaining_time, .id = ZB_ZCL_ATTR_LEVEL_CONTROL_REMAINING_TIME_ID, .a = Access::Read}
		>{}
	>{};
}

DEFINE_GET_CLUSTER_INIT_FOR(ZB_ZCL_CLUSTER_ID_BASIC);
DEFINE_GET_CLUSTER_INIT_FOR(ZB_ZCL_CLUSTER_ID_IDENTIFY);
DEFINE_GET_CLUSTER_INIT_FOR(ZB_ZCL_CLUSTER_ID_ON_OFF);
DEFINE_GET_CLUSTER_INIT_FOR(ZB_ZCL_CLUSTER_ID_GROUPS);
DEFINE_GET_CLUSTER_INIT_FOR(ZB_ZCL_CLUSTER_ID_SCENES);
DEFINE_GET_CLUSTER_INIT_FOR(ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL);
}

#endif
