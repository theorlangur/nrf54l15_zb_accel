/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ZB_MEM_CONFIG_CUSTOM_H
#define ZB_MEM_CONFIG_CUSTOM_H 1


#define ZB_CONFIG_ROLE_ZED
#ifdef ZB_CONFIG_OVERALL_NETWORK_SIZE
#undef ZB_CONFIG_OVERALL_NETWORK_SIZE
#endif
#define ZB_CONFIG_OVERALL_NETWORK_SIZE 16

/**
 * Light routing and application traffic from/to that device.
 */
#define ZB_CONFIG_LIGHT_TRAFFIC

/**
 * Simple user's application at that device: not too many relations
 * to other devices.
 */
#define ZB_CONFIG_APPLICATION_SIMPLE

/**
 * The below section is for advanced users.
 * Before modifying any values, please make sure you fully understand the
 * impact, for example by reading zb_mem_config_common.h
 */
#include "zb_mem_config_common.h"

/**
 * Now if you REALLY know what you do, you can study zb_mem_config_common.h
 * and redefine some configuration parameters, like:
 */
//#if defined(CONFIG_LIGHT_SWITCH_CONFIGURE_TX_POWER) && \
//	defined(CONFIG_ZIGBEE_CHANNEL_SELECTION_MODE_MULTI)
//#undef ZB_CONFIG_SCHEDULER_Q_SIZE
//#define ZB_CONFIG_SCHEDULER_Q_SIZE 28
//#undef ZB_CONFIG_IOBUF_POOL_SIZE
//#define ZB_CONFIG_IOBUF_POOL_SIZE 42
//#else
//#undef ZB_CONFIG_SCHEDULER_Q_SIZE
//#define ZB_CONFIG_SCHEDULER_Q_SIZE 24
//#endif /* CONFIG_LIGHT_SWITCH_CONFIGURE_TX_POWER && CONFIG_ZIGBEE_CHANNEL_SELECTION_MODE_MULTI */

#undef ZB_CONFIG_SCHEDULER_Q_SIZE
#define ZB_CONFIG_SCHEDULER_Q_SIZE 28
#undef ZB_CONFIG_IOBUF_POOL_SIZE
#define ZB_CONFIG_IOBUF_POOL_SIZE 42

/**
 * Increase OTA transfer time by extending the APS duplicate rejection table.
 * This value must be less than the amount of possible APS counter values (256).
 */
#undef ZB_CONFIG_APS_DUPS_TABLE_SIZE
#define ZB_CONFIG_APS_DUPS_TABLE_SIZE 64

/* Memory context definitions. */
#include "zb_mem_config_context.h"

#endif /* ZB_MEM_CONFIG_CUSTOM_H */
