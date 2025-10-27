/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/adc.h>

#include <ram_pwrdn.h>

//#define ALARM_LIST_LOCK_TYPE thread::DummyLock
#include <nrfzbcpp/zb_main.hpp>
#include <nrfzbcpp/zb_std_cluster_desc.hpp>
#include <nrfzbcpp/zb_power_config_tools.hpp>
#include <nrfzbcpp/zb_poll_ctrl_tools.hpp>

#include <nrfzbcpp/zb_status_cluster_desc.hpp>

#include <nrfzbcpp/zb_alarm.hpp>
#include <nrfzbcpp/zb_settings.hpp>

#include "zb/zb_accel_settings.hpp"
#include "zb/zb_accel_cluster_desc.hpp"

#include <zephyr/drivers/sensor/lis2du12.h>
#include <dk_buttons_and_leds.h>
#include "led.h"

//allows using _min_to_qs and similar stuff
using namespace zb::literals;

/**********************************************************************/
/* Configuration constants                                            */
/**********************************************************************/
constexpr bool kPowerSaving = true;//if this is change the MCU must be erased since that option persists otherwise
constexpr uint32_t kFactoryResetWaitMS = 5000;//5s if the dev doesn't join before that
constexpr int8_t kRestartCountToFactoryReset = 3;
constexpr uint32_t kRestartCounterResetTimeoutMS = 15000;//after 15s the restart counter is reset back to 3
constexpr uint32_t kKeepAliveTimeout = 1000*60*30;//30min

constexpr auto kInitialCheckInInterval = 2_min_to_qs;
constexpr auto kInitialLongPollInterval = 60_min_to_qs;//this has to be big in order for the device not to perform permanent parent requests

/**********************************************************************/
/* Zigbee Declarations and Definitions                                */
/**********************************************************************/
static bool g_ZigbeeReady = false;

/* Manufacturer name (32 bytes). */
#define INIT_BASIC_MANUF_NAME      "SFINAE"

/* Model number assigned by manufacturer (32-bytes long string). */
#define INIT_BASIC_MODEL_ID        "Accel-NG"


/* Button used to enter the Bulb into the Identify mode. */
#define IDENTIFY_MODE_BUTTON            DK_BTN1_MSK

/* Button to start Factory Reset */
#define FACTORY_RESET_BUTTON IDENTIFY_MODE_BUTTON

/* Device endpoint, used to receive light controlling commands. */
constexpr uint8_t kACCEL_EP = 1;
constexpr uint16_t kDEV_ID = 0xDEAD;

/* Main application customizable context.
 * Stores all settings and static values.
 */
struct device_ctx_t{
    using accel_type = zb::zb_zcl_accel_basic_t<{.on_event_pool_size = 3}>;

    zb::zb_zcl_basic_names_t basic_attr;
    zb::zb_zcl_power_cfg_battery_info_t battery_attr;
    zb::zb_zcl_poll_ctrl_basic_t poll_ctrl;
    accel_type accel_attr;
    zb::zb_zcl_status_t status_attr;
    zb::zb_zcl_accel_settings_t settings;
};

//attribute shortcuts for template arguments
constexpr auto kAttrX = &device_ctx_t::accel_type::x;
constexpr auto kAttrY = &device_ctx_t::accel_type::y;
constexpr auto kAttrZ = &device_ctx_t::accel_type::z;
constexpr auto kCmdOnWakeUpEvent = &device_ctx_t::accel_type::on_wake_up;
constexpr auto kCmdOnSleepEvent = &device_ctx_t::accel_type::on_sleep;
constexpr auto kCmdOnFlipEvent = &device_ctx_t::accel_type::on_flip;
constexpr auto kAttrStatus1 = &zb::zb_zcl_status_t::status1;
constexpr auto kAttrStatus2 = &zb::zb_zcl_status_t::status2;
constexpr auto kAttrStatus3 = &zb::zb_zcl_status_t::status3;

constexpr int kSleepSendStatusBit1 = 0;
constexpr int kSleepSendStatusCodeOffset = 1;
constexpr int kFlipSendStatusBit1 = 5;
constexpr int kFlipSendStatusCodeOffset = 6;
constexpr int kWakeUpSendStatusBit1 = 10;
constexpr int kWakeUpSendStatusCodeOffset = 11;

constexpr int kStatusCodeSize = 4;

/* Zigbee device application context storage. */
static constinit device_ctx_t dev_ctx{
    .basic_attr = {
	{
	    .zcl_version = ZB_ZCL_VERSION,
	    .power_source = zb::zb_zcl_basic_min_t::PowerSource::Battery
	},
	/*.manufacturer =*/ INIT_BASIC_MANUF_NAME,
	/*.model =*/ INIT_BASIC_MODEL_ID,
    },
	.poll_ctrl = {
	    .check_in_interval = kInitialCheckInInterval,
	    .long_poll_interval = kInitialLongPollInterval,
	    //.short_poll_interval = 1_sec_to_qs,
	},
	.accel_attr = {
	    .x = 0,
	    .y = 0,
	    .z = 0,
	}
};

//forward declare
template<> struct zb::cluster_custom_handler_t<device_ctx_t::accel_type, kACCEL_EP>;
using custom_accel_handler_t = zb::cluster_custom_handler_t<device_ctx_t::accel_type, kACCEL_EP>;

constinit static auto zb_ctx = zb::make_device(
	zb::make_ep_args<{.ep=kACCEL_EP, .dev_id=kDEV_ID, .dev_ver=1}>(
	    dev_ctx.basic_attr
	    , dev_ctx.battery_attr
	    , dev_ctx.poll_ctrl
	    , dev_ctx.accel_attr
	    , dev_ctx.status_attr
	    , dev_ctx.settings
	    )
	);

//a shortcut for a convenient access
constinit static auto &zb_ep = zb_ctx.ep<kACCEL_EP>();

//magic handwaving to avoid otherwise necessary command handling boilerplate
//uses CRTP so that cluster_custom_handler_base_t would know the end type it needs to work with
template<> 
struct zb::cluster_custom_handler_t<device_ctx_t::accel_type, kACCEL_EP>: cluster_custom_handler_base_t<custom_accel_handler_t>
{
    //the rest will be done by cluster_custom_handler_base_t
    static auto& get_device() { return zb_ctx; }
};


/**********************************************************************/
/* Persisten settings                                                 */
/**********************************************************************/

#define SETTINGS_ZB_ACCEL_SUBTREE "zb_accel"
struct ZbSettingsEntries
{
    //had to define the strings like that or otherwise passing 'const char*' as a template parameter at a compile time doesn't work
    //it needs to have an extrenal linking
    inline static constexpr const char flags[] = SETTINGS_ZB_ACCEL_SUBTREE "/accel_flags";
    inline static constexpr const char wake_sleep_threshold[] = SETTINGS_ZB_ACCEL_SUBTREE "/wake_sleep_threshold";
    inline static constexpr const char sleep_duration[] = SETTINGS_ZB_ACCEL_SUBTREE "/sleep_duration";
    inline static constexpr const char sleep_tracking_rate[] = SETTINGS_ZB_ACCEL_SUBTREE "/sleep_tracking_rate";
    inline static constexpr const char active_tracking_rate[] = SETTINGS_ZB_ACCEL_SUBTREE "/active_tracking_rate";
};

using settings_mgr = zb::persistent_settings_manager<
    sizeof(SETTINGS_ZB_ACCEL_SUBTREE)
    ,zb::settings_entry{ZbSettingsEntries::flags, dev_ctx.settings.flags_dw}
    ,zb::settings_entry{ZbSettingsEntries::wake_sleep_threshold, dev_ctx.settings.wake_sleep_threshold}
    ,zb::settings_entry{ZbSettingsEntries::sleep_duration, dev_ctx.settings.sleep_duration}
    ,zb::settings_entry{ZbSettingsEntries::sleep_tracking_rate, dev_ctx.settings.sleep_odr}
    ,zb::settings_entry{ZbSettingsEntries::active_tracking_rate, dev_ctx.settings.active_odr}
>;

//helping constexpr template functions to wrap the change reaction logic into settings-storing logic
template<auto h>
constexpr zb::set_attr_value_handler_t to_settings_handler(const char *name)
{
    return settings_mgr::make_on_changed<zb::to_handler_v<h>>(name);
}

constexpr zb::set_attr_value_handler_t to_settings_handler(const char *name)
{
    return settings_mgr::make_on_changed<nullptr>(name);
}

/**********************************************************************/
/* End of settings section                                            */
/**********************************************************************/

/**********************************************************************/
/* ZephyrOS devices                                                   */
/**********************************************************************/

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct device *const accel_dev = DEVICE_DT_GET(DT_NODELABEL(accel));
static const struct gpio_dt_spec led_dt = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


/**********************************************************************/
/* Battery management                                                 */
/**********************************************************************/
#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

static constinit auto g_Battery = zb::make_battery_measurements_block<{
    .maxBatteryVoltage = 1600,//mV
    .minBatteryVoltage = 900//mV
}>(zb_ep, adc_channels);
constexpr zb_callback_t update_battery_state_zb = &decltype(g_Battery)::update_zb_callback;

/**********************************************************************/
/* FloodGate tool                                                     */
/**********************************************************************/
//there may be no more than 1 request in process
//the rest will be registered as a re-trigger requests and will be processed (re-triggered)
//at the end
template<zb_callback_t cb>
struct FloodGate
{
    bool RequestProcessing()
    {
	if (!m_ZbPosted.exchange(true))
	{
	    m_ExpectedCount = 1;
	    zb_schedule_app_alarm(cb, 0, 0);
	    return true;
	}else
	    m_ReTriggerRequest = true;
	return false;
    }

    bool ProcessingDone()
    {
	if (--m_ExpectedCount)
	    return false;

	m_ZbPosted = false;
	if (m_ReTriggerRequest.exchange(false))
	    return RequestProcessing();
	return false;
    }

    struct FinishAtEnd
    {
	FinishAtEnd(FloodGate &fg):m_Gate(fg){}
	~FinishAtEnd() { m_Gate.ProcessingDone(); }

	FinishAtEnd(FinishAtEnd const&rhs) = delete;
	FinishAtEnd(FinishAtEnd &&rhs) = delete;
	FinishAtEnd& operator=(FinishAtEnd const&rhs) = delete;
	FinishAtEnd& operator=(FinishAtEnd &rhs) = delete;

	void WaitForOneMore(){
	    ++m_Gate.m_ExpectedCount;
	}

	FloodGate &m_Gate;
    };

    FinishAtEnd GetFinishingBlock()
    {
	return {*this};
    }

    thread::SyncVar<bool> m_ZbPosted{false};
    thread::SyncVar<bool> m_ReTriggerRequest{false};
    int m_ExpectedCount = 0;
};

/**********************************************************************/
/* Accelerometer stuff                                                */
/**********************************************************************/

//forward declaration to get it into a template arg
void on_sleep_zb(uint8_t buf);
void on_wake_up_zb(uint8_t buf);

static constinit FloodGate<on_sleep_zb> g_SleepGate{};
static constinit FloodGate<on_wake_up_zb> g_WakeUpGate{};

struct accel_val
{
    sensor_value x;
    sensor_value y;
    sensor_value z;
};

struct FlipTracker
{
    static constexpr float kFlipThreshold = 0.7f;

    float m_LastMeasuredX = std::numeric_limits<float>::quiet_NaN();
    float m_LastMeasuredY = std::numeric_limits<float>::quiet_NaN();
    float m_LastMeasuredZ = std::numeric_limits<float>::quiet_NaN();

    zb::flip_event_arg_t CheckFlip(accel_val const& newVals)
    {
	zb::flip_event_arg_t res{};

	auto TryUpdateMeasurement = [&](sensor_value newSensorX, float &last)
	{
	    float newX = float(newSensorX.val1) + float(newSensorX.val2) / 1000'000.f;
	    if (std::isnan(last))
		last = newX;
	    else if (((last > 0) == (newX > 0)) && (std::abs(newX) > std::abs(last)))
		last = newX;
	    else if ((last > 0) != (newX > 0))
	    {
		if (std::abs(newX - last) > kFlipThreshold)
		{
		    //flip happened
		    last = newX;
		    return true;
		}
	    }
	    return false;
	};

	if (dev_ctx.settings.flags.track_flip)
	{
	    if (dev_ctx.settings.flags.enable_x)
	    {
		res.flip_x = TryUpdateMeasurement(newVals.x, m_LastMeasuredX);
		res.x_neg = m_LastMeasuredX < 0;
	    }
	    if (dev_ctx.settings.flags.enable_y)
	    {
		res.flip_y = TryUpdateMeasurement(newVals.y, m_LastMeasuredY);
		res.y_neg = m_LastMeasuredY < 0;
	    }
	    if (dev_ctx.settings.flags.enable_z)
	    {
		res.flip_z = TryUpdateMeasurement(newVals.z, m_LastMeasuredZ);
		res.z_neg = m_LastMeasuredZ < 0;
	    }
	}
	return res;
    }
};

FlipTracker g_Flip;

/**********************************************************************/
/* Error handling tools                                               */
/**********************************************************************/
constexpr const int16_t kErrorFailedToSend = 0x0e;
constexpr const int16_t kErrorUnknown = 0x0f;
int16_t status_to_error_code(zb_zcl_command_send_status_t *status)
{
    int16_t code = 0;
    if (!status)
	code = 0x0e;
    else if (status->status < 0) 
    {
	if (status->status >= -8)
	    code = -status->status;
	else
	    code = 0x0f;
    }
    return code;
}

/**********************************************************************/
/* ZephyrOS triggers                                                  */
/**********************************************************************/
static lis2du12_trigger g_WakeUpTrigger{ 
    .trig={.type = (enum sensor_trigger_type)LIS2DU12_TRIG_WAKE_UP, .chan = (enum sensor_channel)LIS2DU12_CHAN_ACCEL_XYZ_EXT}, 
    .wake_cfg = { .x_enable=0, .y_enable=0, .z_enable = 1, .wake_threshold = 1, .wake_duration = 1}
};

static lis2du12_trigger g_SleepTrigger{ 
    .trig={.type = (enum sensor_trigger_type)LIS2DU12_TRIG_SLEEP_CHANGE, .chan = (enum sensor_channel)LIS2DU12_CHAN_ACCEL_XYZ_EXT}, 
    .wake_cfg = { .x_enable=0, .y_enable=0, .z_enable = 1, .sleep_on = 1, .sleep_duration = 15, .wake_threshold = 1, .wake_duration = 0}
};

void on_sleep_cmd_sent(zb::cmd_id_t cmd_id, zb_zcl_command_send_status_t *status)
{
    auto processingDone = g_SleepGate.GetFinishingBlock();
    int16_t code = status_to_error_code(status);
    printk("zb: on_sleep_cmd_sent id:%d; status: %d\r\n", cmd_id, -code);

    int16_t newStatus3 = dev_ctx.status_attr.status3;
    constexpr int16_t kMask = (int16_t(1) << kStatusCodeSize) - 1;
    if (code != 0)
	newStatus3 = (newStatus3 & (~(kMask << kSleepSendStatusCodeOffset))) | (code << kSleepSendStatusCodeOffset);
    else
	newStatus3 &= ~(kMask << kSleepSendStatusCodeOffset);

    if (newStatus3 != dev_ctx.status_attr.status3)
	zb_ep.attr<kAttrStatus3>() = newStatus3;

    printk("cmd internals stats after sleep sent\r\n");
    zb_ep.dump_info<kCmdOnFlipEvent, kCmdOnSleepEvent, kCmdOnWakeUpEvent>();
}

void on_flip_cmd_sent(zb::cmd_id_t cmd_id, zb_zcl_command_send_status_t *status)
{
    auto processingDone = g_SleepGate.GetFinishingBlock();
    int16_t code = status_to_error_code(status);
    printk("zb: on_flip_cmd_sent id:%d; status: %d\r\n", cmd_id, -code);

    int16_t newStatus3 = dev_ctx.status_attr.status3;
    constexpr int16_t kMask = (int16_t(1) << kStatusCodeSize) - 1;
    if (code != 0)
	newStatus3 = (newStatus3 & (~(kMask << kFlipSendStatusCodeOffset))) | (code << kFlipSendStatusCodeOffset);
    else
	newStatus3 &= ~(kMask << kFlipSendStatusCodeOffset);

    if (newStatus3 != dev_ctx.status_attr.status3)
	zb_ep.attr<kAttrStatus3>() = newStatus3;

    printk("cmd internals stats after flip sent\r\n");
    zb_ep.dump_info<kCmdOnFlipEvent, kCmdOnSleepEvent, kCmdOnWakeUpEvent>();
}

void on_wake_up_cmd_sent(zb::cmd_id_t cmd_id, zb_zcl_command_send_status_t *status)
{
    auto processingDone = g_WakeUpGate.GetFinishingBlock();
    int16_t code = status_to_error_code(status);
    printk("zb: on_wake_up_cmd_sent id:%d; status: %d\r\n", cmd_id, -code);

    int16_t newStatus3 = dev_ctx.status_attr.status3;
    constexpr int16_t kMask = (int16_t(1) << kStatusCodeSize) - 1;
    if (code != 0)
	newStatus3 = (newStatus3 & (~(kMask << kWakeUpSendStatusCodeOffset))) | (code << kWakeUpSendStatusCodeOffset);
    else
	newStatus3 &= ~(kMask << kWakeUpSendStatusCodeOffset);

    if (newStatus3 != dev_ctx.status_attr.status3)
	zb_ep.attr<kAttrStatus3>() = newStatus3;
}

void on_sleep_zb(uint8_t buf)
{
    auto processingDone = g_SleepGate.GetFinishingBlock();
    printk("zb: sleep detected\r\n");
    if (!g_ZigbeeReady)
    {
	printk("zb: zigbee stack not ready yet\r\n");
	return;
    }
    sensor_sample_fetch(accel_dev);
    accel_val acc;
    sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, &acc.x);
    float x, y, z;
    x = float(acc.x.val1) + float(acc.x.val2) / 1000'000.f;
    y = float(acc.y.val1) + float(acc.y.val2) / 1000'000.f;
    z = float(acc.z.val1) + float(acc.z.val2) / 1000'000.f;

    zb::MeasuredAccelValues vals;
    vals.x = x;
    vals.y = y;
    vals.z = z;

    zb_ep.attr<kAttrX>() = x;
    zb_ep.attr<kAttrY>() = y;
    zb_ep.attr<kAttrZ>() = z;

    printk("cmd internals stats on_sleep_zb begin\r\n");
    zb_ep.dump_info<kCmdOnFlipEvent, kCmdOnSleepEvent, kCmdOnWakeUpEvent>();

    int16_t newStatus3 = dev_ctx.status_attr.status3;
    if (dev_ctx.settings.flags.track_sleep)
    {
	auto id = zb_ep.send_cmd<kCmdOnSleepEvent, {.cb=on_sleep_cmd_sent, .timeout_ms = 5000}>(vals);
	bool failed_to_send = !id;
	if (((newStatus3 & kSleepSendStatusBit1) != 0) != failed_to_send)
	{
	    constexpr int16_t kMask = int16_t(1) << kSleepSendStatusBit1;
	    newStatus3 = (newStatus3 & ~kMask) | (failed_to_send * kMask);
	}
	if (!failed_to_send)
	    processingDone.WaitForOneMore();
    }
    if (dev_ctx.settings.flags.track_flip)
    {
	auto flipRes = g_Flip.CheckFlip(acc);
	if (flipRes)
	{
	    auto id = zb_ep.send_cmd<kCmdOnFlipEvent, {.cb=on_flip_cmd_sent, .timeout_ms = 5000}>(flipRes);
	    bool failed_to_send = !id;
	    if (((newStatus3 & kFlipSendStatusBit1) != 0) != failed_to_send)
	    {
		constexpr int16_t kMask = int16_t(1) << kFlipSendStatusBit1;
		newStatus3 = (newStatus3 & ~kMask) | (failed_to_send * kMask);
	    }
	    if (!failed_to_send)
	    {
		processingDone.WaitForOneMore();
		zb_ep.attr<kAttrStatus2>() = *id;
	    }
	}
    }

    if (newStatus3 != dev_ctx.status_attr.status3)
	zb_ep.attr<kAttrStatus3>() = newStatus3;

    printk("cmd internals stats on_sleep_zb end\r\n");
    zb_ep.dump_info<kCmdOnFlipEvent, kCmdOnSleepEvent, kCmdOnWakeUpEvent>();
}

void on_sleep(const struct device *dev, const struct sensor_trigger *trigger)
{
    //post on Zigbee thread and run immedieately
    if (g_SleepGate.RequestProcessing())
	printk("sleep detected -> zb\r\n");
    else
	printk("sleep detected -> dropped\r\n");
}

void on_wake_up_zb(uint8_t buf)
{
    auto processingDone = g_WakeUpGate.GetFinishingBlock();
    printk("zb: wake up detected\r\n");
    if (!g_ZigbeeReady)
    {
	printk("zb: zigbee stack not ready yet\r\n");
	return;
    }
    sensor_sample_fetch(accel_dev);
    accel_val acc;
    sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, &acc.x);
    float x, y, z;
    x = float(acc.x.val1) + float(acc.x.val2) / 1000'000.f;
    y = float(acc.y.val1) + float(acc.y.val2) / 1000'000.f;
    z = float(acc.z.val1) + float(acc.z.val2) / 1000'000.f;

    zb::MeasuredAccelValues vals;
    vals.x = x;
    vals.y = y;
    vals.z = z;

    zb_ep.attr<kAttrX>() = x;
    zb_ep.attr<kAttrY>() = y;
    zb_ep.attr<kAttrZ>() = z;

    int16_t newStatus3 = dev_ctx.status_attr.status3;

    {
	auto id = zb_ep.send_cmd<kCmdOnWakeUpEvent, {.cb=on_wake_up_cmd_sent, .timeout_ms = 5000}>(vals);
	bool failed_to_send = !id;
	if (((newStatus3 & kWakeUpSendStatusBit1) != 0) != failed_to_send)
	{
	    constexpr int16_t kMask = int16_t(1) << kWakeUpSendStatusBit1;
	    newStatus3 = (newStatus3 & ~kMask) | (failed_to_send * kMask);
	}

	if (!failed_to_send)//we've sent cmd. finishing in the cmd handler
	{
	    processingDone.WaitForOneMore();
	    zb_ep.attr<kAttrStatus2>() = *id;
	}
    }

    if (newStatus3 != dev_ctx.status_attr.status3)
	zb_ep.attr<kAttrStatus3>() = newStatus3;
}

void on_wake_up(const struct device *dev, const struct sensor_trigger *trigger)
{
    //post on Zigbee thread and run immedieately
    if (g_WakeUpGate.RequestProcessing())
	printk("wake up detected -> zb\r\n");
    else
	printk("wake up detected -> postponed\r\n");
}

void reconfigure_interrupts()
{
    //return;
    int ret;
    bool xyz_enabled = dev_ctx.settings.flags.enable_x || dev_ctx.settings.flags.enable_y || dev_ctx.settings.flags.enable_z;
    if (xyz_enabled && (dev_ctx.settings.flags.track_sleep || dev_ctx.settings.flags.track_flip))
    {
	g_SleepTrigger.wake_cfg.x_enable = dev_ctx.settings.flags.enable_x;
	g_SleepTrigger.wake_cfg.y_enable = dev_ctx.settings.flags.enable_y;
	g_SleepTrigger.wake_cfg.z_enable = dev_ctx.settings.flags.enable_z;
	g_SleepTrigger.wake_cfg.sleep_duration = dev_ctx.settings.sleep_duration;
	g_SleepTrigger.wake_cfg.wake_threshold = dev_ctx.settings.wake_sleep_threshold;
	g_SleepTrigger.wake_cfg.inactive_odr = (lis2du12_sleep_odr_t)dev_ctx.settings.sleep_odr;

	ret = sensor_trigger_set(accel_dev, &g_SleepTrigger.trig, &on_sleep);
	if (ret != 0) printk("Failed to set sleep trigger\r\n");
	else printk("Enabled sleep interrupts\r\n");
    }else
    {
	ret = sensor_trigger_set(accel_dev, &g_SleepTrigger.trig, nullptr);
	if (ret != 0) printk("Failed to remove sleep trigger");
	else printk("Disabled wake interrupts\r\n");
    }

    if (xyz_enabled && dev_ctx.settings.flags.track_wake_up)
    {
	g_WakeUpTrigger.wake_cfg.x_enable = dev_ctx.settings.flags.enable_x;
	g_WakeUpTrigger.wake_cfg.y_enable = dev_ctx.settings.flags.enable_y;
	g_WakeUpTrigger.wake_cfg.z_enable = dev_ctx.settings.flags.enable_z;
	g_WakeUpTrigger.wake_cfg.wake_threshold = dev_ctx.settings.wake_sleep_threshold;

	ret = sensor_trigger_set(accel_dev, &g_WakeUpTrigger.trig, &on_wake_up);
	if (ret != 0) printk("Failed to set wake up trigger\r\n");
	else printk("Enabled wake up interrupts\r\n");
    }else
    {
	ret = sensor_trigger_set(accel_dev, &g_WakeUpTrigger.trig, nullptr);
	if (ret != 0) printk("Failed to remove wake up trigger\r\n");
	else printk("Disabled wake up interrupts\r\n");
    }
}

void udpate_accel_values(uint8_t)
{
    printk("udpate_accel_values\r\n");
    if (device_is_ready(accel_dev))
    {
	sensor_sample_fetch(accel_dev);
	accel_val acc;
	sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, &acc.x);
	zb_ep.attr<kAttrX>() = float(acc.x.val1) + float(acc.x.val2) / 1000'000.f;
	zb_ep.attr<kAttrY>() = float(acc.y.val1) + float(acc.y.val2) / 1000'000.f;
	zb_ep.attr<kAttrZ>() = float(acc.z.val1) + float(acc.z.val2) / 1000'000.f;
	zb_ep.attr<kAttrStatus1>() = 0;
	g_Flip.CheckFlip(acc);//providing initial values
	printk("Accel X: %d; Y: %d; Z: %d;\r\n", acc.x.val1, acc.y.val1, acc.z.val1);
    }else
    {
	zb_ep.attr<kAttrStatus1>() = -1;
    }
}

void on_settings_changed(const uint32_t &v)
{
    printk("Settings. Now: %X; Stored: %X\r\n", v, dev_ctx.settings.flags_dw);
    reconfigure_interrupts();
}

void on_active_odr_changed(const uint8_t &v)
{
    printk("Changed active ODR to %X;\r\n", v);
    lis2du12_set_odr(accel_dev, (lis2du12_odr_t)v);
}

void on_wake_sleep_settings_changed()
{
    printk("Settings changed\r\n");
    reconfigure_interrupts();
}

zb::ZbAlarm g_FactoryResetOnNoJoin;
static bool factory_reset_start = false;
void factory_reset_settings()
{
    //blink with led
    led::show_pattern(led::kPATTERN_3_BLIPS_NORMED, 1000); 

    //resetting accelerotmeter settings to defaults
    dev_ctx.battery_attr = {};
    dev_ctx.poll_ctrl = {
	.check_in_interval = kInitialCheckInInterval,
	.long_poll_interval = kInitialLongPollInterval,
    };
    dev_ctx.accel_attr = {};
    dev_ctx.status_attr = {};
    dev_ctx.settings = {};
    settings_save_subtree(SETTINGS_ZB_ACCEL_SUBTREE);
    zigbee_pibcache_pan_id_clear();
}

void do_factory_reset(void*)
{
    factory_reset_start = false;
    g_FactoryResetOnNoJoin.Cancel();
    //factroy reset request is active
    zb_bdb_reset_via_local_action(0);
    factory_reset_settings();
}

zb::ZbAlarm g_EnterLowPowerLongPollMode;

void on_check_in(uint8_t param)
{
    printk("on_check_in\r\n");
    update_battery_state_zb(param);
}

void on_zigbee_start()
{
    printk("on_zigbee_start\r\n");
    g_ZigbeeReady = true;
    if (factory_reset_start)
    {
	do_factory_reset(nullptr);
	return;
    }

    configure_poll_control<{
	.ep = kACCEL_EP, 
	.callback_on_check_in = on_check_in, 
	.sleepy_end_device = kPowerSaving
    }>(dev_ctx.poll_ctrl);

    //should be there already, initial state
    udpate_accel_values(0);
    g_Battery.update();
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
        zb_zdo_app_signal_hdr_t *pHdr;
        auto signalId = zb_get_app_signal(bufid, &pHdr);
        zb_ret_t status = zb_buf_get_status(bufid);
	if (factory_reset_start && !g_FactoryResetOnNoJoin.IsRunning())
	    g_FactoryResetOnNoJoin.Setup(do_factory_reset, nullptr, kFactoryResetWaitMS);

	auto ret = zb::tpl_signal_handler<zb::sig_handlers_t{
	.on_leave = +[]{ 
	    zb_zcl_poll_control_stop(); 
	    k_sleep(K_MSEC(2100));
	    sys_reboot(SYS_REBOOT_COLD);
	},
	    //.on_error = []{ led::show_pattern(led::kPATTERN_3_BLIPS_NORMED, 1000); },
	    .on_dev_reboot = on_zigbee_start,
	    .on_steering = on_zigbee_start,
	    .on_can_sleep = &zb_sleep_now
	   }>(bufid);
    const uint32_t LOCAL_ERR_CODE = (uint32_t) (-ret);	
    if (LOCAL_ERR_CODE != RET_OK) {				
	zb_osif_abort();				
    }							
}

void on_dev_cb_error(int err)
{
    printk("on_dev_cb_error: %d\r\n", err);
}

settings_handler settings_zb_accel = { 
			      .name = SETTINGS_ZB_ACCEL_SUBTREE,
                              .h_set = settings_mgr::zigbee_settings_set,
                              .h_export = settings_mgr::zigbee_settings_export
};

int8_t g_RestartsToFactoryResetLeft = kRestartCountToFactoryReset;
#define SETTINGS_DEV_SUBTREE "dev"
#define SETTINGS_DEV_RESTARTS_LEFT "restarts_left"
static int device_settings_set(const char *name, size_t len,
	settings_read_cb read_cb, void *cb_arg)
{
    int rc;
    bool found = false;
    const char *next;
    if (settings_name_steq(name, SETTINGS_DEV_RESTARTS_LEFT, &next) && !next) {
	if (len != sizeof(g_RestartsToFactoryResetLeft))
	    return -EINVAL;

	rc = read_cb(cb_arg, &g_RestartsToFactoryResetLeft, sizeof(g_RestartsToFactoryResetLeft));
	if (rc >= 0)
	{
	    if (rc != sizeof(g_RestartsToFactoryResetLeft))
		return -EINVAL;
	}
	return 0;
    }

    return -ENOENT;
}

static int device_settings_export(int (*cb)(const char *name,
	    const void *value, size_t val_len))
{
    return cb(SETTINGS_DEV_SUBTREE "/" SETTINGS_DEV_RESTARTS_LEFT, &g_RestartsToFactoryResetLeft, sizeof(g_RestartsToFactoryResetLeft));
}

settings_handler settings_dev = { 
			      .name = "dev",
                              .h_set = device_settings_set,
                              .h_export = device_settings_export
};
zb::ZbAlarm g_ResetRestartsLeftAlarm;

int main(void)
{
    int ret;
    bool led_state = true;

    printk("Main start\r\n");
    if (led::setup() < 0)
	return 0;
    led::start();

    if (device_is_ready(accel_dev))
    {
	if (dev_ctx.settings.active_odr == 0)
	    dev_ctx.settings.active_odr = lis2du12_get_odr(accel_dev);
	else
	{
	    if (lis2du12_set_odr(accel_dev, (lis2du12_odr_t)dev_ctx.settings.active_odr) < 0)
		dev_ctx.status_attr.status2 = -1;
	}
	reconfigure_interrupts();
    }else
    {
	printk("Accelerometer is not ready\r\n");
	dev_ctx.status_attr.status1 = -1;
    }

    printk("Main: before configuring ADC\r\n");
    if (g_Battery.setup() < 0)
	return 0;

    printk("Main: before settings init\r\n");
    int err = settings_subsys_init();
    settings_register(&settings_zb_accel);
    settings_register(&settings_dev);

    err = settings_load();
    --g_RestartsToFactoryResetLeft;
    factory_reset_start = g_RestartsToFactoryResetLeft == 0;
    if (factory_reset_start)
	led::show_pattern(led::kPATTERN_2_BLIPS_NORMED, 1000); 

    bool factory_reset_finish = g_RestartsToFactoryResetLeft < 0;
    if (factory_reset_finish)
	g_RestartsToFactoryResetLeft = kRestartCountToFactoryReset;
    settings_save_subtree(SETTINGS_DEV_SUBTREE);

    printk("Main: before zigbee erase persistent storage\r\n");
    zigbee_erase_persistent_storage(factory_reset_finish);
    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(kKeepAliveTimeout));

    if constexpr (kPowerSaving)
    {
	zigbee_configure_sleepy_behavior(true);
	power_down_unused_ram();
    }

    /* Register callback for handling ZCL commands. */
    auto dev_cb = zb::tpl_device_cb<
	zb::dev_cb_handlers_desc{ .error_handler = on_dev_cb_error }
	//handler
    , zb::set_attr_val_gen_desc_t{
	{
	    .ep = kACCEL_EP,
	    .cluster = zb::kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS,
	    .attribute = zb::kZB_ATTR_ID_MAIN_SETTINGS
	},
	to_settings_handler<on_settings_changed>(ZbSettingsEntries::flags)
      }
    , zb::set_attr_val_gen_desc_t{
	{
	    .ep = kACCEL_EP,
	    .cluster = zb::kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS,
	    .attribute = zb::kZB_ATTR_ID_WAKE_SLEEP_THRESHOLD
	},
	to_settings_handler<on_wake_sleep_settings_changed>(ZbSettingsEntries::wake_sleep_threshold)
      }
    , zb::set_attr_val_gen_desc_t{
	{
	    .ep = kACCEL_EP,
	    .cluster = zb::kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS,
	    .attribute = zb::kZB_ATTR_ID_SLEEP_DURATION
	},
	to_settings_handler<on_wake_sleep_settings_changed>(ZbSettingsEntries::sleep_duration)
      }
    , zb::set_attr_val_gen_desc_t{
	{
	    .ep = kACCEL_EP,
	    .cluster = zb::kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS,
	    .attribute = zb::kZB_ATTR_ID_SLEEP_TRACKING_RATE
	},
	to_settings_handler<on_wake_sleep_settings_changed>(ZbSettingsEntries::sleep_tracking_rate)
      }
    , zb::set_attr_val_gen_desc_t{
	{
	    .ep = kACCEL_EP,
	    .cluster = zb::kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS,
	    .attribute = zb::kZB_ATTR_ID_ACTIVE_TRACKING_RATE
	},
	to_settings_handler<on_active_odr_changed>(ZbSettingsEntries::active_tracking_rate)
      }
    >;
    ZB_ZCL_REGISTER_DEVICE_CB(dev_cb);

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(zb_ctx);

    printk("Main: before zigbee enable\r\n");
    zigbee_enable();
    g_ResetRestartsLeftAlarm.Setup(
	    [](void*)
	    { 
		g_RestartsToFactoryResetLeft = kRestartCountToFactoryReset; 
		settings_save_subtree(SETTINGS_DEV_SUBTREE);
		printk("Restart left count reset\r\n");
	    }, nullptr, kRestartCounterResetTimeoutMS);

    printk("Main: sleep forever\r\n");
    while (1) {
	k_sleep(K_FOREVER);
    }
    return 0;
}
