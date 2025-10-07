/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/settings/settings.h>

#include <ram_pwrdn.h>

//#define ALARM_LIST_LOCK_TYPE thread::DummyLock
#include <nrfzbcpp/zb_main.hpp>
#include <nrfzbcpp/zb_std_cluster_desc.hpp>
#include <nrfzbcpp/zb_power_config_cluster_desc.hpp>
#include <nrfzbcpp/zb_poll_ctrl_cluster_desc.hpp>

#include <nrfzbcpp/zb_status_cluster_desc.hpp>

#include <nrfzbcpp/zb_alarm.hpp>

#include "zb/zb_accel_settings.hpp"
#include "zb/zb_accel_cluster_desc.hpp"

#include <zephyr/drivers/sensor/lis2du12.h>

constexpr bool kPowerSaving = false;//true;

/**********************************************************************/
/* Zigbee Declarations and Definitions                                */
/**********************************************************************/
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

constexpr auto kAttrX = &device_ctx_t::accel_type::x;
constexpr auto kAttrY = &device_ctx_t::accel_type::y;
constexpr auto kAttrZ = &device_ctx_t::accel_type::z;
constexpr auto kCmdOnWakeUpEvent = &device_ctx_t::accel_type::on_wake_up;
constexpr auto kCmdOnSleepEvent = &device_ctx_t::accel_type::on_sleep;
constexpr auto kCmdOnFlipEvent = &device_ctx_t::accel_type::on_flip;
constexpr auto kAttrStatus1 = &zb::zb_zcl_status_t::status1;
constexpr auto kAttrBattVoltage = &zb::zb_zcl_power_cfg_battery_info_t::batt_voltage;
constexpr auto kAttrBattPercentage = &zb::zb_zcl_power_cfg_battery_info_t::batt_percentage_remaining;

constexpr uint32_t kPowerCycleThresholdSeconds = 6 * 60 - 1; //Just under 6 minutes


/**********************************************************************/
/* Persisten settings                                                 */
/**********************************************************************/
#define SETTINGS_ZB_ACCEL_SUBTREE "zb_accel"
struct ZbSettingsEntries
{
    inline static constexpr const char flags[] = SETTINGS_ZB_ACCEL_SUBTREE "/accel_flags";
    inline static constexpr const char wake_sleep_threshold[] = SETTINGS_ZB_ACCEL_SUBTREE "/wake_sleep_threshold";
    inline static constexpr const char sleep_duration[] = SETTINGS_ZB_ACCEL_SUBTREE "/sleep_duration";
    inline static constexpr const char sleep_tracking_rate[] = SETTINGS_ZB_ACCEL_SUBTREE "/sleep_tracking_rate";
};

using namespace zb::literals;
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
	    .check_in_interval = 2_min_to_qs,
	    .long_poll_interval = 60_min_to_qs,
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

constinit static auto &zb_ep = zb_ctx.ep<kACCEL_EP>();

template<> 
struct zb::cluster_custom_handler_t<device_ctx_t::accel_type, kACCEL_EP>: cluster_custom_handler_base_t<custom_accel_handler_t>
{
    //the rest will be done by cluster_custom_handler_base_t
    static auto& get_device() { return zb_ctx; }
};


static bool g_ZigbeeReady = false;

/**********************************************************************/
/* ZephyrOS devices                                                   */
/**********************************************************************/

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

//#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
//static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static const struct device *const accel_dev = DEVICE_DT_GET(DT_NODELABEL(accel));
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/**********************************************************************/
/* Accelerometer stuff                                                */
/**********************************************************************/
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
		res.flip_x = TryUpdateMeasurement(newVals.x, m_LastMeasuredX);
	    if (dev_ctx.settings.flags.enable_y)
		res.flip_y = TryUpdateMeasurement(newVals.y, m_LastMeasuredY);
	    if (dev_ctx.settings.flags.enable_z)
		res.flip_z = TryUpdateMeasurement(newVals.z, m_LastMeasuredZ);
	}
	return res;
    }
};

FlipTracker g_Flip;

static lis2du12_trigger g_WakeUpTrigger{ 
    .trig={.type = (enum sensor_trigger_type)LIS2DU12_TRIG_WAKE_UP, .chan = (enum sensor_channel)LIS2DU12_CHAN_ACCEL_XYZ_EXT}, 
    .wake_cfg = { .x_enable=0, .y_enable=0, .z_enable = 1, .wake_threshold = 1, .wake_duration = 1}
};

static lis2du12_trigger g_SleepTrigger{ 
    .trig={.type = (enum sensor_trigger_type)LIS2DU12_TRIG_SLEEP_CHANGE, .chan = (enum sensor_channel)LIS2DU12_CHAN_ACCEL_XYZ_EXT}, 
    .wake_cfg = { .x_enable=0, .y_enable=0, .z_enable = 1, .sleep_on = 1, .sleep_duration = 15, .wake_threshold = 1, .wake_duration = 0}
};

void on_cmd_sent(zb::cmd_id_t cmd_id, zb_zcl_command_send_status_t *status)
{
    printk("zb: on_cmd_sent id:%d; status: %d\r\n", cmd_id, status->status);
}

static constinit thread::SyncVar<bool> g_SleepZbPosted{false};
void on_sleep_zb(uint8_t buf)
{
    g_SleepZbPosted = false;
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

    if (dev_ctx.settings.flags.track_sleep)
	zb_ep.send_cmd<kCmdOnSleepEvent, {.cb=on_cmd_sent}>(vals);
    if (dev_ctx.settings.flags.track_flip)
    {
	auto flipRes = g_Flip.CheckFlip(acc);
	if (flipRes)
	    zb_ep.send_cmd<kCmdOnFlipEvent, {.cb=on_cmd_sent}>(flipRes);
    }
}

void on_sleep(const struct device *dev, const struct sensor_trigger *trigger)
{
    //post on Zigbee thread and run immedieately
    if (!g_SleepZbPosted.exchange(true))
    {
	printk("sleep detected -> zb\r\n");
	zb_schedule_app_alarm(on_sleep_zb, 0, 0);
    }
    else
    {
	printk("sleep detected -> dropped\r\n");
    }
}

static constinit thread::SyncVar<bool> g_WakeUpZbPosted{false};
void on_wake_up_zb(uint8_t buf)
{
    g_WakeUpZbPosted = false;
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

    zb_ep.send_cmd<kCmdOnWakeUpEvent, {.cb=on_cmd_sent}>(vals);
}

void on_wake_up(const struct device *dev, const struct sensor_trigger *trigger)
{
    //post on Zigbee thread and run immedieately
    if (!g_WakeUpZbPosted.exchange(true))
    {
	printk("wake up detected -> zb\r\n");
	zb_schedule_app_alarm(on_wake_up_zb, 0, 0);
    }else
    {
	printk("wake up detected -> dropped\r\n");
    }
}

void reconfigure_interrupts()
{
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
	//printk("Accel not ready");
	//printk("Inv Delta: %.2lf\r\n", (double)g_inv_delta);
    }
}

void on_settings_changed(const uint32_t &v)
{
    printk("Settings. Now: %X; Stored: %X\r\n", v, dev_ctx.settings.flags_dw);
    reconfigure_interrupts();
}

void on_wake_sleep_settings_changed()
{
    printk("Settings changed\r\n");
    reconfigure_interrupts();
}

zb::ZbTimerExt16 g_PeriodicAccel;

void on_zigbee_start()
{
    printk("on_zigbee_start\r\n");
    g_ZigbeeReady = true;
    zb_zcl_poll_control_start(0, kACCEL_EP);
    //zb_zcl_poll_controll_register_cb(&udpate_accel_values);
    //g_PeriodicAccel.Setup([]{ udpate_accel_values(0); return true; }, 10000);

    if constexpr (kPowerSaving)
    {
	if (dev_ctx.poll_ctrl.long_poll_interval != 0xffffffff)
	    zb_zdo_pim_set_long_poll_interval(dev_ctx.poll_ctrl.long_poll_interval * 1000 / 4);
    }
    else
	zb_zdo_pim_set_long_poll_interval(1000 * 10);

    //should be there already, initial state
    udpate_accel_values(0);
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
	printk("zboss: sig handler, sigid %d; status: %d\r\n", signalId, status);
	auto ret = zb::tpl_signal_handler<zb::sig_handlers_t{
	.on_leave = +[]{ 
	    zb_zcl_poll_control_stop(); 
	    k_sleep(K_MSEC(2100));
	    sys_reboot(SYS_REBOOT_COLD);
	},
	    //.on_error = []{ led::show_pattern(led::kPATTERN_3_BLIPS_NORMED, 1000); },
	    .on_dev_reboot = on_zigbee_start,
	    .on_steering = on_zigbee_start,
	    .on_can_sleep = &zb_sleep_now,
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

int main(void)
{
    int ret;
    bool led_state = true;

    printk("Main start\r\n");
    if (!gpio_is_ready_dt(&led)) {
	return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
	return 0;
    }

    if (device_is_ready(accel_dev))
    {
	reconfigure_interrupts();
	//ret = sensor_trigger_set(accel_dev, &g_WakeUpTrigger.trig, &on_wakeup);
	//if (ret != 0) printk("Failed to set trigger on wakeup: %d\r\n", ret);
	//else printk("Set trigger on wakeup\r\n");
    }else
    {
	printk("Accelerometer is not ready\r\n");
	dev_ctx.status_attr.status1 = -1;
    }

    printk("Main: before settings init\r\n");
    int err = settings_subsys_init();

    printk("Main: before zigbee erase persistent storage\r\n");
    //TODO: implement a counter logic: if reset count reaches 3 -> zigbee reset
    //after 10 seconds of activity the count is reset to 0
    zigbee_erase_persistent_storage(false);
    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(1000*60*30));

    if constexpr (kPowerSaving)
    {
	zb_set_rx_on_when_idle(false);
	zigbee_configure_sleepy_behavior(true);
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
	zb::to_handler_v<on_settings_changed>
	, zb::settings_v<ZbSettingsEntries::flags, dev_ctx.settings.flags_dw>
      }
    , zb::set_attr_val_gen_desc_t{
	{
	    .ep = kACCEL_EP,
	    .cluster = zb::kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS,
	    .attribute = zb::kZB_ATTR_ID_WAKE_SLEEP_THRESHOLD
	},
	zb::to_handler_v<on_wake_sleep_settings_changed>
	, zb::settings_v<ZbSettingsEntries::wake_sleep_threshold, dev_ctx.settings.wake_sleep_threshold>
      }
    , zb::set_attr_val_gen_desc_t{
	{
	    .ep = kACCEL_EP,
	    .cluster = zb::kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS,
	    .attribute = zb::kZB_ATTR_ID_SLEEP_DURATION
	},
	zb::to_handler_v<on_wake_sleep_settings_changed>
	, zb::settings_v<ZbSettingsEntries::sleep_duration, dev_ctx.settings.sleep_duration>
      }
    , zb::set_attr_val_gen_desc_t{
	{
	    .ep = kACCEL_EP,
	    .cluster = zb::kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS,
	    .attribute = zb::kZB_ATTR_ID_SLEEP_TRACKING_RATE
	},
	zb::to_handler_v<on_wake_sleep_settings_changed>
	, zb::settings_v<ZbSettingsEntries::sleep_tracking_rate, dev_ctx.settings.sleep_odr>
      }
    >;
    ZB_ZCL_REGISTER_DEVICE_CB(dev_cb);

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(zb_ctx);

    printk("Main: before settings load\r\n");
    err = settings_load();

    //settings_save_one(, const void *value, size_t val_len)

    if constexpr (kPowerSaving)
    {
	power_down_unused_ram();
    }

    printk("Main: before zigbee enable\r\n");
    zigbee_enable();

    printk("Main: sleep forever\r\n");
    while (1) {
	k_sleep(K_FOREVER);
    }
    return 0;
}
//SETTINGS_STATIC_HANDLER_DEFINE()
