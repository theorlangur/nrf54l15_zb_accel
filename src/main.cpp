/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */


//#include <iostream>
#include <memory>

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

#include <nrfzbcpp/zb_accel_cluster_desc.hpp>
#include <nrfzbcpp/zb_status_cluster_desc.hpp>

#include <nrfzbcpp/zb_alarm.hpp>

#include "zb/zb_accel_settings.hpp"

#include <zephyr/drivers/sensor/lis2du12.h>

constexpr bool kPowerSaving = false;//true;

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
typedef struct {
    zb::zb_zcl_basic_names_t basic_attr;
    zb::zb_zcl_power_cfg_battery_info_t battery_attr;
    zb::zb_zcl_poll_ctrl_basic_t poll_ctrl;
    zb::zb_zcl_accel_basic_t accel_attr;
    zb::zb_zcl_status_t status_attr;
    zb::zb_zcl_accel_settings_t settings;
} bulb_device_ctx_t;

constexpr auto kAttrX = &zb::zb_zcl_accel_basic_t::x;
constexpr auto kAttrY = &zb::zb_zcl_accel_basic_t::y;
constexpr auto kAttrZ = &zb::zb_zcl_accel_basic_t::z;
constexpr auto kCmdOnEvent = &zb::zb_zcl_accel_basic_t::on_event;
constexpr auto kAttrStatus1 = &zb::zb_zcl_status_t::status1;
constexpr auto kAttrBattVoltage = &zb::zb_zcl_power_cfg_battery_info_t::batt_voltage;
constexpr auto kAttrBattPercentage = &zb::zb_zcl_power_cfg_battery_info_t::batt_percentage_remaining;

constexpr uint32_t kPowerCycleThresholdSeconds = 6 * 60 - 1; //Just under 6 minutes

using namespace zb::literals;
/* Zigbee device application context storage. */
static constinit bulb_device_ctx_t dev_ctx{
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
	    .z = 0
	}
};

constinit static auto zb_ctx = zb::make_device(
	zb::make_ep_args<{.ep=kACCEL_EP, .dev_id=kDEV_ID, .dev_ver=1, .cmd_queue_depth = 4}>(
	    dev_ctx.basic_attr
	    , dev_ctx.battery_attr
	    , dev_ctx.poll_ctrl
	    , dev_ctx.accel_attr
	    , dev_ctx.status_attr
	    , dev_ctx.settings
	    )
	);

constinit static auto &zb_ep = zb_ctx.ep<kACCEL_EP>();

    /* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   2000

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

struct accel_settings
{
    uint32_t detect_flip_x    : 1;
    uint32_t detect_flip_y    : 1;
    uint32_t detect_flip_z    : 1;
    uint32_t regular_readings : 1;
    uint32_t unused           : 28;
};

constinit static accel_settings g_Settings{};
constexpr float kFlipThreshold = 1.f;

struct accel_val
{
    sensor_value x;
    sensor_value y;
    sensor_value z;
};

//static lis2du12_trigger g_WakeUpTrigger{ 
//    .trig={.type = (enum sensor_trigger_type)LIS2DU12_TRIG_WAKE_UP, .chan = (enum sensor_channel)LIS2DU12_CHAN_ACCEL_XYZ_EXT}, 
//    .wake_cfg = {.z_enable = 0, .threshold = 1}
//};
//void on_wakeup(const struct device *dev, const struct sensor_trigger *trigger)
//{
//    printk("wake up detected\r\n");
//    sensor_sample_fetch(accel_dev);
//    accel_val acc;
//    sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, &acc.x);
//}

static lis2du12_trigger g_SleepTrigger{ 
    .trig={.type = (enum sensor_trigger_type)LIS2DU12_TRIG_WAKE_UP, .chan = (enum sensor_channel)LIS2DU12_CHAN_ACCEL_XYZ_EXT}, 
    .wake_cfg = { .x_enable=1, .y_enable=1, .z_enable = 1, .threshold = 1}
};

static bool g_ZigbeeReady = false;

void on_cmd_sent(zb::cmd_id_t cmd_id, zb_zcl_command_send_status_t *status)
{
    printk("zb: on_cmd_sent\r\n");
    printk("zb: on_cmd_sent: status: %d\r\n", status->status);
}

void on_sleep_zb(uint8_t buf)
{
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

    zb::EventArgs events;
    if (g_Settings.detect_flip_x)
    {
	bool diff = std::abs(x - dev_ctx.accel_attr.x) > kFlipThreshold;
	if (diff)
	{
	    events.flip_x = 1;
	    //notify x-flip
	}
    }

    if (g_Settings.detect_flip_y)
    {
	bool diff = std::abs(y - dev_ctx.accel_attr.y) > kFlipThreshold;
	if (diff)
	{
	    //notify y-flip
	    events.flip_y = 1;
	}
    }

    if (g_Settings.detect_flip_z)
    {
	bool diff = std::abs(z - dev_ctx.accel_attr.z) > kFlipThreshold;
	if (diff)
	{
	    events.flip_z = 1;
	    //notify z-flip
	}
    }


    zb_ep.attr<kAttrX>() = x;
    zb_ep.attr<kAttrY>() = y;
    zb_ep.attr<kAttrZ>() = z;

    zb_ep.send_cmd<kCmdOnEvent, {.cb=on_cmd_sent}>(events);
}

void on_sleep(const struct device *dev, const struct sensor_trigger *trigger)
{
    //post on Zigbee thread and run immedieately
    printk("wake up detected -> zb\r\n");
    zb_schedule_app_alarm(on_sleep_zb, 0, 0);
}

void reconfigure_interrupts()
{
	//   if (   (g_WakeUpTrigger.wake_cfg.x_enable != g_Settings.detect_flip_x)
	//|| (g_WakeUpTrigger.wake_cfg.y_enable != g_Settings.detect_flip_y)
	//|| (g_WakeUpTrigger.wake_cfg.z_enable != g_Settings.detect_flip_z)
	//      )
    {
	//g_WakeUpTrigger.wake_cfg.x_enable = g_Settings.detect_flip_x;
	//g_WakeUpTrigger.wake_cfg.y_enable = g_Settings.detect_flip_y;
	//g_WakeUpTrigger.wake_cfg.z_enable = g_Settings.detect_flip_z;
	int ret;
	if (dev_ctx.settings.flags.enable_x || dev_ctx.settings.flags.enable_y || dev_ctx.settings.flags.enable_z)
	{
	    //ret = sensor_trigger_set(accel_dev, &g_WakeUpTrigger.trig, &on_wakeup);
	    //if (ret != 0) printk("Failed to set wake up trigger");
	    ret = sensor_trigger_set(accel_dev, &g_SleepTrigger.trig, &on_sleep);
	    if (ret != 0) printk("Failed to set sleep trigger\r\n");
	    else printk("Enabled wake interrupts\r\n");
	}else
	{
	    //ret = sensor_trigger_set(accel_dev, &g_WakeUpTrigger.trig, nullptr);
	    //if (ret != 0) printk("Failed to remove wake up trigger");
	    ret = sensor_trigger_set(accel_dev, &g_SleepTrigger.trig, nullptr);
	    if (ret != 0) printk("Failed to remove sleep trigger");
	    else printk("Disabled wake interrupts\r\n");
	}
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

zb::ZbTimerExt16 g_PeriodicAccel;

void on_zigbee_start()
{
    printk("on_zigbee_start\r\n");
    g_ZigbeeReady = true;
    zb_zcl_poll_control_start(0, kACCEL_EP);
    //zb_zcl_poll_controll_register_cb(&udpate_accel_values);
    g_PeriodicAccel.Setup([]{ udpate_accel_values(0); return true; }, 10000);

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
      }
    >;
    ZB_ZCL_REGISTER_DEVICE_CB(dev_cb);

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(zb_ctx);

    printk("Main: before settings load\r\n");
    err = settings_load();

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
