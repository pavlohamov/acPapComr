/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdlib.h>

#include "board_cfg.h"
#include "someGeneric.hpp"
#include "telnet.h"
#include "storage.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
static const char *TAG = "ph";

typedef int (*Initer_f)();

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))
#endif

#if 0
#define INITIAL_BLANK (300)
#define CYCLE_TIME (4000)
#define RLY_HOLD_TIME 1000
#else
#define INITIAL_BLANK (3 * 60 * 1000)
#define CYCLE_TIME (60 * 1000)
#define RLY_HOLD_TIME 1000
#endif

static i2c_master_dev_handle_t s_i2cDev;

static int init_gpio(void) {
	static const gpio_config_t o_conf = {
		.pin_bit_mask = BIT64(BOARD_CFG_GPIO_RLY_1) | BIT64(BOARD_CFG_GPIO_RLY_2) |
			BIT64(BOARD_CFG_GPIO_LED_0) | BIT64(BOARD_CFG_GPIO_LED_2) | BIT64(BOARD_CFG_GPIO_LED_1) | BIT64(BOARD_CFG_GPIO_LED_3)
			| BIT64(BOARD_CFG_GPIO_FORCE_PWR),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	int rv = gpio_config(&o_conf);
	if (rv)
		return rv;
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 1);

	static const gpio_config_t i_conf = {
		.pin_bit_mask = BIT64(BOARD_CFG_GPIO_NPGOOD)
#ifdef BOARD_CFG_GPIO_BROWNOUT
				| BIT64(BOARD_CFG_GPIO_BROWNOUT)
#endif
				,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	rv = gpio_config(&i_conf);
	if (rv)
		return rv;

	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_1, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_2, 0);

	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_0, 1);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_1, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_2, 1);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_3, 1);

	return 0;
}

static int init_i2c() {

	static const i2c_master_bus_config_t cfg = {
			.i2c_port = BOARD_CFG_I2C_BUS,
			.sda_io_num = BOARD_CFG_GPIO_SDA,
			.scl_io_num = BOARD_CFG_GPIO_SCL,
			.clk_source = I2C_CLK_SRC_DEFAULT,
#if SOC_LP_I2C_SUPPORTED
			.lp_source_clk = LP_I2C_SCLK_DEFAULT,
#endif
			.glitch_ignore_cnt = 7,
			.intr_priority = 0,
			.trans_queue_depth = 0,
			.flags = {
					.enable_internal_pullup = 0,
					.allow_pd = 0,
			},
	};

	static const i2c_device_config_t devcfg = {
			.dev_addr_length = I2C_ADDR_BIT_LEN_7,
			.device_address = 0x50,
			.scl_speed_hz = 400 * 1000,
			.scl_wait_us = 0,
			.flags = {
					.disable_ack_check = 0,
			},
	};

	i2c_master_bus_handle_t bus;
	ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));

	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &devcfg, &s_i2cDev));

	return 0;
}

static const Initer_f s_inits[] = {
	init_gpio,
	init_i2c,
//	LogUtil_init,
	UiEngine_init,
//	WiFi_init,
//	Telnet_init,
};

static esp_timer_handle_t s_heartBeatTimer;

static void onHbTout(void* arg) {

	static int lvl = 1;

	lvl = !lvl;
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_2, lvl);

	esp_timer_stop(s_heartBeatTimer);
	esp_timer_start_periodic(s_heartBeatTimer, lvl ? 600 * 1000ULL : 100 * 1000ULL);
}

static void waiter(uint32_t ms, const int64_t base) {
	const int64_t endAt = ms * 1000UL + esp_timer_get_time();
	int prevPrc = -1;
	while (endAt > esp_timer_get_time()) {
		const uint32_t now = esp_timer_get_time() / 1000000UL + base;
		const int64_t delta = (endAt - esp_timer_get_time()) / 1000UL;
		const int wait4 = delta > 5000 ? 5000 : delta;
		const int prc = delta * 100 / ms;

		if (prevPrc != prc) {
			ESP_LOGD(TAG, "waiting %2d%% %dms", prc, wait4);
			UiEngine_SetPercent(prc, now);
			prevPrc = prc;
		}

		vTaskDelay(pdMS_TO_TICKS(wait4));
	}
}

extern "C" void app_main() {

	esp_log_level_set(TAG, ESP_LOG_VERBOSE);

	for (int i = 0; i < ARRAY_SIZE(s_inits); ++i) {
		int rv = s_inits[i]();
		if (rv) {
			ESP_LOGE(TAG, "%d %p failed with %d", i, s_inits[i], rv);
			vTaskDelay(pdMS_TO_TICKS(2000));
			esp_restart();
			return;
		} else {

			ESP_LOGI(TAG, "%d %p ok", i, s_inits[i]);
		}
	}


	Storage_t st;

	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 1);
	Storage_init(s_i2cDev, &st);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 0);

	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_1, 1);

	UiEngine_SetCycles(st.itm.cycles);
	UiEngine_SetUptime(st.itm.uptime);
	ESP_LOGI(TAG, "Uptime %ld. cycles %ld. next addr %ld", st.itm.uptime, st.itm.cycles, st.nextAddr);

	static const esp_timer_create_args_t hbc = {
			.callback = onHbTout,
			.arg = NULL,
			.dispatch_method = ESP_TIMER_TASK,
			.name = "heartBeat",
			.skip_unhandled_events = false,
	};

	ESP_ERROR_CHECK(esp_timer_create(&hbc, &s_heartBeatTimer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(s_heartBeatTimer, 100 * 1000ULL));

	ESP_LOGI(TAG, "do nothing for first 3 minutes");
//	waiter(INITIAL_BLANK, st.itm.uptime);
//	vTaskDelay(pdMS_TO_TICKS(INITIAL_BLANK));
	ESP_LOGI(TAG, "Waiting is done");

	const int64_t startAt = esp_timer_get_time();
	const uint32_t worked4 = st.itm.uptime;

	UiEngine_SetCycles(st.itm.cycles);
	UiEngine_SetUptime(st.itm.uptime);
	UiEngine_forceredraw();

	while (1) {

		ESP_LOGI(TAG, "Cycling");

		gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_0, 0);

		gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_1, 1);
		vTaskDelay(pdMS_TO_TICKS(RLY_HOLD_TIME));
		gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_1, 0);
		vTaskDelay(pdMS_TO_TICKS(RLY_HOLD_TIME));

		gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_2, 1);
		vTaskDelay(pdMS_TO_TICKS(RLY_HOLD_TIME));
		gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_2, 0);
		vTaskDelay(pdMS_TO_TICKS(RLY_HOLD_TIME));

		gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_0, 1);

		ESP_LOGI(TAG, "Cycle done. Saving");

		st.itm.cycles += 1;
		st.itm.uptime = worked4 + (esp_timer_get_time() - startAt) / 1000000ULL;

		if (gpio_get_level((gpio_num_t)BOARD_CFG_GPIO_NPGOOD)) {
			ESP_LOGE(TAG, "Power loss");
//			esp_restart();
//			return; // must not get here
		}

		gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 1);
		int rv = Storage_save(s_i2cDev, &st);
		gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 0);
		if (rv)
			ESP_LOGE(TAG, "Save failure -%X", rv);
		else
			ESP_LOGI(TAG, "Saved %ld cycles. Worktime %ld", st.itm.cycles, st.itm.uptime);

		UiEngine_SetCycles(st.itm.cycles);
		UiEngine_SetUptime(st.itm.uptime);
		UiEngine_forceredraw();

		waiter(CYCLE_TIME, st.itm.uptime);
	}


}
