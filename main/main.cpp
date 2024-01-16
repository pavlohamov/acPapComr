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
#include "driver/i2c.h"

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
#define CYCLE_TIME (100)
#define RLY_HOLD_TIME 100
#else
#define INITIAL_BLANK (3 * 60 * 1000)
#define CYCLE_TIME (6 * 1000)
#define RLY_HOLD_TIME 1000
#endif

static int init_gpio(void) {
	gpio_config_t io_conf = {
		.pin_bit_mask = BIT64(BOARD_CFG_GPIO_RLY_1) | BIT64(BOARD_CFG_GPIO_RLY_2) |
			BIT64(BOARD_CFG_GPIO_LED_0)| BIT64(BOARD_CFG_GPIO_LED_2),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	int rv = gpio_config(&io_conf);
	if (rv)
		return rv;

	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_1, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_2, 0);

	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_0, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_2, 0);

	return 0;
}

static int init_i2c() {

	int rv = i2c_driver_install(BOARD_CFG_I2C_BUS, I2C_MODE_MASTER, 0, 0, 0);
	if (rv) {
		ESP_LOGE(TAG, "i2c_driver_install %d", rv);
		return rv;
	}

	static const i2c_config_t cfg = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = BOARD_CFG_GPIO_SDA,
			.scl_io_num = BOARD_CFG_GPIO_SCL,
			.sda_pullup_en = false,
			.scl_pullup_en = false,
			.master = { .clk_speed = 400 * 1000} ,
			.clk_flags = 0,
	};

	rv = i2c_param_config(BOARD_CFG_I2C_BUS, &cfg);
	if (rv) {
		ESP_LOGE(TAG, "i2c_param_config %d", rv);
		return rv;
	}

	return rv;
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

static void waiter(uint32_t ms) {
	const int64_t endAt = ms * 1000UL + esp_timer_get_time();
	while (endAt > esp_timer_get_time()) {
		const int64_t delta = (endAt - esp_timer_get_time()) / 1000UL;
		const int wait4 = delta > 100 ? 100 : delta;

		const int prc = delta * 100 / ms;

		ESP_LOGD(TAG, "waiting %2d%%", prc);

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
	Storage_init(&st);

	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_2, 1);


	static const esp_timer_create_args_t hbc = {
			.callback = onHbTout,
			.arg = NULL,
			.dispatch_method = ESP_TIMER_TASK,
			.name = "heartBeat",
			.skip_unhandled_events = false,
	};

	ESP_ERROR_CHECK(esp_timer_create(&hbc, &s_heartBeatTimer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(s_heartBeatTimer, 100 * 1000ULL));

	// do nothing for first 3 minutes
//	waiter(INITIAL_BLANK);

	const int64_t startAt = esp_timer_get_time();
	const uint32_t worked4 = st.itm.uptime;

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

		int rv = Storage_save(&st);
		if (rv)
			ESP_LOGE(TAG, "Save failure -%X", rv);
		else
			ESP_LOGI(TAG, "Saved %ld cycles. Worktime %ld", st.itm.cycles, st.itm.uptime);

		waiter(CYCLE_TIME);
	}


}
