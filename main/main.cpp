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
#include "buttons.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "cmd_system.h"
#include "cmd_nvs.h"

#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_console.h"
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

static adc_continuous_handle_t s_adc_handle = NULL;

static adc_cali_handle_t s_adc_caibration = NULL;
static uint32_t s_battVoltage;
static int s_go2next;
static SemaphoreHandle_t s_waitsem;

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

	static i2c_master_bus_handle_t bus;
	ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &devcfg, &s_i2cDev));
	return 0;
}

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}


static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle) {

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 256,
		.flags = {
			.flush_pool = 0,
		}
    };
    adc_continuous_handle_t handle;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));


    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {};
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
    }
    adc_continuous_config_t dig_cfg = {
		.pattern_num = channel_num,
		.adc_pattern = adc_pattern,
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static int adc_init() {
	static adc_channel_t channel[] = { BOARD_CFG_GPIO_ADC_VIN };

	continuous_adc_init(channel, ARRAY_SIZE(channel), &s_adc_handle);

    example_adc_calibration_init(ADC_UNIT_1, BOARD_CFG_GPIO_ADC_VIN, ADC_ATTEN_DB_12, &s_adc_caibration);
    return 0;
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {

	uint32_t rv = 0;
	uint32_t sz = edata->size / SOC_ADC_DIGI_RESULT_BYTES;
	adc_digi_output_data_t *ptr = (adc_digi_output_data_t*)edata->conv_frame_buffer;

	for (size_t i = 0; i < sz; ++i) {
		rv += ptr[i].type1.data;
	}
	rv /= sz;
	s_battVoltage = rv;

	if (user_data) {
		uint32_t *adcVal = (uint32_t*)user_data;
		*adcVal = rv;
	}

//    ESP_ERROR_CHECK(gpio_set_level(BOARD_CFG_GPIO_ENA_LDO, (rv < 3750)));

    return false;
}

static int console_init() {

	esp_console_repl_t *repl = NULL;
	esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
	repl_config.prompt = CONFIG_IDF_TARGET ":~$";
	repl_config.max_cmdline_length = 1024;

	esp_console_register_help_command();
	register_system_common();
	register_nvs();

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
    ESP_LOGI(TAG, "USB initialization");
#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
#else
#error Unsupported console type
#endif

    ESP_ERROR_CHECK(esp_console_start_repl(repl));

	return 0;
}

static const Initer_f s_inits[] = {
	init_gpio,
	init_i2c,
	adc_init,
//	LogUtil_init,
//	UiEngine_init,
//	WiFi_init,
//	Telnet_init,
	console_init,
};

static esp_timer_handle_t s_heartBeatTimer;

static void onHbTout(void* arg) {

	static int lvl = 1;

	lvl = !lvl;
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_2, lvl);

	esp_timer_stop(s_heartBeatTimer);
	esp_timer_start_periodic(s_heartBeatTimer, lvl ? 600 * 1000ULL : 100 * 1000ULL);
}

static void onButtonPress(int btn, int evt, void *arg) {
	SemaphoreHandle_t storage_lock = (SemaphoreHandle_t)arg;

	Buttons::Wrapper &buttonHolder = Buttons::Wrapper::instance();
	ESP_LOGE(TAG, "btn %d stt %d %d %d", btn, evt, gpio_get_level(GPIO_NUM_21), buttonHolder.pressedFor(btn));

	if (buttonHolder.pressedFor(btn) < 10 * 1000) {
		s_go2next += evt == Buttons::RELEASE;
		if (s_go2next)
			xSemaphoreGive(s_waitsem);
		return;
	}
	ESP_LOGI(TAG, "WIPING");
	xSemaphoreTake(storage_lock, portMAX_DELAY);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 1);
	Storage_wipe(s_i2cDev);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 0);
	esp_restart();
	xSemaphoreGive(storage_lock);
}

static void waiter(uint32_t ms, const int64_t base) {
	const int64_t endAt = ms * 1000UL + esp_timer_get_time();
	int prevPrc = -1;
	while (!s_go2next && (endAt > esp_timer_get_time())) {
		const uint32_t now = esp_timer_get_time() / 1000000UL + base;
		const int64_t delta = (endAt - esp_timer_get_time()) / 1000UL;
		const int wait4 = delta > 5000 ? 5000 : delta;
		const int prc = delta * 100 / ms;

		if (prevPrc != prc) {
			ESP_LOGD(TAG, "waiting %2d%% %dms", prc, wait4);
			UiEngine_SetPercent(prc, now);
			prevPrc = prc;
			int voltage = 0;
			ESP_ERROR_CHECK(adc_cali_raw_to_voltage(s_adc_caibration, s_battVoltage, &voltage));
			voltage = voltage * 2 * BOARD_CFG_ADC_RATIO_MV + 50;
			UiEngine_SetVoltage(voltage);
			ESP_LOGD(TAG, "wa %d -> %d", s_battVoltage, voltage);
		}

		xSemaphoreTake(s_waitsem, pdMS_TO_TICKS(wait4));
	}
	s_go2next = 0;
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

	s_waitsem = xSemaphoreCreateBinary();

	SemaphoreHandle_t storage_lock = xSemaphoreCreateRecursiveMutex();
	xSemaphoreGive(storage_lock);

	gpio_install_isr_service(0);

	Buttons::Wrapper &buttonHolder = Buttons::Wrapper::instance();
	buttonHolder.add(BOARD_CFG_GPIO_USER_BTN, 0, onButtonPress, storage_lock);


	xSemaphoreTake(storage_lock, portMAX_DELAY);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 1);
	Storage_t st;
	Storage_init(s_i2cDev, &st);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_1, 1);
	xSemaphoreGive(storage_lock);

	static const esp_timer_create_args_t hbc = {
		.callback = onHbTout,
		.arg = NULL,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "heartBeat",
		.skip_unhandled_events = false,
	};

	uint32_t adcVal = 0;

	if (s_adc_handle) {
		adc_continuous_evt_cbs_t cbs = {
			.on_conv_done = s_conv_done_cb,
			.on_pool_ovf = NULL,
		};
		ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(s_adc_handle, &cbs, &adcVal));
		ESP_ERROR_CHECK(adc_continuous_start(s_adc_handle));
	}

	ESP_ERROR_CHECK(esp_timer_create(&hbc, &s_heartBeatTimer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(s_heartBeatTimer, 100 * 1000ULL));

	xSemaphoreTake(s_waitsem, pdMS_TO_TICKS(100));

	UiEngine_init();
	UiEngine_SetCycles(st.itm.cycles);
	UiEngine_SetUptime(st.itm.uptime);
	ESP_LOGI(TAG, "Uptime %ld. cycles %ld. next addr %ld", st.itm.uptime, st.itm.cycles, st.nextAddr);

	ESP_LOGI(TAG, "do nothing for first 3 minutes");
	waiter(INITIAL_BLANK, st.itm.uptime);
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

		xSemaphoreTake(storage_lock, portMAX_DELAY);
		gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 1);
		int rv = Storage_save(s_i2cDev, &st);
		gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_FORCE_PWR, 0);
		xSemaphoreGive(storage_lock);
		if (rv)
			ESP_LOGE(TAG, "Save failure -%X", rv);
		else
			ESP_LOGI(TAG, "Saved %ld cycles. Worktime %ld", st.itm.cycles, st.itm.uptime);

		UiEngine_SetCycles(st.itm.cycles);
		UiEngine_SetUptime(st.itm.uptime);
//		UiEngine_forceredraw();

		waiter(CYCLE_TIME, st.itm.uptime);
	}


}
