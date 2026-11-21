/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdlib.h>

#include "board_cfg.h"
#include "someGeneric.hpp"
#include "NvStorage.hpp"
#include "Buttons.hpp"
#include "relayer.hpp"

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

#define SAVE_PERIOD_MS 30'000


static adc_continuous_handle_t s_adc_handle = NULL;

static adc_cali_handle_t s_adc_caibration = NULL;
static uint32_t s_battVoltage;

static i2c_master_dev_handle_t s_i2cDev;


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
			.enable_internal_pullup = 1,
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
        adc_pattern[i].atten = ADC_ATTEN_DB_0;
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
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

static int adc_init() {
	static adc_channel_t channel[] = { BOARD_CFG_GPIO_ADC_VIN };

	continuous_adc_init(channel, ARRAY_SIZE(channel), &s_adc_handle);

    example_adc_calibration_init(ADC_UNIT_1, BOARD_CFG_GPIO_ADC_VIN, ADC_ATTEN_DB_0, &s_adc_caibration);
    return 0;
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {

	uint32_t rv = 0;
	uint32_t sz = edata->size / SOC_ADC_DIGI_RESULT_BYTES;
	adc_digi_output_data_t *ptr = (adc_digi_output_data_t*)edata->conv_frame_buffer;

	for (size_t i = 0; i < sz; ++i) {
		rv += ptr[i].type2.data;
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
	init_i2c,
	adc_init,
//	UiEngine_init,
//	WiFi_init,
	console_init,
};

static esp_timer_handle_t s_heartBeatTimer;

static void onHbTout(void* arg) {

	static int lvl = 1;

	lvl = !lvl;
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_3, lvl);

	esp_timer_stop(s_heartBeatTimer);
	esp_timer_start_periodic(s_heartBeatTimer, !lvl ? 600 * 1000ULL : 100 * 1000ULL);
}


#define PAGE_SIZE 32
#define SWAP_16(x) (((x << 8) & 0xFF00) | ((x >> 8) & 0xFF))

enum {
	EVT_Brownout,
	EVT_Save2storage,
	EVT_EraseStorage,
};

extern "C" void app_main() {

	esp_log_level_set(TAG, ESP_LOG_VERBOSE);

	Relayer relayer;
	if (!relayer.start(1, 4096)) {
		ESP_LOGE(TAG, "not started");
		esp_restart();
	}

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

	gpio_install_isr_service(0);

	typedef struct {
		uint32_t cycles;
		uint32_t uptime;
	} OdoSave_t;

	// s_i2cDev
	struct FlashPart {
		bool read(off_t off, void* data, size_t size) {
			uint16_t addr = SWAP_16(off);
			int rv = i2c_master_transmit_receive(s_i2cDev, (uint8_t*)&addr, sizeof(addr), (uint8_t*)data, size, 150);
			if (rv) {
				ESP_LOGE(TAG, "read @%X fail %d", off, rv);
				return false;
			}
			// ESP_LOGV(TAG, "read @%X ok", off);
			return true;
		}
		bool write(off_t off, const void* data, size_t size) {
			const uint8_t* ptr = static_cast<const uint8_t*>(data);

			size_t done = 0;
			int rv = 0;
			while (done < size) {
				off_t dst = off + done;
				size_t to_write = PAGE_SIZE - (dst % PAGE_SIZE);
				to_write = std::min(to_write, size - done);

				uint16_t addr = SWAP_16(dst);
				i2c_master_transmit_multi_buffer_info_t bu[] = {
					{ .write_buffer = (uint8_t*)&addr, .buffer_size = sizeof(addr) },
					{ .write_buffer = ptr + done, .buffer_size = to_write },
				};
				rv = i2c_master_multi_buffer_transmit(s_i2cDev, bu, 2, 150);
				if (rv) {
					ESP_LOGE(TAG, "write @%X fail %d %zu of %zu", dst, rv, done, size);
					break;
				}
				done += to_write;
				if (done < size) {
					vTaskDelay(pdMS_TO_TICKS(6));
				}
			}
			if (rv) {
				ESP_LOGE(TAG, "write @%X fail %d", off, rv);
				return false;
			}
			// ESP_LOGV(TAG, "write @%X ok", off);
			return true;
		}
		bool erase(size_t size) {
			ESP_LOGI(TAG, "erase @%X", size);
			static const uint8_t buff[32] = {};
			for (size_t i = 0; i < 2048; i += sizeof(buff)) {
				uint16_t addr = SWAP_16(i);
				i2c_master_transmit_multi_buffer_info_t bu[] = {
					{ .write_buffer = (uint8_t*)&addr, .buffer_size = sizeof(addr) },
					{ .write_buffer = buff, .buffer_size = sizeof(buff) },
				};

				int rv = i2c_master_multi_buffer_transmit(s_i2cDev, bu, 2, 150);
				if (rv) {
					ESP_LOGE(TAG, "write @%X fail %d %zu of %zu", addr, rv, i, size);
					break;
				}
				vTaskDelay(pdMS_TO_TICKS(6));
			}
			return true;
		}
	} fp;

	NvStorage<OdoSave_t, 2048, FlashPart> stor{fp};

	QueueHandle_t queue = xQueueCreate(16, sizeof(uint32_t));

	auto onButtonPress = [&relayer, &queue](int btn, int evt, void *arg) {
		Buttons::Wrapper &buttonHolder = Buttons::Wrapper::instance();
		if (evt != Buttons::RELEASE) // todo: fix call from isr
			ESP_LOGE(TAG, "btn %d stt %d %d", btn, evt, buttonHolder.pressed_for(btn));

		relayer.forceRelay(evt != Buttons::RELEASE);

		if (buttonHolder.pressed_for(btn) < 10 * 1000) {
			if (evt == Buttons::RELEASE)
				relayer.go2next();
			return;
		}

		uint32_t sig = EVT_EraseStorage;
		if (!xPortInIsrContext()) {
			xQueueSend(queue, &sig, 0);
			return;
		}
		BaseType_t hpw = pdFALSE;
		xQueueSendFromISR(queue, &sig, &hpw);
		if (hpw) {
			portYIELD_FROM_ISR();
		}
	};
	auto onBrownout = [&queue](int btn, int evt, void *arg) {
		uint32_t sig = EVT_Brownout;
		if (!xPortInIsrContext()) {
			xQueueSend(queue, &sig, 0);
			return;
		}
		BaseType_t hpw = pdFALSE;
		xQueueSendFromISR(queue, &sig, &hpw);
		if (hpw) {
			portYIELD_FROM_ISR();
		}
	};

	Buttons::Wrapper &buttonHolder = Buttons::Wrapper::instance();
	buttonHolder.add(BOARD_CFG_GPIO_USER_BTN, 0, 0, onButtonPress, nullptr);
	buttonHolder.add(BOARD_CFG_GPIO_BROWNOUT, 0, 1, onBrownout, nullptr);

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

	OdoSave_t motorhours = {};
	stor.find(motorhours);

	UiEngine_init();
	UiEngine_SetText("booting");
	ESP_LOGW(TAG, "Uptime %ld. cycles %ld", motorhours.uptime, motorhours.cycles);
	ESP_LOGI(TAG, "do nothing for first 3 minutes");


//	char text[33];
//	snprintf(text, sizeof(text), "initial wait for %d sec", INITIAL_BLANK / 1000);
//	UiEngine_SetText(text);
//	waiter(motorhours.cycles, INITIAL_BLANK, motorhours.uptime);
//	ESP_LOGI(TAG, "Waiting is done");

	const uint32_t clycled4 = motorhours.cycles;
	const uint32_t worked4 = motorhours.uptime;

	const uint32_t startAt = esp_timer_get_time() / 1000ULL;
	uint64_t saveAt_ms = startAt;
	while (1) {
		const uint64_t now_ms = esp_timer_get_time() / 1000ULL;
		const uint32_t currUptime = (now_ms - startAt) / 1000ULL + worked4;
		uint32_t sig = -1;
		if (!xQueueReceive(queue, &sig, pdMS_TO_TICKS(25))) {
			int voltage = 0;
			ESP_ERROR_CHECK(adc_cali_raw_to_voltage(s_adc_caibration, s_battVoltage, &voltage));
			voltage = voltage * BOARD_CFG_ADC_RATIO_MV;
			UiEngine_SetData(motorhours.cycles, voltage + 280, 0, currUptime);
			UiEngine_SetText(relayer.info().c_str());
			if (now_ms - saveAt_ms > SAVE_PERIOD_MS)
				sig = EVT_Save2storage;
		}

		switch (sig) {
			case EVT_Brownout:
				ESP_LOGE(TAG, "brownout");
				break;
			case EVT_Save2storage: {
				motorhours.cycles = clycled4 + relayer.cycles();
				motorhours.uptime = currUptime;
				ESP_LOGW(TAG, "saving Uptime %ld. cycles %ld %5d adc", motorhours.uptime, motorhours.cycles, s_battVoltage);
				const uint32_t saveStart = esp_timer_get_time() / 1000ULL;
				if (stor.set(motorhours)) {
					const uint32_t saved = esp_timer_get_time() / 1000ULL - saveStart;
					ESP_LOGI(TAG, "save took %dms", saved);
					saveAt_ms = now_ms;
				} else {
					ESP_LOGE(TAG, "save failed");
				}
			} break;

			case EVT_EraseStorage:
				ESP_LOGE(TAG, "user requested storage wipe");
				stor.erase();
				esp_restart();
				break;
		}
	}


}
