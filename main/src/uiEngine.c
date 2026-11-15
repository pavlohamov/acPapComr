/*
 * uiEngine.cpp
 *
 *  Created on: 12 Jan 2024
 *      Author: pavloha
 */


#include "board_cfg.h"
#include "someGeneric.hpp"

#include <stdio.h>

#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "lvgl.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_timer.h"

#include "esp_dma_utils.h"

#define LCD_HOST  SPI2_HOST

#define FB_LENGTH (BOARD_CFG_DISPLAY_WIDTH * BOARD_CFG_DISPLAY_HEIGHT)
#define FB_SIZE (FB_LENGTH * sizeof(lv_color16_t) / 10)

#define LVGL_TICK_PERIOD_MS 100

#include "esp_log.h"
static const char *TAG = "UiEngine";

#ifndef BOARD_CFG_DISPLAY

int UiEngine_init(void) { return 0; }
bool UiEngine_lock(int timeout_ms) { return true; }
void UiEngine_unlock(void) {}

#else

static struct {
	SemaphoreHandle_t lock;
	SemaphoreHandle_t waiter;
	TaskHandle_t tid;

	esp_lcd_panel_handle_t panel_handle;
	lv_display_t *disp;

	lv_obj_t *cycles;
	lv_obj_t *uptime;
	lv_obj_t *percent;

} s_uiInternals;


static void on_lvgl_tick(void *arg) {
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void cfg_lcd_panel(esp_lcd_panel_handle_t panel_handle) {
	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

	ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, 1));
	ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, 0, 1));
	ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 18, 82));
	ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, 0));
//
//	ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, 0));
//	ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, 1, 1));
//	esp_lcd_panel_set_gap(panel_handle, 18, 82);
}

static void lvgl_routine(void *arg) {
    ESP_LOGI(TAG, "Started LVGL task");

	uint32_t wait4 = 50;

	uint32_t startAt = esp_timer_get_time() / 1'000'000ULL;
	while (1) {
		const uint64_t now_s = esp_timer_get_time() / 1'000'000ULL;
		if (wait4 > 50)
			wait4 = 50;
		xSemaphoreTake(s_uiInternals.waiter, pdMS_TO_TICKS(wait4));

		if (UiEngine_lock(-1)) {
			if (now_s - startAt > 10) {
			    ESP_LOGI(TAG, "reinit panel");
				cfg_lcd_panel(s_uiInternals.panel_handle);
				startAt = now_s;
			}
			wait4 = lv_timer_handler();
			UiEngine_unlock();
		}
	}
}


static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}


static esp_lcd_panel_io_handle_t init_lcd_bus(void) {

	spi_bus_config_t buscfg = {
		.sclk_io_num = BOARD_CFG_GPIO_LCD_CLK,
		.mosi_io_num = BOARD_CFG_GPIO_LCD_MOSI,
		.miso_io_num = -1,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = BOARD_CFG_DISPLAY_WIDTH * BOARD_CFG_DISPLAY_HEIGHT * 2,
	};
	ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));


    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BOARD_CFG_GPIO_LCD_DC,
        .cs_gpio_num = BOARD_CFG_GPIO_LCD_CS,
        .pclk_hz = 20e6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
    };

    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
    return io_handle;
}

static esp_lcd_panel_handle_t init_lcd_panel(esp_lcd_panel_io_handle_t io_handle) {

	static const esp_lcd_panel_dev_config_t panel_config = {
		.reset_gpio_num = BOARD_CFG_GPIO_LCD_RST,    // Set to -1 if not use
		.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,  // RGB element order: R-G-B
		.bits_per_pixel = 16,                        // Implemented by LCD command `3Ah` (16/18)
		// .vendor_config = &vendor_config,          // Uncomment this line if use custom initialization commands
	};

	esp_lcd_panel_handle_t panel_handle = NULL;
	ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
	cfg_lcd_panel(panel_handle);

//static uint16_t line[BOARD_CFG_DISPLAY_WIDTH];
//
//	while (1) {
//		for (int y = 0; y < BOARD_CFG_DISPLAY_HEIGHT; ++y) {
//			memset(line, 0, sizeof(line));
//			esp_lcd_panel_draw_bitmap(panel_handle, 0, y, BOARD_CFG_DISPLAY_WIDTH, y + 1, line);
//		}
//
//		int val = 0;
//		for (int y = 0; y < BOARD_CFG_DISPLAY_HEIGHT; ++y) {
//			memset(line, val, sizeof(line));
//			val = ~val;
//			esp_lcd_panel_draw_bitmap(panel_handle, 0, y, BOARD_CFG_DISPLAY_WIDTH, y + 1, line);
//			vTaskDelay(pdMS_TO_TICKS(10));
//		}
//	}

	return panel_handle;
}


static void example_lvgl_port_update_callback(lv_display_t *disp) {
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);

    switch (rotation) {
    case LV_DISPLAY_ROTATION_0:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISPLAY_ROTATION_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISPLAY_ROTATION_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISPLAY_ROTATION_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
//    example_lvgl_port_update_callback(disp);
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
//    static int val = 0;
//    memset(px_map, val, (area->y2 - area->y1) * (area->x2 - area->x1) * sizeof(lv_color16_t));
//    val = ~val;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

static lv_display_t* init_lvgl(esp_lcd_panel_handle_t panel_handle) {

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // create a lvgl display
    lv_display_t *display = lv_display_create(BOARD_CFG_DISPLAY_WIDTH, BOARD_CFG_DISPLAY_HEIGHT);

    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, FB_SIZE, 0);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, FB_SIZE, 0);
    assert(buf1);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, buf2, FB_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // associate the mipi panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    // set color depth
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = on_lvgl_tick,
        .name = "lvgl_tick",
		.arg = 0,
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000ULL));

    return display;
}

bool UiEngine_lock(int timeout_ms) {
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(s_uiInternals.lock, timeout_ticks) == pdTRUE;
}

void UiEngine_unlock(void) {
    xSemaphoreGiveRecursive(s_uiInternals.lock);
}

int UiEngine_SetData(uint32_t cyc, uint32_t mv, uint32_t prc, uint32_t sec) {
	if (!UiEngine_lock(200)) {
		ESP_LOGE(TAG, "UI locked");
		return 0;
	}

	const int sc = sec % 60;
	const int m = (sec / 60) % 60;
	const int h = sec / 3600;

	lv_label_set_text_fmt(s_uiInternals.cycles, "Cycles %6ld.  BAT %2ld.%02ldV %02lds", cyc, mv / 1000, (mv / 10) % 100, prc);
	if (!h)
		lv_label_set_text_fmt(s_uiInternals.uptime, "MotoMinutes %2d:%2d", m, sc);
	else
		lv_label_set_text_fmt(s_uiInternals.uptime, "MotoHrs: %d (min %2d:%2d)", h, m, sc);

	UiEngine_unlock();
	return 0;
}

int UiEngine_SetText(const char *text) {

	if (!UiEngine_lock(200)) {
		ESP_LOGE(TAG, "UI locked");
		return 0;
	}
	lv_label_set_text_fmt(s_uiInternals.percent, "%s", text);
	UiEngine_unlock();
    ESP_LOGI(TAG, "UI: '%s'", text);
	return 0;
}

int UiEngine_init(void) {

	if (s_uiInternals.tid)
		return 0;

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BOARD_CFG_GPIO_LCD_BUSY
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(BOARD_CFG_GPIO_LCD_BUSY, 0);

	s_uiInternals.lock = xSemaphoreCreateRecursiveMutex();
	s_uiInternals.waiter = xSemaphoreCreateBinary();

    esp_lcd_panel_io_handle_t io_handle = init_lcd_bus();
    s_uiInternals.panel_handle = init_lcd_panel(io_handle);

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_uiInternals.panel_handle, true));

    s_uiInternals.disp = init_lvgl(s_uiInternals.panel_handle);
	if (!s_uiInternals.disp) {
		ESP_LOGE(TAG, "Failed to init lvgl");
		return -1;
	}
	xSemaphoreGiveRecursive(s_uiInternals.lock);

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, s_uiInternals.disp));

	lv_obj_t *scr = lv_scr_act();

	s_uiInternals.cycles = lv_label_create(scr);
	s_uiInternals.uptime = lv_label_create(scr);
	s_uiInternals.percent = lv_label_create(scr);

	lv_label_set_text(s_uiInternals.cycles, "...");
	lv_label_set_text(s_uiInternals.uptime, "...");
	lv_label_set_text(s_uiInternals.percent, "...");

	lv_obj_align(s_uiInternals.cycles, LV_ALIGN_TOP_LEFT, 0, 0);
	lv_obj_align_to(s_uiInternals.uptime, s_uiInternals.cycles, LV_ALIGN_OUT_BOTTOM_MID, 0, 3);
	lv_obj_align_to(s_uiInternals.percent, s_uiInternals.uptime, LV_ALIGN_OUT_BOTTOM_MID, 0, 3);

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(lvgl_routine, "LVGL", 5 * 1024, NULL, 2, &s_uiInternals.tid);
    return 0;
}
#endif
