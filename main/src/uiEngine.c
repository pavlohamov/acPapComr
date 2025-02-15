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
#include "driver/i2c.h"
#include "driver/spi_master.h"

#include "esp_lcd_panel_ssd1681.h"
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
#define FB_SIZE (FB_LENGTH * sizeof(lv_color_t))

#define LVGL_TICK_PERIOD_MS 100

#include "lvgl.h"

#include "esp_log.h"
static const char *TAG = "UiEngine";

#ifndef BOARD_CFG_DISPLAY

int UiEngine_init(void) { return 0; }
bool UiEngine_lock(int timeout_ms) { return true; }
void UiEngine_unlock(void) {}

#else

static struct {
	SemaphoreHandle_t lock;
	SemaphoreHandle_t panelBusy;
	SemaphoreHandle_t waiter;
	TaskHandle_t tid;

	lv_disp_drv_t disp_drv;
	lv_disp_draw_buf_t disp_buf;

	lv_indev_drv_t indev_drv;
	int isr_fired;

	bool partialRfr;
	lv_obj_t *cycles;
	lv_obj_t *uptime;
	lv_obj_t *percent;
} s_uiInternals;

static bool ep_flush_ready_cb(const esp_lcd_panel_handle_t handle, const void *edata, void *user_data) {
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_data;
    lv_disp_flush_ready(disp_driver);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_uiInternals.panelBusy, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}

static uint8_t *converted_buffer_black;
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    ESP_LOGI(TAG, "FLUSH %d:%d %d:%d", offsetx1, offsetx2, offsety1, offsety2);

    int len_bits = (abs(offsetx1 - offsetx2)) * (abs(offsety1 - offsety2) + 1);

    memset(converted_buffer_black, 0x00, len_bits / 8);
    for (int i = 0; i < len_bits; i++) {
    	const int val = lv_color_brightness(color_map[i]) < 251;
        converted_buffer_black[i / 8] |= val << (7 - (i % 8));
    }

    ESP_LOGI(TAG, "%d %d %d %d lb %d", offsetx1, offsetx2, offsety1, offsety2, len_bits);
    ESP_ERROR_CHECK(epaper_panel_set_bitmap_color(panel_handle, SSD1681_EPAPER_BITMAP_BLACK));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, converted_buffer_black));

//    ESP_ERROR_CHECK(epaper_panel_set_bitmap_color(panel_handle, SSD1681_EPAPER_BITMAP_RED));
//    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, converted_buffer_black));
    if (s_uiInternals.partialRfr)
		ESP_ERROR_CHECK(epaper_panel_refresh_partial(panel_handle));
    else
    	ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
}

static void lvgl_wait_cb(struct _lv_disp_drv_t *disp_drv) {
    xSemaphoreTake(s_uiInternals.panelBusy, portMAX_DELAY);
}

static void lvgl_port_update_cb(lv_disp_drv_t *drv) {

    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    switch (drv->rotated) {
		case LV_DISP_ROT_NONE:
			// Rotate LCD display
			esp_lcd_panel_swap_xy(panel_handle, false);
			esp_lcd_panel_mirror(panel_handle, false, false);
			break;
		case LV_DISP_ROT_90:
			// Rotate LCD display
			esp_lcd_panel_swap_xy(panel_handle, true);
			esp_lcd_panel_mirror(panel_handle, false, true);
			break;
		case LV_DISP_ROT_180:
			// Rotate LCD display
			esp_lcd_panel_swap_xy(panel_handle, false);
			esp_lcd_panel_mirror(panel_handle, true, true);
			break;
		case LV_DISP_ROT_270:
			// Rotate LCD display
			esp_lcd_panel_swap_xy(panel_handle, true);
			esp_lcd_panel_mirror(panel_handle, true, false);
			break;
    }
}

static void on_lvgl_tick(void *arg) {
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_routine(void *arg) {
    ESP_LOGI(TAG, "Started LVGL task");

	lv_obj_t *scr = lv_scr_act();

	s_uiInternals.cycles = lv_label_create(scr);
	s_uiInternals.uptime = lv_label_create(scr);
	s_uiInternals.percent = lv_label_create(scr);

	lv_label_set_text(s_uiInternals.cycles, " ...");
	lv_label_set_text(s_uiInternals.uptime, " ...");
	lv_label_set_text(s_uiInternals.percent, " ...");

	lv_obj_align(s_uiInternals.cycles, LV_ALIGN_TOP_LEFT, 0, 0);
	lv_obj_align_to(s_uiInternals.uptime, s_uiInternals.cycles, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
	lv_obj_align_to(s_uiInternals.percent, s_uiInternals.uptime, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

	uint32_t wait4 = 50;
	while (1) {
		if (wait4 > 50)
			wait4 = 50;
		xSemaphoreTake(s_uiInternals.waiter, pdMS_TO_TICKS(wait4));
		if (UiEngine_lock(-1)) {
			wait4 = lv_timer_handler();
			UiEngine_unlock();
		}
	}
}

static esp_lcd_panel_io_handle_t init_lcd_bus(void *user_ctx) {

	spi_bus_config_t buscfg = {
		.sclk_io_num = BOARD_CFG_GPIO_LCD_CLK,
		.mosi_io_num = BOARD_CFG_GPIO_LCD_MOSI,
		.miso_io_num = -1,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = BOARD_CFG_DISPLAY_WIDTH * BOARD_CFG_DISPLAY_HEIGHT / 8,
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

static esp_lcd_panel_handle_t init_lcd_panel(esp_lcd_panel_io_handle_t io_handle, void *user_ctx) {

    static const esp_lcd_ssd1681_config_t epaper_ssd1681_config = {
        .busy_gpio_num = BOARD_CFG_GPIO_LCD_BUSY,
    };
    static const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BOARD_CFG_GPIO_LCD_RST,
        .flags.reset_active_high = false,
        .vendor_config = (esp_lcd_ssd1681_config_t*)&epaper_ssd1681_config
    };

    gpio_install_isr_service(0);

    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1681(io_handle, &panel_config, &panel_handle));

    ESP_LOGI(TAG, "Resetting e-Paper display...");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Initializing e-Paper display...");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Turning e-Paper display on...");
	ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
	vTaskDelay(pdMS_TO_TICKS(100));

	ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
	ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
	esp_lcd_panel_invert_color(panel_handle, false);

    static const epaper_panel_callbacks_t cbs = {
        .on_epaper_refresh_done = ep_flush_ready_cb
    };
    epaper_panel_register_event_callbacks(panel_handle, (epaper_panel_callbacks_t*)&cbs, user_ctx);

    return panel_handle;
}


static int init_lvgl(lv_disp_drv_t *drv, lv_disp_draw_buf_t *fb, lv_indev_drv_t *in, esp_lcd_panel_handle_t panel_handle) {

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    lv_color_t *buf1 = NULL;
    lv_color_t *buf2 = NULL;

//    static const esp_dma_mem_info_t psram_info = {
//        .extra_heap_caps = MALLOC_CAP_SPIRAM,
//        .dma_alignment_bytes = 4, //legacy API behaviour is only check max dma buffer alignment
//    };
//    if (!esp_dma_capable_malloc(FB_SIZE, &psram_info, (void **)&buf1, NULL))
//        ESP_ERROR_CHECK(esp_dma_capable_malloc(FB_SIZE, &psram_info, (void **)&buf2, NULL));
//    else
    {
    	ESP_LOGI(TAG, "PSRAM alloc fail %d bytes", FB_SIZE);
        static const esp_dma_mem_info_t internal = {
            .extra_heap_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA,
            .dma_alignment_bytes = 4, //legacy API behaviour is only check max dma buffer alignment
        };
        ESP_ERROR_CHECK(esp_dma_capable_malloc(FB_SIZE, &internal, (void **)&buf1, NULL));
        ESP_ERROR_CHECK(esp_dma_capable_malloc(FB_SIZE, &internal, (void **)&buf2, NULL));
    }
    assert(buf1);
    assert(buf2);
    ESP_LOGI(TAG, "%d buf1@%p, buf2@%p", FB_SIZE, buf1, buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(fb, buf1, buf2, FB_LENGTH);


    converted_buffer_black = heap_caps_malloc(BOARD_CFG_DISPLAY_WIDTH * BOARD_CFG_DISPLAY_HEIGHT / 8, MALLOC_CAP_DMA);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(drv);
    drv->hor_res = BOARD_CFG_DISPLAY_WIDTH;
    drv->ver_res = BOARD_CFG_DISPLAY_HEIGHT;
    drv->flush_cb = lvgl_flush_cb;
    drv->wait_cb = lvgl_wait_cb;
    drv->drv_update_cb = lvgl_port_update_cb;
    drv->draw_buf = fb;
    drv->user_data = panel_handle;
    //// NOTE: The ssd1681 e-paper is monochrome and 1 byte represents 8 pixels
    //// so full_refresh is MANDATORY because we cannot set position to bitmap at pixel level
    drv->full_refresh = true;

    lv_disp_t *disp = lv_disp_drv_register(drv);
    if (!disp) {
    	free(buf1);
    	free(buf2);
        ESP_LOGE(TAG, "can't register display driver");
        return -1;
    }

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = on_lvgl_tick,
        .name = "lvgl_tick",
		.arg = 0,
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    return 0;
}

bool UiEngine_lock(int timeout_ms) {
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(s_uiInternals.lock, timeout_ticks) == pdTRUE;
}

void UiEngine_unlock(void) {
    xSemaphoreGiveRecursive(s_uiInternals.lock);
}


int UiEngine_SetCycles(uint32_t cyc) {
	UiEngine_lock(-1);
	s_uiInternals.partialRfr = false;

	lv_label_set_text_fmt(s_uiInternals.cycles, "%ld", cyc);
	UiEngine_unlock();
	return 0;
}

static void set_uptime(uint32_t sec) {
	const int sc = sec % 60;
	const int m = (sec / 60) % 60;
	const int h = sec / 3600;

    ESP_LOGD(TAG, "CUtime %ld %d %d %d %d %d", sec, sc, m, h);
    if (!h)
    	lv_label_set_text_fmt(s_uiInternals.uptime, "%02dm %02ds", m, sc);
    else
    	lv_label_set_text_fmt(s_uiInternals.uptime, "%6d", h);
}

int UiEngine_SetUptime(uint32_t sec) {
	UiEngine_lock(-1);
	set_uptime(sec);
	s_uiInternals.partialRfr = false;
	UiEngine_unlock();
	return 0;
}

int UiEngine_SetPercent(uint32_t prc, uint32_t sec) {
	if (!UiEngine_lock(100)) {
		ESP_LOGE(TAG, "UI locked");
		return 0;
	}
	s_uiInternals.partialRfr = 0;
	lv_label_set_text_fmt(s_uiInternals.percent, "%02ld%%", prc);
	set_uptime(sec);
	UiEngine_unlock();
	return 0;
}

void UiEngine_forceredraw(void) {
	s_uiInternals.partialRfr = false;
	xSemaphoreGive(s_uiInternals.waiter);
}


int UiEngine_init(void) {

	if (s_uiInternals.tid)
		return 0;

	s_uiInternals.lock = xSemaphoreCreateRecursiveMutex();
	s_uiInternals.panelBusy = xSemaphoreCreateBinary();
	s_uiInternals.waiter = xSemaphoreCreateBinary();
	xSemaphoreGive(s_uiInternals.panelBusy);

    esp_lcd_panel_io_handle_t io_handle = init_lcd_bus(&s_uiInternals.disp_drv);
    esp_lcd_panel_handle_t panel_handle = init_lcd_panel(io_handle, &s_uiInternals.disp_drv);

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


	int rv = init_lvgl(&s_uiInternals.disp_drv, &s_uiInternals.disp_buf, &s_uiInternals.indev_drv, panel_handle);
	if (rv) {
		ESP_LOGE(TAG, "Failed to init lvgl %d", rv);
		return rv;

	}

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(lvgl_routine, "LVGL", 4 * 1024, NULL, 2, &s_uiInternals.tid);
    return 0;
}
#endif
