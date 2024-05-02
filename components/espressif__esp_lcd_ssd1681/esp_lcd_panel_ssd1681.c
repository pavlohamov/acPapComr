/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#if CONFIG_LCD_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif
#include "esp_log.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_ssd1681.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_ssd1681_commands.h"

#include "esp_timer.h"

//#define SSD1681_LUT_SIZE                   159

#define SSD_X        122
#define SSD_Y        250

#define SSD_MAX_X 176
#define SSD_MAX_Y 296

static const char *TAG = "lcd_panel.epaper";

typedef struct {
    esp_lcd_epaper_panel_cb_t cb;
    void *args;
} epaper_panel_callback_t;

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    // --- Normal configurations
    // Configurations from panel_dev_config
    int reset_gpio_num;
    bool reset_level;
    // Configurations from epaper_ssd1681_conf
    int busy_gpio_num;
    bool full_refresh;
    // Configurations from interface functions
    int gap_x;
    int gap_y;
    // Configurations from e-Paper specific public functions
    epaper_panel_callback_t refresh_done_isr_cb;
    esp_lcd_ssd1681_bitmap_color_t bitmap_color;
    bool _mirror_y;
    bool _swap_xy;
    // --- Other private fields
    bool _mirror_x;
    bool _invert_color;
} epaper_panel_t;

static esp_err_t poll_busy(esp_lcd_panel_t *panel);

static void onBusyIsr(void *arg);

static esp_err_t epaper_set_lut(esp_lcd_panel_io_handle_t io, const uint8_t *lut);
static esp_err_t epaper_set_cursor(esp_lcd_panel_io_handle_t io, uint32_t cur_x, uint32_t cur_y);
static esp_err_t epaper_set_area(esp_lcd_panel_io_handle_t io, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
static esp_err_t pap_set_vram(esp_lcd_panel_io_handle_t io, const uint8_t *bw, const uint8_t *r, size_t size);

static esp_err_t epaper_panel_del(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_reset(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_init(esp_lcd_panel_t *panel);
static esp_err_t draw_bmp(esp_lcd_panel_t *panel, int sx, int sy, int ex, int ey, const void *fb);
static esp_err_t epaper_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t epaper_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t epaper_panel_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t epaper_panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t epaper_panel_disp_on_off(esp_lcd_panel_t *panel, bool on_off);


static void onBusyIsr(void *arg) {
    epaper_panel_t *p = arg;
    epaper_panel_callback_t *cb = &p->refresh_done_isr_cb;

    gpio_intr_disable(p->busy_gpio_num);
    if (cb->cb)
    	cb->cb(&p->base, NULL, cb->args);
}

esp_err_t epaper_panel_register_event_callbacks(esp_lcd_panel_t *panel, epaper_panel_callbacks_t *cbs, void *user_ctx) {
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    ESP_RETURN_ON_FALSE(cbs, ESP_ERR_INVALID_ARG, TAG, "cbs is NULL");
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);
    pap->refresh_done_isr_cb.cb = cbs->on_epaper_refresh_done;
    pap->refresh_done_isr_cb.args = user_ctx;
    return ESP_OK;
}

//esp_err_t epaper_panel_set_custom_lut(esp_lcd_panel_t *panel, uint8_t *lut, size_t size) {
//    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
//    ESP_RETURN_ON_FALSE(lut, ESP_ERR_INVALID_ARG, TAG, "lut is NULL");
//    ESP_RETURN_ON_FALSE(size == SSD1681_LUT_SIZE, ESP_ERR_INVALID_ARG, TAG, "Invalid lut size");
//    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
//    epaper_set_lut(epaper_panel->io, lut);
//    return ESP_OK;
//}
//
//static esp_err_t epaper_set_lut(esp_lcd_panel_io_handle_t io, const uint8_t *lut) {
//    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io,
//    		SSD1681_CMD_SET_LUT_REG, lut, 153), TAG, "SSD1681_CMD_SET_LUT_REG err");
//
//    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io,
//    		SSD1681_CMD_SET_DISP_UPDATE_CTRL, (uint8_t[]) { lut[153] }, 1), TAG, "SSD1681_CMD_SET_DISP_UPDATE_CTRL err");
//
//    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io,
//    		SSD1681_CMD_SET_GATE_DRIVING_VOLTAGE, (uint8_t[]) { lut[154] }, 1), TAG, "SSD1681_CMD_SET_GATE_DRIVING_VOLTAGE err");
//
//    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io,
//    		SSD1681_CMD_SET_SRC_DRIVING_VOLTAGE, (uint8_t[]) { lut[155], lut[156], lut[157] }, 3), TAG, "SSD1681_CMD_SET_SRC_DRIVING_VOLTAGE err");
//
//    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io,
//    		SSD1681_CMD_SET_VCOM_REG, (uint8_t[]) { lut[158] }, 1), TAG, "SSD1681_CMD_SET_VCOM_REG err");
//
//    return ESP_OK;
//}

static esp_err_t epaper_set_cursor(esp_lcd_panel_io_handle_t io, uint32_t cur_x, uint32_t cur_y) {
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io,
    		SSD1681_CMD_SET_INIT_X_ADDR_COUNTER, (uint8_t[]) { cur_x & 0xff }, 1), TAG, "SSD1681_CMD_SET_INIT_X_ADDR_COUNTER err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io,
    		SSD1681_CMD_SET_INIT_Y_ADDR_COUNTER, (uint8_t[]) { (cur_y & 0xff), (cur_y >> 8) & 0xff }, 2), TAG, "SSD1681_CMD_SET_INIT_Y_ADDR_COUNTER err");

    return ESP_OK;
}

static esp_err_t epaper_set_area(esp_lcd_panel_io_handle_t io, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y) {

#if 0
	const int ex = end_x / 8 + !!(end_x % 8);
	const uint8_t x[] = {
			(start_x / 8) & 0xff,
			ex & 0xff
	};
	const uint8_t y[] = {
			start_y & 0xff,
			(start_y  >> 8) & 0xff,
			end_y & 0xff,
			(end_y  >> 8) & 0xff,
	};
#else
	const uint8_t x[] = {
			start_x,
			end_x
	};
	const uint8_t y[] = {
			start_y & 0xff,
			(start_y  >> 8) & 0xff,
			end_y & 0xff,
			(end_y  >> 8) & 0xff,
	};
#endif

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_RAMX_START_END_POS, x, sizeof(x)), TAG, "SSD1681_CMD_SET_RAMX_START_END_POS err");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_RAMY_START_END_POS, y, sizeof(y)), TAG, "SSD1681_CMD_SET_RAMY_START_END_POS err");

    return ESP_OK;
}

static esp_err_t poll_busy(esp_lcd_panel_t *panel) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    const int64_t max = esp_timer_get_time() + 3000e3;
    int tout = 0;
    while (!tout && gpio_get_level(epaper_panel->busy_gpio_num)) {
        vTaskDelay(pdMS_TO_TICKS(15));
        tout = esp_timer_get_time() > max;
    }
    if (tout) {
        ESP_LOGI(TAG, "wait timeout");
        return -1;
    }
    return ESP_OK;
}

static esp_err_t pap_set_vram(esp_lcd_panel_io_handle_t io, const uint8_t *bw, const uint8_t *r, size_t size) {
	if (!size)
		return ESP_OK;

	if (bw)
		ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, SSD1681_CMD_WRITE_BLACK_VRAM, bw, size), TAG, "data bw_bitmap err");
	if (r)
		ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, SSD1681_CMD_WRITE_RED_VRAM, r, size), TAG, "data red_bitmap err");

	return ESP_OK;
}

static esp_err_t pap_refresh(esp_lcd_panel_t *panel, int full) {

    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);

    uint8_t duc_flag = pap->_invert_color ? SSD1681_PARAM_COLOR_RW_INVERSE_BIT : SSD1681_PARAM_COLOR_BW_INVERSE_BIT;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(pap->io,
    		SSD1681_CMD_DISP_UPDATE_CTRL, (uint8_t[]) { duc_flag, 0x00 }, 2), TAG, "SSD1681_CMD_DISP_UPDATE_CTRL err");

    uint8_t update_cmd = full ? 0xF7 : 0xFF;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(pap->io,
    		SSD1681_CMD_SET_DISP_UPDATE_CTRL, (uint8_t[]) { update_cmd }, 1), TAG, "SSD1681_CMD_SET_DISP_UPDATE_CTRL err");

    gpio_intr_enable(pap->busy_gpio_num);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(pap->io, SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0), TAG,
                        "SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ err");

    return ESP_OK;
}

esp_err_t epaper_panel_refresh_screen(esp_lcd_panel_t *panel) {
    return pap_refresh(panel, 1);
}


esp_err_t epaper_panel_refresh_partial(esp_lcd_panel_t *panel) {
    return pap_refresh(panel, 0);
}

static esp_err_t epaper_panel_reset(esp_lcd_panel_t *panel) {
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = pap->io;

    if (pap->reset_gpio_num >= 0) {
        ESP_RETURN_ON_ERROR(gpio_set_level(pap->reset_gpio_num, pap->reset_level), TAG, "gpio_set_level error");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(gpio_set_level(pap->reset_gpio_num, !pap->reset_level), TAG, "gpio_set_level error");
        vTaskDelay(pdMS_TO_TICKS(10));
    } else
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SWRST, NULL, 0), TAG, "param SSD1681_CMD_SWRST err");

    poll_busy(panel);
    return ESP_OK;
}

esp_err_t epaper_panel_set_bitmap_color(esp_lcd_panel_t *panel, esp_lcd_ssd1681_bitmap_color_t color) {
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->bitmap_color = color;
    return ESP_OK;
}


static int setWindow(epaper_panel_t *pap, int x, int y, int h, int w) {

#if 01
//	y += 8;
   // int mode = SSD16XX_DATA_ENTRY_XIYDY;
	int mode = SSD16XX_DATA_ENTRY_XIYIX;
	const int x_start = 1 + y / SSD16XX_PIXELS_PER_BYTE;
	const int x_end = (y + h) / SSD16XX_PIXELS_PER_BYTE;
	const int y_start = x;
	const int y_end = (x + w);
#else
    int mode = SSD16XX_DATA_ENTRY_XDYIY;
    const int panel_h = 122 - 122 % 8;
    const int x_start = (panel_h - 1 - y) / SSD16XX_PIXELS_PER_BYTE;
    const int x_end = (panel_h - 1 - (y + 122 - 1)) / SSD16XX_PIXELS_PER_BYTE;
    const int y_start = x;
    const int y_end = (x + 250 - 1);
#endif

//    ESP_LOGW(TAG, "%d %d", x, y);
//    ESP_LOGE(TAG, "%d %d %d %d", x_start, x_end, y_start, y_end);

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(pap->io, SSD1681_CMD_DATA_ENTRY_MODE, (uint8_t[]) { mode }, 1), TAG, "");

    ESP_RETURN_ON_ERROR(epaper_set_area(pap->io, x_start, y_start, x_end, y_end), TAG, "epaper_set_area() error");
    ESP_RETURN_ON_ERROR(epaper_set_cursor(pap->io, x_start, y_start), TAG, "epaper_set_cursor() error");

    return 0;
}


static int super_test(esp_lcd_panel_t *panel) {

    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);

    const int line_sz = SSD_X / 8 + !!(SSD_X & 0x7);
    const int bs = line_sz * SSD_Y;

    uint8_t *buff = heap_caps_malloc(bs, MALLOC_CAP_DMA);

    ESP_ERROR_CHECK(epaper_panel_set_bitmap_color(panel, SSD1681_EPAPER_BITMAP_BLACK));

    const int testLen = SSD_Y;

    epaper_panel_set_gap(panel, 0, 0);


    int x = 0;
    int y = 0;

    int full = 1;
	while (1) {

	    ESP_LOGE(TAG, "loop");
	    int pos = 0;
	    while ((pos + testLen) <= bs) {

	    	setWindow(pap, 0, 0, line_sz * 8, SSD_Y);
			memset(buff, 0x0, bs);
			ESP_RETURN_ON_ERROR(pap_set_vram(pap->io, buff, buff, bs), TAG, "panel_epaper_set_vram error");

	    	setWindow(pap, x, y, 8, testLen);
	    	x += SSD_Y / 5;
	    	if (x >= SSD_Y) {
	    		x = 0;
	    		y += 8;
				if (y > SSD_X)
					y = 0;
	    	}
		    ESP_LOGI(TAG, "%d/%d", pos, testLen);
			memset(buff, 0xff, testLen);
			pos += testLen;

			ESP_RETURN_ON_ERROR(pap_set_vram(pap->io, buff, buff, testLen), TAG, "panel_epaper_set_vram error");
			pap_refresh(panel, full);
			full = 0;
			poll_busy(panel);

			vTaskDelay(pdMS_TO_TICKS(2));
	    }
	}

}

static esp_err_t draw_bmp(esp_lcd_panel_t *panel, int sx, int sy, int ex, int ey, const void *fb) {
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);
    if (gpio_get_level(pap->busy_gpio_num))
        return ESP_ERR_NOT_FINISHED;

    ESP_RETURN_ON_FALSE(fb, ESP_ERR_INVALID_ARG, TAG, "bitmap is null");
    ESP_RETURN_ON_FALSE((sx < ex) && (sy < ey), ESP_ERR_INVALID_ARG, TAG, "start position must be smaller than end position");

    const int w = abs(sx - ex);
    const int h = abs(sy - ey);
    int buffer_size = w * h;
    buffer_size = buffer_size / 8 ;//+ !!(buffer_size % 8);

	if (!esp_ptr_dma_capable(fb)) {
		ESP_LOGW(TAG, "Bitmap not DMA capable, use DMA capable memory to avoid additional data copy.");
	}

    sx += pap->gap_x;
    sy += pap->gap_y;

//	ESP_LOGW(TAG, " %d %d %d %d", sx, sy, ex, ey);
//	ESP_LOGW(TAG, " %d %d %d %d", sx, sy, w, h);
	setWindow(pap, sx, sy, w, h);

    const uint8_t *bw = (pap->bitmap_color == SSD1681_EPAPER_BITMAP_BLACK) ? fb : NULL;
    const uint8_t *red = (pap->bitmap_color == SSD1681_EPAPER_BITMAP_RED) ? fb : NULL;

    ESP_RETURN_ON_ERROR(pap_set_vram(pap->io, bw, red, buffer_size), TAG, "panel_epaper_set_vram error");

    return ESP_OK;
}

static esp_err_t epaper_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data) {
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);
    pap->_invert_color = invert_color_data;
    return ESP_OK;
}

static esp_err_t epaper_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y) {
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);
    pap->_mirror_x = mirror_x;
    pap->_mirror_y = mirror_y;
    return ESP_OK;
}

static esp_err_t epaper_panel_swap_xy(esp_lcd_panel_t *panel, bool val) {
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);
    pap->_swap_xy = val;
    return ESP_OK;
}

static esp_err_t epaper_panel_set_gap(esp_lcd_panel_t *panel, int x, int y) {
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);
    pap->gap_x = x;
    pap->gap_y = y;
    return ESP_OK;
}

static esp_err_t epaper_panel_disp_on_off(esp_lcd_panel_t *panel, bool isOn) {
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = pap->io;
    if (isOn) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io,
        		SSD1681_CMD_SET_DISP_UPDATE_CTRL, (uint8_t[]) { SSD1681_PARAM_DISP_UPDATE_MODE_1 }, 1), TAG, "SSD1681_CMD_SET_DISP_UPDATE_CTRL err");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0), TAG,
                            "SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ err");
        poll_busy(panel);
    } else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io,
        		SSD1681_CMD_SLEEP_CTRL, (uint8_t[]) { SSD1681_PARAM_SLEEP_MODE_1 }, 1), TAG, "SSD1681_CMD_SLEEP_CTRL err");
        // BUSY pin will stay HIGH, so do not call poll_busy() here
    }

    return ESP_OK;
}

esp_err_t esp_lcd_new_panel_ssd1681(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *const pcfg,
                          esp_lcd_panel_handle_t *const ret_panel) {
	esp_err_t ret = 0;
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    ESP_RETURN_ON_FALSE(io && pcfg && ret_panel, ESP_ERR_INVALID_ARG, TAG, "1 or more args is NULL");
    esp_lcd_ssd1681_config_t *cfg = pcfg->vendor_config;

    epaper_panel_t *pap = calloc(1, sizeof(epaper_panel_t));
    ESP_GOTO_ON_FALSE(pap, ESP_ERR_NO_MEM, err, TAG, "no mem for epaper panel");


    pap->_invert_color = false;
    pap->_swap_xy = false;
    pap->_mirror_x = false;
    pap->_mirror_y = false;
    pap->gap_x = 0;
    pap->gap_y = 0;
    pap->bitmap_color = SSD1681_EPAPER_BITMAP_BLACK;
    pap->full_refresh = true;
    // configurations
    pap->io = io;
    pap->reset_gpio_num = pcfg->reset_gpio_num;
    pap->busy_gpio_num = cfg->busy_gpio_num;
    pap->reset_level = pcfg->flags.reset_active_high;
    // functions
    pap->base.del = epaper_panel_del;
    pap->base.reset = epaper_panel_reset;
    pap->base.init = epaper_panel_init;
    pap->base.draw_bitmap = draw_bmp;
    pap->base.invert_color = epaper_panel_invert_color;
    pap->base.set_gap = epaper_panel_set_gap;
    pap->base.mirror = epaper_panel_mirror;
    pap->base.swap_xy = epaper_panel_swap_xy;
    pap->base.disp_on_off = epaper_panel_disp_on_off;
    *ret_panel = &pap->base;

    if (pap->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << pcfg->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line err");
    }

    if (pap->busy_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = 0x01,
            .pin_bit_mask = 1ULL << pap->busy_gpio_num,
        };
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
        ESP_LOGI(TAG, "Add handler for GPIO %d", pap->busy_gpio_num);
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for BUSY line err");
        ESP_GOTO_ON_ERROR(gpio_isr_handler_add(pap->busy_gpio_num, onBusyIsr, pap), err, TAG, "configure GPIO for BUSY line err");
        // Enable GPIO intr only before refreshing, to avoid other commands caused intr trigger
        gpio_intr_disable(pap->busy_gpio_num);
    }
    ESP_LOGD(TAG, "new epaper panel @%p", pap);
    return ESP_OK;

err:
    if (pap) {
        if (pcfg->reset_gpio_num >= 0)
            gpio_reset_pin(pcfg->reset_gpio_num);
        if (cfg->busy_gpio_num >= 0)
            gpio_reset_pin(cfg->busy_gpio_num);
        free(pap);
    }
    return ret;
}

static esp_err_t epaper_panel_del(esp_lcd_panel_t *panel) {
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);

    if ((pap->reset_gpio_num) >= 0)
        gpio_reset_pin(pap->reset_gpio_num);

    if (pap->busy_gpio_num >= 0)
    	gpio_reset_pin(pap->busy_gpio_num);


    ESP_LOGD(TAG, "del ssd1681 epaper panel @%p", pap);
    free(pap);
    return ESP_OK;
}


static esp_err_t epaper_panel_init(esp_lcd_panel_t *panel) {
    epaper_panel_t *pap = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = pap->io;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SWRST, NULL, 0), TAG, "param SSD1681_CMD_SWRST err");
    poll_busy(panel);

    static const uint8_t output[] = {
    		(SSD_Y - 1) & 0xFF,
			SSD_Y >> 8,
			0x0,
    };

    static const uint8_t x[] = {
    		0,
			(SSD_X - 1) / 8,
    };

    static const uint8_t y[] = {
    		0, 0,
    		output[0], output[1],
    };

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_OUTPUT_CTRL, output, sizeof(output)), TAG, "SSD1681_CMD_OUTPUT_CTRL err");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_RAMX_START_END_POS, x, sizeof(x)), TAG, "SET_RAMX_START_END_POS err");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_RAMY_START_END_POS, y, sizeof(y)), TAG, "SET_RAMY_START_END_POS err");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_BORDER_WAVEFORM, (uint8_t[]) { 0x05 }, 1), TAG, "SSD1681_CMD_SET_BORDER_WAVEFORM err");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_TEMP_SENSOR, (uint8_t[]) { 0x80 }, 1), TAG, "SSD1681_CMD_SET_TEMP_SENSOR err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_DISP_UPDATE_CTRL, (uint8_t[]) { SSD1681_PARAM_DISP_UPDATE_MODE_1 }, 1), TAG, "SSD1681_CMD_SET_DISP_UPDATE_CTRL err");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0), TAG, "param SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ err");
    poll_busy(panel);

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_END_OPTION, (uint8_t[]) { SSD1681_PARAM_END_OPTION_KEEP }, 1), TAG, "SSD1681_CMD_SET_END_OPTION err");

//    super_test(panel);
    return ESP_OK;
}
