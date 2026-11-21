/*
 * uiEngine.cpp
 *
 *  Created on: 12 Jan 2024
 *      Author: pavloha
 *
 *  Application screen on top of TftUI (esp_tools/UItft). Keeps the old C API
 *  from someGeneric.hpp so callers do not change.
 */

#include "board_cfg.h"
#include "someGeneric.hpp"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
static const char *TAG = "UiEngine";

#ifndef BOARD_CFG_DISPLAY

int UiEngine_init(void) { return 0; }
bool UiEngine_lock(int timeout_ms) { return true; }
void UiEngine_unlock(void) {}
int UiEngine_SetData(uint32_t cyc, uint32_t mv, uint32_t prc, uint32_t sec) { return 0; }
int UiEngine_SetText(const char *text) { ESP_LOGI(TAG, "UI: '%s'", text); return 0; }

#else

#include "UItft.hpp"
#include <new>

#define UI_TASK_PRIO   2
#define UI_TASK_STACK  (6 * 1024)

namespace {

class CompressorUI final: public TftUI {
public:
	using TftUI::TftUI;

	// callers hold UI lock
	void setData(uint32_t cyc, uint32_t mv, uint32_t prc, uint32_t sec) {
		cyc_ = cyc; mv_ = mv; prc_ = prc; sec_ = sec;
		data_dirty_ = true;
	}
	void setText(const char *text) {
		snprintf(text_, sizeof(text_), "%s", text);
		text_dirty_ = true;
	}

protected:
	// UI thread. Widgets are created here: display exists only after run() started
	void draw() noexcept override {
		lock();
		if (!cycles_)
			create();
		if (data_dirty_)
			apply_data();
		if (text_dirty_) {
			lv_label_set_text(percent_, text_);
			text_dirty_ = false;
		}
		unlock();
	}

private:
	void create() {
		lv_obj_t *scr = lv_display_get_screen_active(display());

		cycles_ = lv_label_create(scr);
		uptime_ = lv_label_create(scr);
		percent_ = lv_label_create(scr);

		lv_label_set_text(cycles_, "...");
		lv_label_set_text(uptime_, "...");
		lv_label_set_text(percent_, "...");

		lv_obj_align(cycles_, LV_ALIGN_TOP_LEFT, 0, 0);
		lv_obj_align_to(uptime_, cycles_, LV_ALIGN_OUT_BOTTOM_MID, 0, 3);
		lv_obj_align_to(percent_, uptime_, LV_ALIGN_OUT_BOTTOM_MID, 0, 3);
	}

	void apply_data() {
		const unsigned sc = sec_ % 60;
		const unsigned m = (sec_ / 60) % 60;
		const unsigned h = sec_ / 3600;

		lv_label_set_text_fmt(cycles_, "Cycles %6lu.  BAT %2lu.%02luV %02lus",
				(unsigned long)cyc_, (unsigned long)(mv_ / 1000), (unsigned long)((mv_ / 10) % 100), (unsigned long)prc_);
		if (!h)
			lv_label_set_text_fmt(uptime_, "MotoMinutes %2u:%2u", m, sc);
		else
			lv_label_set_text_fmt(uptime_, "MotoHrs: %u (min %2u:%2u)", h, m, sc);
		data_dirty_ = false;
	}

	lv_obj_t *cycles_ = nullptr;
	lv_obj_t *uptime_ = nullptr;
	lv_obj_t *percent_ = nullptr;

	uint32_t cyc_ = 0, mv_ = 0, prc_ = 0, sec_ = 0;
	bool data_dirty_ = false;
	char text_[64] = "...";
	bool text_dirty_ = false;
};

CompressorUI *s_ui;

TftUI::Config board_config() {
	TftUI::Config cfg;
	cfg.host = SPI2_HOST;
	cfg.gpio_sclk = BOARD_CFG_GPIO_LCD_CLK;
	cfg.gpio_mosi = BOARD_CFG_GPIO_LCD_MOSI;
	cfg.gpio_cs = BOARD_CFG_GPIO_LCD_CS;
	cfg.gpio_dc = BOARD_CFG_GPIO_LCD_DC;
	cfg.gpio_rst = BOARD_CFG_GPIO_LCD_RST;
	cfg.gpio_bl = BOARD_CFG_GPIO_LCD_BUSY;
	cfg.bl_on_level = 0;
	cfg.pclk_hz = 20 * 1000 * 1000;

	cfg.hres = BOARD_CFG_DISPLAY_WIDTH;
	cfg.vres = BOARD_CFG_DISPLAY_HEIGHT;
	cfg.swap_xy = true;
	cfg.mirror_x = false;
	cfg.mirror_y = true;
	cfg.invert = false;
	cfg.gap_x = 18;
	cfg.gap_y = 82;

	cfg.fb_lines = BOARD_CFG_DISPLAY_HEIGHT / 4;
	cfg.keepalive_ms = 2000;
	cfg.probe_sio = false;
	return cfg;
}

} // namespace

bool UiEngine_lock(int timeout_ms) {
	if (!s_ui)
		return false;
	if (timeout_ms < 0) {
		s_ui->lock();
		return true;
	}
	const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
	while (!s_ui->try_lock()) {
		if (int32_t(xTaskGetTickCount() - deadline) >= 0)
			return false;
		vTaskDelay(1);
	}
	return true;
}

void UiEngine_unlock(void) {
	if (s_ui)
		s_ui->unlock();
}

int UiEngine_SetData(uint32_t cyc, uint32_t mv, uint32_t prc, uint32_t sec) {
	if (!UiEngine_lock(200)) {
		ESP_LOGE(TAG, "UI locked");
		return 0;
	}
	s_ui->setData(cyc, mv, prc, sec);
	UiEngine_unlock();
	return 0;
}

int UiEngine_SetText(const char *text) {
	if (!UiEngine_lock(200)) {
		ESP_LOGE(TAG, "UI locked");
		return 0;
	}
	s_ui->setText(text);
	UiEngine_unlock();
	ESP_LOGI(TAG, "UI: '%s'", text);
	return 0;
}

int UiEngine_init(void) {
	if (s_ui)
		return 0;

	CompressorUI *ui = new (std::nothrow) CompressorUI(board_config());
	if (!ui)
		return -1;

	if (!ui->start(UI_TASK_PRIO, UI_TASK_STACK)) {
		ESP_LOGE(TAG, "UI task not started");
		delete ui;
		return -1;
	}
	s_ui = ui;
	return 0;
}

#endif
