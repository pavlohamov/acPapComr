/*
 * uiEngine.cpp
 *
 *  Created on: 12 Jan 2024
 *      Author: pavloha
 *
 *  Application screen on top of TftUI (esp_tools/UItft). Keeps the old C API
 *  from someGeneric.hpp so callers do not change.
 *
 *  Layout ("variant B"), 284x76:
 *   +--+------------------------------------------+-----------+
 *   |##| <State>   <countdown>                    |  Supply   |  top band 42px
 *   |##|                                          |  12.4 V   |
 *   +--+-----------------+----------------+-------+-----------+
 *   |  Motor hours       | Cycles         | Uptime            |  bottom band 34px
 *   |  1:42:46           | 179            | 0:12:05           |
 *   +--------------------+----------------+-------------------+
 */

#include "board_cfg.h"
#include "someGeneric.hpp"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "esp_log.h"
static const char *TAG = "UiEngine";

#ifndef BOARD_CFG_DISPLAY

int UiEngine_init(void) { return 0; }
bool UiEngine_lock(int timeout_ms) { return true; }
void UiEngine_unlock(void) {}
int UiEngine_SetData(uint32_t cyc, uint32_t mv, uint32_t prc, uint32_t sec) { return 0; }
int UiEngine_SetState(UiState_t st, uint32_t left_ms, uint32_t total_ms) { return 0; }
int UiEngine_SetText(const char *text) { ESP_LOGI(TAG, "UI: '%s'", text); return 0; }

#else

#include "UItft.hpp"
#include <new>

#define UI_TASK_PRIO   2
#define UI_TASK_STACK  (6 * 1024)

// fonts: enable in menuconfig (LVGL -> Font usage), falls back to default font
#if defined(CONFIG_LV_FONT_MONTSERRAT_24)
#define FONT_BIG   (&lv_font_montserrat_24)
#else
#define FONT_BIG   LV_FONT_DEFAULT
#endif
#if defined(CONFIG_LV_FONT_MONTSERRAT_12)
#define FONT_SMALL (&lv_font_montserrat_12)
#else
#define FONT_SMALL LV_FONT_DEFAULT
#endif
#if defined(CONFIG_LV_FONT_MONTSERRAT_16)
#define FONT_VALUE (&lv_font_montserrat_16)
#else
#define FONT_VALUE LV_FONT_DEFAULT
#endif

namespace {

// palette tuned for TN panels: only "pure" channel values (0x00/0xFF).
// Mid-greys and dark tints shift hue / invert with viewing angle on TN,
// saturated primaries and black/white stay readable.
const uint32_t C_BG        = 0x000000;
const uint32_t C_BAND      = 0x000000;   // bands separated by a line, not by tint
const uint32_t C_DIVIDER   = 0x00FFFF;
const uint32_t C_TEXT      = 0xFFFFFF;
const uint32_t C_LABEL     = 0x00FFFF;   // hierarchy by size, color only marks "caption"
const uint32_t C_DANGER    = 0xFF0000;

struct StateStyle {
	const char *name;
	uint32_t accent;   // left bar
	uint32_t timer;    // countdown color
};

// order must match UiState_t
const StateStyle kStates[] = {
	/* UI_STATE_BOOT     */ { "Booting",  0xFFFFFF, 0xFFFFFF },
	/* UI_STATE_STARTING */ { "Starting", 0xFFFF00, 0xFFFF00 },
	/* UI_STATE_IDLE     */ { "Idle",     0xFFFFFF, 0xFFFFFF },
	/* UI_STATE_PURGE_1  */ { "Purge 1",  0x00FF00, 0x00FF00 },
	/* UI_STATE_PAUSE    */ { "Pause",    0x00FF00, 0x00FF00 },
	/* UI_STATE_PURGE_2  */ { "Purge 2",  0x00FF00, 0x00FF00 },
	/* UI_STATE_FAULT    */ { "Fault",    0xFF0000, 0xFF0000 },
};
static_assert(sizeof(kStates) / sizeof(*kStates) == UI_STATE_COUNT, "kStates vs UiState_t");

const int TOP_H = 42;
const int BOT_H = 76 - TOP_H;
const int COL_W = 94;

// lv_label_set_text() always invalidates; skip identical text to keep SPI quiet
template<size_t N>
void set_text(lv_obj_t *lbl, char (&cache)[N], const char *text) {
	if (!strncmp(cache, text, N))
		return;
	strlcpy(cache, text, N);
	lv_label_set_text(lbl, cache);
}

lv_obj_t *box(lv_obj_t *parent, int x, int y, int w, int h, uint32_t color) {
	lv_obj_t *o = lv_obj_create(parent);
	lv_obj_remove_style_all(o);
	lv_obj_set_pos(o, x, y);
	lv_obj_set_size(o, w, h);
	lv_obj_set_style_bg_color(o, lv_color_hex(color), 0);
	lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
	lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
	return o;
}

lv_obj_t *label(lv_obj_t *parent, const lv_font_t *font, uint32_t color, const char *text) {
	lv_obj_t *l = lv_label_create(parent);
	lv_obj_set_style_text_font(l, font, 0);
	lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
	lv_label_set_text(l, text);
	return l;
}

void fmt_hms(char *buf, size_t sz, uint32_t sec) {
	const unsigned h = sec / 3600;
	if (h >= 100)
		snprintf(buf, sz, "%u h", h);
	else
		snprintf(buf, sz, "%u:%02u:%02u", h, unsigned(sec / 60 % 60), unsigned(sec % 60));
}

void fmt_countdown(char *buf, size_t sz, uint32_t left_ms, uint32_t total_ms) {
	if (total_ms && total_ms < 60 * 1000) // short phases: tenths of second
		snprintf(buf, sz, "%u.%u s", unsigned(left_ms / 1000), unsigned(left_ms / 100 % 10));
	else {
		const uint32_t s = (left_ms + 999) / 1000;
		snprintf(buf, sz, "%02u:%02u", unsigned(s / 60), unsigned(s % 60));
	}
}

class CompressorUI final: public TftUI {
public:
	using TftUI::TftUI;

	// callers hold UI lock
	void setData(uint32_t cyc, uint32_t mv, uint32_t sec) {
		cyc_ = cyc; mv_ = mv; motor_s_ = sec;
	}
	void setState(UiState_t st, uint32_t left_ms, uint32_t total_ms) {
		if (unsigned(st) >= sizeof(kStates) / sizeof(*kStates))
			st = UI_STATE_FAULT;
		state_ = st; left_ms_ = left_ms; total_ms_ = total_ms;
	}
	void setText(const char *text) {
		strlcpy(text_, text, sizeof(text_));
	}

protected:
	// UI thread, every loop. Widgets are created here: display exists only after run() started
	void draw() noexcept override {
		lock();
		if (!state_lbl_)
			create();
		apply();
		unlock();
	}

private:
	void create() {
		lv_obj_t *scr = lv_display_get_screen_active(display());
		lv_obj_set_style_bg_color(scr, lv_color_hex(C_BG), 0);
		lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
		lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

		// top band: accent bar + "State  countdown" row
		bar_ = box(scr, 0, 0, 6, TOP_H, kStates[UI_STATE_BOOT].accent);

		lv_obj_t *row = box(scr, 6, 0, 198, TOP_H, C_BG);
		lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
		lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
		lv_obj_set_style_pad_left(row, 8, 0);
		lv_obj_set_style_pad_column(row, 10, 0);
		state_lbl_ = label(row, FONT_BIG, C_TEXT, "");
		timer_lbl_ = label(row, FONT_BIG, kStates[UI_STATE_BOOT].timer, "");

		lv_obj_t *l = label(scr, FONT_SMALL, C_LABEL, "Supply");
		lv_obj_align(l, LV_ALIGN_TOP_RIGHT, -8, 3);
		supply_lbl_ = label(scr, FONT_VALUE, C_TEXT, "--.- V");
		lv_obj_align(supply_lbl_, LV_ALIGN_TOP_RIGHT, -8, 18);

		// bottom band: 3 metric columns
		lv_obj_t *band = box(scr, 0, TOP_H, 284, BOT_H, C_BAND);
		box(scr, 0, TOP_H, 284, 1, C_DIVIDER);
		static const char *const titles[] = { "Motor hours", "Cycles", "Uptime" };
		for (int i = 0; i < 3; ++i) {
			const int x = 8 + i * COL_W;
			if (i)
				box(band, i * COL_W, 6, 1, BOT_H - 12, C_DIVIDER);
			l = label(band, FONT_SMALL, C_LABEL, titles[i]);
			lv_obj_set_pos(l, x, 1);
			metric_lbl_[i] = label(band, FONT_VALUE, C_TEXT, "-");
			lv_obj_set_pos(metric_lbl_[i], x, 14);
		}
	}

	void apply() {
		char buf[32];
		const StateStyle &st = kStates[state_];

		// state + accent; on boot show the free text from UiEngine_SetText()
		set_text(state_lbl_, state_cache_, (state_ == UI_STATE_BOOT && text_[0]) ? text_ : st.name);
		if (state_ != shown_state_) {
			shown_state_ = state_;
			lv_obj_set_style_bg_color(bar_, lv_color_hex(st.accent), 0);
			lv_obj_set_style_text_color(state_lbl_, lv_color_hex(state_ == UI_STATE_FAULT ? C_DANGER : C_TEXT), 0);
			lv_obj_set_style_text_color(timer_lbl_, lv_color_hex(st.timer), 0);
		}

		const bool show_timer = state_ != UI_STATE_FAULT && state_ != UI_STATE_BOOT && total_ms_;
		if (show_timer) {
			fmt_countdown(buf, sizeof(buf), left_ms_, total_ms_);
			set_text(timer_lbl_, timer_cache_, buf);
			lv_obj_remove_flag(timer_lbl_, LV_OBJ_FLAG_HIDDEN);
		} else {
			lv_obj_add_flag(timer_lbl_, LV_OBJ_FLAG_HIDDEN);
		}

		snprintf(buf, sizeof(buf), "%u.%u V", unsigned(mv_ / 1000), unsigned(mv_ / 100 % 10));
		set_text(supply_lbl_, supply_cache_, buf);

		fmt_hms(buf, sizeof(buf), motor_s_);
		set_text(metric_lbl_[0], metric_cache_[0], buf);
		snprintf(buf, sizeof(buf), "%lu", (unsigned long)cyc_);
		set_text(metric_lbl_[1], metric_cache_[1], buf);
		fmt_hms(buf, sizeof(buf), uint32_t(esp_timer_get_time() / 1000000));
		set_text(metric_lbl_[2], metric_cache_[2], buf);
	}

	lv_obj_t *bar_ = nullptr;
	lv_obj_t *state_lbl_ = nullptr;
	lv_obj_t *timer_lbl_ = nullptr;
	lv_obj_t *supply_lbl_ = nullptr;
	lv_obj_t *metric_lbl_[3] = {};

	char state_cache_[24] = "";
	char timer_cache_[16] = "";
	char supply_cache_[16] = "";
	char metric_cache_[3][16] = {};
	int shown_state_ = -1;

	// model (written by API under lock)
	uint32_t cyc_ = 0, mv_ = 0, motor_s_ = 0;
	UiState_t state_ = UI_STATE_BOOT;
	uint32_t left_ms_ = 0, total_ms_ = 0;
	char text_[24] = "";
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
	// old code drove this pin low at init ("backlight"), keep it
	cfg.gpio_bl = BOARD_CFG_GPIO_LCD_BUSY;
	cfg.bl_on_level = 0;
	cfg.pclk_hz = 20 * 1000 * 1000;

	cfg.hres = BOARD_CFG_DISPLAY_WIDTH;
	cfg.vres = BOARD_CFG_DISPLAY_HEIGHT;
	// 284x76 window in ST7789 240x320 RAM, landscape (same as old cfg_lcd_panel)
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
	(void)prc;
	if (!UiEngine_lock(200)) {
		ESP_LOGE(TAG, "UI locked");
		return 0;
	}
	s_ui->setData(cyc, mv, sec);
	UiEngine_unlock();
	return 0;
}

int UiEngine_SetState(UiState_t st, uint32_t left_ms, uint32_t total_ms) {
	if (!UiEngine_lock(200)) {
		ESP_LOGE(TAG, "UI locked");
		return 0;
	}
	s_ui->setState(st, left_ms, total_ms);
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
