/*
 * relayer.cpp
 *
 *  Created on: 15 Nov 2026
 *      Author: pavloha
 */


#include "relayer.hpp"

#include "esp_timer.h"

#include "driver/gpio.h"

#include "board_cfg.h"

#include "esp_log.h"
static const char *TAG = "relayer";


#if 0
#define INITIAL_BLANK (3000)
#define CYCLE_TIME (4000)
#define RLY_PURGE_TIME_1 1000
#define RLY_PURGE_TIME_2 RLY_PURGE_TIME_1
#define RLY_DLY_TIME (1 * 1000)
#else
#define INITIAL_BLANK (3 * 60 * 1000)
#define CYCLE_TIME (10 * 60 * 1000)
#define RLY_PURGE_TIME_1 1000
#define RLY_PURGE_TIME_2 RLY_PURGE_TIME_1
#define RLY_DLY_TIME (5 * 1000)
#endif


static int init_gpio(void) {
	static const gpio_config_t o_conf = {
		.pin_bit_mask = BIT64(BOARD_CFG_GPIO_RLY_1_L) | BIT64(BOARD_CFG_GPIO_RLY_1_H) | BIT64(BOARD_CFG_GPIO_RLY_2_L) | BIT64(BOARD_CFG_GPIO_RLY_2_H)
		| BIT64(BOARD_CFG_GPIO_LED_0) | BIT64(BOARD_CFG_GPIO_LED_1) | BIT64(BOARD_CFG_GPIO_LED_2) | BIT64(BOARD_CFG_GPIO_LED_3),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	int rv = gpio_config(&o_conf);
	if (rv)
		return rv;

	static const gpio_config_t i_conf = {
		.pin_bit_mask = BIT64(BOARD_CFG_GPIO_BROWNOUT),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	rv = gpio_config(&i_conf);
	if (rv)
		return rv;

	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_1_L, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_1_H, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_2_L, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_RLY_2_H, 0);

	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_0, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_1, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_2, 0);
	gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_3, 0);

	return 0;
}

static inline void setRly(int rly, bool state) {
	static const int lut[] = {
#if 01
			BOARD_CFG_GPIO_RLY_1_L, BOARD_CFG_GPIO_RLY_2_L,
#else
			BOARD_CFG_GPIO_RLY_1_H, BOARD_CFG_GPIO_RLY_2_H,
#endif
	};
	if (rly > 1)
		return;

	gpio_set_level((gpio_num_t)lut[rly], state);
}


void Relayer::forceRelay(bool on) {
	setRly(0, on);
	setRly(1, on);
}


void* Relayer::def(SM::FSM *const fsm, const SM::FSM_Event *event) {
	void* state = reinterpret_cast<void*>(SM::FSM::Default);
	switch (event->sig) {
		case SM::FSM_Signal_ENTRY:
			break;
		case SM::FSM_Signal_STABLE:
			SM::FSM::Tran(fsm, initialWait);
			break;
		case SM::FSM_Signal_EXIT:
			break;
		default:
			if (event->sig >= SM::FSM_Signal_User)
				ESP_LOGE(TAG, "%s unhandled event %d", __FUNCTION__, event->sig);
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			break;
	}
	return state;
}

void* Relayer::initialWait(SM::FSM *const fsm, const SM::FSM_Event *event) {
	void* state = reinterpret_cast<void*>(def);
	Relayer* thiz = reinterpret_cast<Relayer*>(fsm);
    const uint64_t elapsed_ms = esp_timer_get_time() / 1'000UL - thiz->last_ms_;

	switch (event->sig) {
		case SM::FSM_Signal_ENTRY:
			thiz->last_ms_ = esp_timer_get_time() / 1'000UL;
			break;
		case SM::FSM_Signal_STABLE:
			ESP_LOGI(TAG, "initial blank %d ms", INITIAL_BLANK);
			break;
		case SM::FSM_Signal_EXIT:
			break;
		case SM::FSM_Signal_routine:
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			if (elapsed_ms >= INITIAL_BLANK) {
				SM::FSM::Tran(fsm, onFirst);
			}
			break;
		case Sig_Next:
			ESP_LOGI(TAG, "%s go2next", __FUNCTION__);
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			SM::FSM::Tran(fsm, onFirst);
			break;
	}
	return state;
}

void* Relayer::waitTill(SM::FSM *const fsm, const SM::FSM_Event *event) {
	void* state = reinterpret_cast<void*>(def);
	Relayer* thiz = reinterpret_cast<Relayer*>(fsm);
    const uint64_t elapsed_ms = esp_timer_get_time() / 1'000UL - thiz->last_ms_;

	switch (event->sig) {
		case SM::FSM_Signal_ENTRY:
			thiz->last_ms_ = esp_timer_get_time() / 1'000UL;
			break;
		case SM::FSM_Signal_STABLE:
			ESP_LOGI(TAG, "Waiting for %d ms", CYCLE_TIME);
			break;
		case SM::FSM_Signal_EXIT:
			break;
		case SM::FSM_Signal_routine:
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			if (elapsed_ms >= CYCLE_TIME) {
				SM::FSM::Tran(fsm, onFirst);
			}
			break;
		case Sig_Next:
			ESP_LOGI(TAG, "%s go2next", __FUNCTION__);
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			SM::FSM::Tran(fsm, onFirst);
			break;
	}
	return state;
}

void* Relayer::toggling(SM::FSM *const fsm, const SM::FSM_Event *event) {
	void* state = reinterpret_cast<void*>(def);
	Relayer* thiz = reinterpret_cast<Relayer*>(fsm);

	switch (event->sig) {
		case SM::FSM_Signal_ENTRY:
			thiz->last_ms_ = esp_timer_get_time() / 1'000UL;
			gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_0, 1);
			break;
		case SM::FSM_Signal_EXIT:
			gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_0, 0);
			break;
	}
	return state;
}

void* Relayer::onFirst(SM::FSM *const fsm, const SM::FSM_Event *event) {
	void* state = reinterpret_cast<void*>(toggling);
	Relayer* thiz = reinterpret_cast<Relayer*>(fsm);
    const uint64_t elapsed_ms = esp_timer_get_time() / 1'000UL - thiz->last_ms_;

	switch (event->sig) {
		case SM::FSM_Signal_ENTRY:
			thiz->last_ms_ = esp_timer_get_time() / 1'000UL;
			break;
		case SM::FSM_Signal_STABLE:
			ESP_LOGI(TAG, "First ON Second OFF %d ms", RLY_PURGE_TIME_1);
			setRly(0, 1);
			break;
		case SM::FSM_Signal_EXIT:
			setRly(0, 0);
			gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_1, 0);
			break;
		case SM::FSM_Signal_routine:
			gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_1, (elapsed_ms / 100) & 1);
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			if (elapsed_ms >= RLY_PURGE_TIME_1) {
				SM::FSM::Tran(fsm, waitup);
			}
			break;
		case Sig_Next:
			ESP_LOGI(TAG, "%s go2next", __FUNCTION__);
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			SM::FSM::Tran(fsm, waitup);
			break;
	}
	return state;
}

void* Relayer::waitup(SM::FSM *const fsm, const SM::FSM_Event *event) {
	void* state = reinterpret_cast<void*>(toggling);
	Relayer* thiz = reinterpret_cast<Relayer*>(fsm);
    const uint64_t elapsed_ms = esp_timer_get_time() / 1'000UL - thiz->last_ms_;

	switch (event->sig) {
		case SM::FSM_Signal_ENTRY:
			thiz->last_ms_ = esp_timer_get_time() / 1'000UL;
			break;
		case SM::FSM_Signal_STABLE:
			ESP_LOGI(TAG, "First OFF Second OFF %d ms", RLY_DLY_TIME);
			break;
		case SM::FSM_Signal_EXIT:
			break;
		case SM::FSM_Signal_routine:
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			if (elapsed_ms >= RLY_DLY_TIME) {
				SM::FSM::Tran(fsm, onSecond);
			}
			break;
		case Sig_Next:
			ESP_LOGI(TAG, "%s go2next", __FUNCTION__);
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			SM::FSM::Tran(fsm, onSecond);
			break;
	}
	return state;
}

void* Relayer::onSecond(SM::FSM *const fsm, const SM::FSM_Event *event) {
	void* state = reinterpret_cast<void*>(toggling);
	Relayer* thiz = reinterpret_cast<Relayer*>(fsm);
    const uint64_t elapsed_ms = esp_timer_get_time() / 1'000UL - thiz->last_ms_;

	switch (event->sig) {
		case SM::FSM_Signal_ENTRY:
			thiz->last_ms_ = esp_timer_get_time() / 1'000UL;
			break;
		case SM::FSM_Signal_STABLE:
			ESP_LOGI(TAG, "First OFF Second ON %d ms", RLY_PURGE_TIME_2);
			setRly(1, 1);
			break;
		case SM::FSM_Signal_EXIT:
			gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_2, 0);
			setRly(1, 0);
			thiz->cycles_ += 1;
			break;
		case SM::FSM_Signal_routine:
			gpio_set_level((gpio_num_t)BOARD_CFG_GPIO_LED_2, (elapsed_ms / 100) & 1);
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			if (elapsed_ms >= RLY_PURGE_TIME_2) {
				SM::FSM::Tran(fsm, waitTill);
			}
			break;
		case Sig_Next:
			ESP_LOGI(TAG, "%s go2next", __FUNCTION__);
			state = reinterpret_cast<void*>(SM::FSM::Handled);
			SM::FSM::Tran(fsm, waitTill);
			break;
	}
	return state;
}


std::string Relayer::info() const {
	if (isInState(initialWait)) {
		const uint64_t elapsed_ms = esp_timer_get_time() / 1'000UL - last_ms_;
		const uint32_t left = INITIAL_BLANK - elapsed_ms;

		char text[129];
		snprintf(text, sizeof(text), "init wait %lu/%u sec", left / 1000, INITIAL_BLANK / 1000);
		return text;
	}

	if (isInState(waitTill)) {
		const uint64_t elapsed_ms = esp_timer_get_time() / 1'000UL - last_ms_;
		const uint32_t left = CYCLE_TIME - elapsed_ms;

		char text[129];
		snprintf(text, sizeof(text), "     wait %lu/%u sec", left / 1000, CYCLE_TIME / 1000);
	}

	if (isInState(onFirst)) {
		const uint64_t elapsed_ms = esp_timer_get_time() / 1'000UL - last_ms_;
		const uint32_t left = RLY_PURGE_TIME_1 - elapsed_ms;

		char text[129];
		snprintf(text, sizeof(text), "   ON OFF %lu/%u ms", left, RLY_PURGE_TIME_1);
		return "";
	}

	if (isInState(waitup)) {
		const uint64_t elapsed_ms = esp_timer_get_time() / 1'000UL - last_ms_;
		const uint32_t left = RLY_PURGE_TIME_1 - elapsed_ms;

		char text[129];
		snprintf(text, sizeof(text), "  OFF OFF %lu/%u ms", left, RLY_DLY_TIME);
		return "";
	}

	if (isInState(onSecond)) {
		const uint64_t elapsed_ms = esp_timer_get_time() / 1'000UL - last_ms_;
		const uint32_t left = RLY_PURGE_TIME_1 - elapsed_ms;

		char text[129];
		snprintf(text, sizeof(text), "  OFF  ON %lu/%u ms", left, RLY_PURGE_TIME_1);
		return "";
	}

	return "wtf?!";
}

void Relayer::run() {

	init_gpio();
	Init(def);

	constexpr SM::FSM_Event rot { SM::FSM_Signal_routine };

	while (running()) {
		SM::FSM_Signal_t sig;
		if (!xQueueReceive(queue_, &sig, pdMS_TO_TICKS(1))) {
			Dispatch(&rot);
			continue;
		}
		SM::FSM_Event evt { sig };
		Dispatch(&evt);
	}

}
