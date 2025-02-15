/*
 * buttons.cpp
 *
 *  Created on: Oct 10, 2024
 *      Author: pavloha
 */

#include <errno.h>

#include "buttons.hpp"

#include "esp_timer.h"
#include "driver/gpio.h"

#include "esp_log.h"
static const char *TAG = "buttons";

using namespace Buttons;

#define K_MSEC(x) (x * 1000UL)


#define TOUT_DEBOUNCE (K_MSEC(25))

#define TOUT_FIRST_REPEAT (K_MSEC(300))
#define TOUT_REPEAT (K_MSEC(125))

enum {
	STT_RELEASED,
	STT_PRESSED,
	STT_REPEATED,
};

class Buttons::Button {
public:
	Button(int gpio, int btn, onButtonEvt_f cb, void *arg): _gpio((gpio_num_t)gpio), _btn(btn), _cb(cb), _arg(arg), _state(0), _pressedAt(0) {
		const esp_timer_create_args_t cfg = {
			.callback = onTout,
			.arg = this,
			.dispatch_method = ESP_TIMER_TASK,
			.name = "btntim",
			.skip_unhandled_events = false,
		};

		ESP_ERROR_CHECK(esp_timer_create(&cfg, &_tim));
		ESP_ERROR_CHECK(gpio_isr_handler_add(_gpio, onButtonCb, this));
		ESP_ERROR_CHECK(gpio_intr_enable(_gpio));

		onButtonCb(this);
	}
	virtual ~Button() {}
private:
	void inline callback(int evt) {
		 if (_cb)
			 _cb(_btn, evt, _arg);
	}
private:
	gpio_num_t _gpio;
	gpio_isr_handle_t _isr;
	esp_timer_handle_t _tim;
	const int _btn;
	const onButtonEvt_f _cb;
	void *const _arg;
	int _state;
	int64_t _pressedAt;
	int64_t _releasedAt;
public:

	static void onButtonCb(void *arg) {
		Button *b = reinterpret_cast<Button*>(arg);

		const int state = gpio_get_level(b->_gpio);

		if (state)
			esp_timer_start_once(b->_tim, TOUT_DEBOUNCE);
		else {
			esp_timer_stop(b->_tim);
			int was = b->_state;
			b->_state = STT_RELEASED;
			b->_pressedAt = 0;
			b->_releasedAt = esp_timer_get_time() / 1000LL;
			if (was)
				b->callback(RELEASE);
		}
	}

	static void onTout(void *arg) {
		Button *b = reinterpret_cast<Button*>(arg);
		switch (b->_state) {
			case STT_RELEASED:
				esp_timer_start_once(b->_tim, TOUT_FIRST_REPEAT);
				b->_pressedAt = esp_timer_get_time() / 1000LL;
				b->callback(PRESS);
				b->_releasedAt = 0;
				b->_state = STT_PRESSED;
				break;
			case STT_PRESSED:
				b->_state = STT_REPEATED;
				esp_timer_start_periodic(b->_tim, TOUT_REPEAT);
				b->callback(FIRST_REPEAT);
				break;
			case STT_REPEATED:
				b->callback(REPEAT);
				break;
		}
	}

	int getState() {
		return _state != STT_RELEASED;
	}
	int pressedFor() {
		return _pressedAt ? esp_timer_get_time() / 1000LL - _pressedAt : 0;
	}
	int releasedFor() {
		return _releasedAt ? esp_timer_get_time() / 1000LL - _releasedAt : 0;
	}

};

Wrapper::Wrapper() {
}

Wrapper::~Wrapper() {
	for (auto b : _buttons)
		delete b;
}

Button* Wrapper::add(int pin, int btn, onButtonEvt_f cb, void *arg) {

	static const gpio_config_t i_conf = {
		.pin_bit_mask = BIT64(pin),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_ANYEDGE,
	};
	int rv = gpio_config(&i_conf);
	if (rv) {
		ESP_LOGE(TAG, "gpio init fail %d", rv);
		return NULL;
	}

	Button *b = new Button(pin, btn, cb, arg);
	if (!b) {
		ESP_LOGE(TAG, "No MEM!");
		return NULL;
	}

	_buttons.push_back(b);
	return b;
}

//   0 - released
//   1 - pressed
// < 0 - error
int Wrapper::getState(int btn) const {
	if ((size_t)btn >= _buttons.size())
		return -EINVAL;
	return _buttons[btn]->getState();
}

int Wrapper::pressedFor(int btn) const {
	if ((size_t)btn >= _buttons.size())
		return -EINVAL;
	return _buttons[btn]->pressedFor();
}

int Wrapper::releasedFor(int btn) const {
	if ((size_t)btn >= _buttons.size())
		return -EINVAL;
	return _buttons[btn]->releasedFor();
}


