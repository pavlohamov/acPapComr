/*
 * buttons.hpp
 *
 *  Created on: Oct 10, 2024
 *      Author: pavloha
 */

#pragma once

#include <vector>

namespace Buttons {

enum {
	PRESS,
	FIRST_REPEAT,
	REPEAT,
	RELEASE,
};

typedef void (*onButtonEvt_f) (int btn, int evt, void *arg);


class Button;

class Wrapper {
private:
	Wrapper();
	virtual ~Wrapper();

public:
	static Wrapper &instance() {
		static Wrapper ins;
		return ins;
	}

	Button* add(int gpio_n, int btn, onButtonEvt_f cb, void *arg, bool invert = false);

//   0 - released
//   1 - pressed
// < 0 - error
	int getState(int btn) const;
// how many ms is pressed
	int pressedFor(int btn) const;
// how many ms since released
	int releasedFor(int btn) const;

private:
	std::vector<Button*> _buttons;
};

};
