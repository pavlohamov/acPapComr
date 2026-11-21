/*
 * relayer.hpp
 *
 *  Created on: 15 Nov 2026
 *      Author: pavloha
 */

#ifndef MAIN_SRC_RELAYER_HPP_
#define MAIN_SRC_RELAYER_HPP_

#include <cstdint>
#include <string>

#include "sm/FSM.hpp"
#include "Runnable.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

class Relayer: public SM::FSM, public Runnable {
public:
	Relayer() {
		queue_ = xQueueCreate(16, sizeof(SM::FSM_Signal_t));
	}

	virtual ~Relayer() {
		vQueueDelete(queue_);
	}

	void go2next() {
		SM::FSM_Signal_t sig = Sig_Next;
		if (!xPortInIsrContext()) {
			xQueueSend(queue_, &sig, 0);
			return;
		}
		BaseType_t hpw = pdFALSE;
		xQueueSendFromISR(queue_, &sig, &hpw);
		if (hpw) {
			portYIELD_FROM_ISR();
		}
	}

	void forceRelay(bool on);

	uint64_t cycles() const { return cycles_; }

    static void* def(SM::FSM *const fsm, const SM::FSM_Event *event);
    	static void* initialWait(SM::FSM *const fsm, const SM::FSM_Event *event);

    	static void* waitTill(SM::FSM *const fsm, const SM::FSM_Event *event);

    	static void* toggling(SM::FSM *const fsm, const SM::FSM_Event *event);
    		static void* onFirst(SM::FSM *const fsm, const SM::FSM_Event *event);
    		static void* waitup(SM::FSM *const fsm, const SM::FSM_Event *event);
    		static void* onSecond(SM::FSM *const fsm, const SM::FSM_Event *event);

	enum class Phase { Boot, InitWait, Idle, Purge1, Pause, Purge2 };
	struct Status {
		Phase phase;
		uint32_t left_ms;
		uint32_t total_ms;
	};
	Status status() const;

	std::string info() const;

protected:
    virtual void run() noexcept;

private:
    uint64_t last_ms_ = 0;
    uint64_t cycles_ = 0;
    QueueHandle_t queue_;

    enum {
    	Sig_Next = SM::FSM_Signal_User,
    };

};




#endif /* MAIN_SRC_RELAYER_HPP_ */
