/*
 * someGeneric.hpp
 *
 *  Created on: 12 Jan 2024
 *      Author: pavloha
 */


#pragma once


#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>


int LogUtil_init();

int UiEngine_init(void);

int UiEngine_SetData(uint32_t cyc, uint32_t mv, uint32_t prc, uint32_t sec);
int UiEngine_SetText(const char *text);

typedef enum {
	UI_STATE_BOOT,
	UI_STATE_STARTING,   // initial blank after power-up
	UI_STATE_IDLE,       // waiting for the next purge cycle
	UI_STATE_PURGE_1,
	UI_STATE_PAUSE,
	UI_STATE_PURGE_2,
	UI_STATE_FAULT,
	UI_STATE_COUNT,
} UiState_t;

// left_ms / total_ms: countdown of the current phase (total_ms == 0 -> no timer)
int UiEngine_SetState(UiState_t st, uint32_t left_ms, uint32_t total_ms);

int WiFi_init(void);


#ifdef __cplusplus
}
#endif
