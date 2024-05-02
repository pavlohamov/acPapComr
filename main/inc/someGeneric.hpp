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
bool UiEngine_lock(int timeout_ms);
void UiEngine_unlock(void);


int UiEngine_SetCycles(uint32_t cyc);
int UiEngine_SetUptime(uint32_t cyc);
int UiEngine_SetPercent(uint32_t prc);

void UiEngine_forceredraw(void);

int WiFi_init(void);


#ifdef __cplusplus
}
#endif
