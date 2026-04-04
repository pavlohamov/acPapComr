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


int UiEngine_SetData(uint32_t cyc, uint32_t mv, uint32_t prc, uint32_t sec);
int UiEngine_SetText(const char *text);

int WiFi_init(void);


#ifdef __cplusplus
}
#endif
