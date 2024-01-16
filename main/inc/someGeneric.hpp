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


int LogUtil_init();

int UiEngine_init(void);
bool UiEngine_lock(int timeout_ms);
void UiEngine_unlock(void);

int WiFi_init(void);


#ifdef __cplusplus
}
#endif
