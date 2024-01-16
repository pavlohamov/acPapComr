/*
 * telnet.h
 *
 *  Created on: 17 May 2023
 *      Author: Pavlo
 */

#pragma once


#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


int Telnet_put(const void *data, size_t len);
int Telnet_init(void);

#ifdef __cplusplus
}
#endif
