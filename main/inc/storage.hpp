/*
 * storage.hpp
 *
 *  Created on: 28 Apr 2024
 *      Author: pavloha
 */

#pragma once


#include <stdint.h>

#include "driver/i2c_master.h"

typedef struct __attribute__((packed)) {
	uint32_t cycles;
	uint32_t uptime;
	uint8_t crc;
} SavedItem_t;

typedef struct {
	SavedItem_t itm;
	uint32_t nextAddr;
} Storage_t;


int Storage_init(i2c_master_dev_handle_t dev, Storage_t *p);

int Storage_save(i2c_master_dev_handle_t dev, Storage_t *p);
