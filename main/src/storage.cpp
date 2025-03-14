/*
 * storage.cpp
 *
 *  Created on: 28 Apr 2024
 *      Author: pavloha
 */

#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "storage.hpp"
#include "sdkconfig.h"
#include "board_cfg.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "rom/crc.h"

#include "esp_system.h"
#include "esp_log.h"
static const char *TAG = "storage";

static const char s_magic[] = "NVRAM is Ready.";

#define NVRAM_SIZE (2 * 1024)

#define STORAGE_SIZE (NVRAM_SIZE - sizeof(s_magic))
#define STORAGE_ITM_COUNT (STORAGE_SIZE / sizeof(SavedItem_t))

#define SWAP_16(x) (((x << 8) & 0xFF00) | ((x >> 8) & 0xFF))

#define PAGE_SIZE 32

int Storage_init(i2c_master_dev_handle_t dev, Storage_t *p) {
	if (!p) {
		ESP_LOGE(TAG, "input is null");
		return -EINVAL;
	}

	memset(p, 0, sizeof(*p));

	char hdr[sizeof(s_magic)];

	uint16_t addr = 0;
	int rv = i2c_master_transmit_receive(dev, (uint8_t*)&addr, sizeof(addr), (uint8_t*)hdr, sizeof(hdr), 10);
	if (rv) {
		ESP_LOGE(TAG, "i2c read failure %d", rv);
		return rv;
	}

	ESP_LOG_BUFFER_HEX_LEVEL(TAG, hdr, sizeof(hdr), ESP_LOG_WARN);

	if (memcmp(s_magic, hdr, sizeof(hdr))) {
		ESP_LOGI(TAG, "NVRAM must be initialized");
		Storage_wipe(dev);
	} else {
		ESP_LOGI(TAG, "NVRAM is inited. search for next adddr");

		uint32_t size = STORAGE_SIZE;
		uint8_t *ptr = (uint8_t*)malloc(size);

		addr = SWAP_16(sizeof(s_magic));
		rv = i2c_master_transmit_receive(dev, (uint8_t*)&addr, sizeof(addr), ptr, size, 1500);

		SavedItem_t *itm = (SavedItem_t*)ptr;
		SavedItem_t *next = itm;
		if (rv) {
			ESP_LOGE(TAG, "i2c read failure %X", rv);
			memset(p, 0, sizeof(*p));
		} else {
			ESP_LOGI(TAG, "read done search of %d els", STORAGE_ITM_COUNT);
			for (int i = 1; i < STORAGE_ITM_COUNT; ++i) {
				SavedItem_t *si = itm + i;
				if (si->cycles == 0xFFFFFFFF)
					continue;

				const uint8_t crc = crc8_le(0, (uint8_t*)si, sizeof(*si) - sizeof(si->crc));
				if (crc != si->crc)
					continue;

				if (next->cycles == 0xFFFFFFFF)
					next = si;

				if (si->cycles > next->cycles)
					next = si;
			}
			p->itm = *next;
			p->nextAddr = (uint32_t)next - (uint32_t)ptr;
			if (p->itm.cycles == 0xFFFFFFFF)
				memset(p, 0, sizeof(*p));
		}
		free(ptr);
	}
	ESP_LOGI(TAG, "Uptime %ld. cycles %ld. next addr %ld", p->itm.uptime, p->itm.cycles, p->nextAddr);

	return 0;
}

int Storage_wipe(i2c_master_dev_handle_t dev) {
	static const size_t burst = PAGE_SIZE;
	uint8_t *ptr = (uint8_t*)malloc(burst + 2);
	uint16_t *pAddr = (uint16_t*)ptr;
	uint8_t *data = ptr + 2;

	for (int i = 0; i < NVRAM_SIZE; i += burst) {
		*pAddr = SWAP_16(i);
		memset(data, 0xFF, burst);
		if (!i)
			memcpy(data, s_magic, sizeof(s_magic));

		if (!i || i == burst)
			ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, burst, ESP_LOG_WARN);

		ESP_LOGI(TAG, "Write %d/%d", i, NVRAM_SIZE);


		int rv = i2c_master_transmit(dev, ptr, burst + 2, 50);
		if (rv) {
			ESP_LOGE(TAG, "i2c write failure %d", rv);
		}
		vTaskDelay(pdMS_TO_TICKS(6));
	}
	free(ptr);
	return 0;
}

int Storage_save(i2c_master_dev_handle_t dev, Storage_t *p) {

	p->nextAddr += sizeof(p->itm);

	p->itm.crc = crc8_le(0, (uint8_t*)&p->itm, sizeof(p->itm) - sizeof(p->itm.crc));

	static const uint32_t maxAddr = NVRAM_SIZE - NVRAM_SIZE % sizeof(SavedItem_t) - sizeof(s_magic);
	if (p->nextAddr >= maxAddr) // round address
		p->nextAddr = 0;

	const uint16_t start = p->nextAddr + sizeof(s_magic);
	const uint16_t end = start + sizeof(p->itm);

	ESP_LOGI(TAG, "Saving:Uptime %ld. cycles %ld. addr %ld", p->itm.uptime, p->itm.cycles, start);

	static const int size = sizeof(p->itm) + 2;
	uint8_t *ptr = (uint8_t*)malloc(size);
	uint16_t *pa = (uint16_t*)ptr;
	uint8_t *data = ptr + 2;

	memcpy(data, &p->itm, sizeof(p->itm));
	*pa = SWAP_16(start);
	int rv = 0;
	if (start / PAGE_SIZE == end / PAGE_SIZE) {
		rv = i2c_master_transmit(dev, ptr, size, 50);
	} else {
		uint16_t write = PAGE_SIZE - start % PAGE_SIZE;
		uint16_t nextWrite = sizeof(p->itm) - write;
		uint16_t nextPage = start + write;
		ESP_LOGI(TAG, "Saving:not fits to 1 page %d != %d. %d + %d pga%d", start / PAGE_SIZE, end / PAGE_SIZE, write, nextWrite, nextPage);
		i2c_master_transmit(dev, ptr, write + 2, 50);
		vTaskDelay(pdMS_TO_TICKS(6));
		*pa = SWAP_16(nextPage);
		uint8_t *pitm = (uint8_t*)&p->itm;
		memcpy(data, pitm + write, nextWrite);
		rv = i2c_master_transmit(dev, ptr, nextWrite + 2, 50);
	}
	vTaskDelay(pdMS_TO_TICKS(6));

	free(ptr);

	if (rv) {
		ESP_LOGE(TAG, "i2c write failure %X", rv);
		return rv;
	}

	return 0;
}
