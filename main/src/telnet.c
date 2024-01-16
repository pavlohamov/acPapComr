/*
 * telnet.c
 *
 *  Created on: 17 May 2023
 *      Author: Pavlo
 */

#include "telnet.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <sys/socket.h>
#include <unistd.h>

#include "esp_log.h"
static const char *TAG = "mTelnet";


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

//#include "oled_lcd.h"

#define RB_SIZE (16 * 1024)

static struct {
	RingbufHandle_t rb;
	SemaphoreHandle_t sem;
} s_int;


static void telnet_routine(void *arg) {

	int srv = -1;
	int cli = -1;

	while (1) {
		if (cli >= 0)
			close(cli);

		if (srv >= 0)
			close(srv);

		struct sockaddr_in addr4 = {};
		addr4.sin_family = AF_INET;
		addr4.sin_port = htons(24);

		srv = socket(addr4.sin_family, SOCK_STREAM, IPPROTO_TCP);
		if (srv < 0) {
			ESP_LOGE(TAG, "Can't create sock %d", srv);
			sleep(1);
			continue;
		}

		int one = 1;
		int rv = setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
		if (rv < 0) {
			ESP_LOGE(TAG, "REUSEADDR %d %s", errno, strerror(errno));
			sleep(1);
			continue;
		}

		rv = bind(srv, (struct sockaddr*)&addr4, sizeof(addr4));
		if (rv < 0) {
			ESP_LOGE(TAG, "bind %d %s", errno, strerror(errno));
			sleep(1);
			continue;
		}

		rv = listen(srv, 1);
		if (rv < 0) {
			ESP_LOGE(TAG, "listen %d %s", errno, strerror(errno));
			sleep(1);
			continue;
		}

		struct sockaddr_in cliAddr;
		socklen_t len = sizeof(cliAddr);

		//oled_lcd_set_user("Not streaming");
		cli = accept(srv, (struct sockaddr*)&cliAddr, &len);
		if (cli < 0) {
			ESP_LOGD(TAG, "Accept %d", -errno);
			continue;
		}
		//oled_lcd_set_user("Streaming!");
		ESP_LOGD(TAG, "Accepted %d", cli);
		while (cli) {
			size_t size = 0;
			void *data = xRingbufferReceiveUpTo(s_int.rb, &size, portMAX_DELAY, RB_SIZE);
			if (!size || !data) {
				xSemaphoreTake(s_int.sem, portMAX_DELAY);
				continue;
			}
			int rv = send(cli, data, size, 0);
			vRingbufferReturnItem(s_int.rb, data);
			if (rv < 0) {
				ESP_LOGD(TAG, "Disconnected");
				break;
			}
		}
	}

}

int Telnet_put(const void *data, size_t len) {
	int rv = xRingbufferSend(s_int.rb, data, len, 0);
	xSemaphoreGive(s_int.sem);
	return rv ? len : 0;

}


int Telnet_init(void) {
	s_int.rb = xRingbufferCreate(RB_SIZE, RINGBUF_TYPE_BYTEBUF);
	s_int.sem = xSemaphoreCreateBinary();
	xTaskCreate(telnet_routine, "telnet", 2048, NULL, 12, NULL);
	return 0;
}

