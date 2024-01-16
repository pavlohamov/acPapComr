/*
 * wifi.c
 *
 *  Created on: 17 May 2023
 *      Author: Pavlo
 */


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>

#include "driver/gpio.h"

#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_event.h"
#include "esp_crc.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_log.h"
static const char *TAG = "mWiFi";


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

//#include "oled_lcd.h"

typedef struct {
	const char *ssid;
	const char *psk;
} WiFiCred_t;

static const WiFiCred_t s_creds[] = {
//		{ "SiGaN-Guest", "Mission99" },
//		{ "SiGaN-Guest", "sddd" },
};

static int getConnection(wifi_ap_record_t *ap) {
	for (int i = 0; i < sizeof(s_creds) / sizeof(*s_creds); ++i) {
		if (!strcmp((char*)ap->ssid, s_creds[i].ssid))
			return i;
	}
	return -1;
}

static int startAP(void) {

    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));

    wifi_config_t cfg;
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_AP, &cfg));

    snprintf((char*)cfg.ap.ssid, sizeof(cfg.ap.ssid), "SiLoG-%04X", *(uint16_t*)(mac + 4));
    snprintf((char*)cfg.ap.password, sizeof(cfg.ap.password), "0123456789");
    cfg.ap.authmode = WIFI_AUTH_WPA3_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));

    char buff[128];
	snprintf(buff, sizeof(buff), "WiFi AP '%s' pass '%s' ", cfg.ap.ssid, cfg.ap.password);
	ESP_LOGI(TAG, "%s", buff);
//	//oled_lcd_set_header(buff);
//	//oled_lcd_set_wifi("");
    return 0;
}

static void onEventWifi(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
	switch (event_id) {
		case WIFI_EVENT_WIFI_READY: {
			ESP_LOGD(TAG, "WiFi Ready");
//			//oled_lcd_set_header("WiFi Ready");
		} break;
		case WIFI_EVENT_SCAN_DONE: {
		    uint16_t count = 0;
		    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&count));
			ESP_LOGI(TAG, "Scan Done. Found %d", count);
//			//oled_lcd_set_header("WiFi STA Scan Done");
			wifi_ap_record_t *aps = malloc(count * sizeof(wifi_ap_record_t));
		    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&count, aps));

			int tryConnect = -1;
			for (int i = 0; i < count; ++i) {
				wifi_ap_record_t *ap = aps + i;
				ESP_LOGI(TAG, "%d/%d '%.32s' %d", i + 1, count, ap->ssid, ap->authmode);
				if (tryConnect == -1)
					tryConnect = getConnection(ap);
			}
			free(aps);

			if (tryConnect != -1) {
				char buff[64];
				snprintf(buff, sizeof(buff), "Connecting to %s", s_creds[tryConnect].ssid);
				ESP_LOGI(TAG, "%s", buff);
				//oled_lcd_set_header(buff);

			    wifi_config_t cfg;
			    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &cfg));
			    memcpy(cfg.sta.ssid, s_creds[tryConnect].ssid, strlen(s_creds[tryConnect].ssid));
			    memcpy(cfg.sta.password, s_creds[tryConnect].psk, strlen(s_creds[tryConnect].psk));
			    esp_wifi_clear_fast_connect();
			    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
			    ESP_ERROR_CHECK(esp_wifi_connect());
				break;
			}
			startAP();
		} break;

		case WIFI_EVENT_STA_START: {
			ESP_LOGI(TAG, "WiFi STA Ready. Scanning");
			//oled_lcd_set_header("WiFi STA Ready. Scanning");
		    esp_wifi_scan_start(NULL, false);
		} break;
		case WIFI_EVENT_STA_STOP: {
			ESP_LOGI(TAG, "Station stop");
		} break;
		case WIFI_EVENT_STA_CONNECTED: {
			ESP_LOGI(TAG, "Station connected to AP");
			//oled_lcd_set_header("WiFi STA Connected");
		} break;
		case WIFI_EVENT_STA_DISCONNECTED: {
			wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t*)event_data;
			//oled_lcd_set_header("WiFi STA Disconnected. Starting AP");
			ESP_LOGI(TAG, "Station disconnected from AP %d", event->reason);
			startAP();
		} break;
		case WIFI_EVENT_STA_AUTHMODE_CHANGE: {
			ESP_LOGI(TAG, "the auth mode of AP connected by device's station changed");
		} break;
		case WIFI_EVENT_AP_START: {
			ESP_LOGI(TAG, "Soft-AP start");
		} break;
		case WIFI_EVENT_AP_STOP: {
			ESP_LOGI(TAG, "Soft-AP stop");
		} break;
		case WIFI_EVENT_AP_STACONNECTED: {
			ESP_LOGI(TAG, "a station connected to Soft-AP");
			//oled_lcd_set_wifi("WiFi AP: Clien Connected");
		} break;
		case WIFI_EVENT_AP_STADISCONNECTED: {
			ESP_LOGI(TAG, "a station disconnected from Soft-AP");
			//oled_lcd_set_wifi("");
		} break;
		case WIFI_EVENT_AP_PROBEREQRECVED: {
			ESP_LOGI(TAG, "Receive probe request packet in soft-AP interface");
		} break;
		default: {
			ESP_LOGD(TAG, "event %ld", event_id);
		} break;

	}

}

static void onEventIp(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
	switch (event_id) {
		case IP_EVENT_STA_GOT_IP: {
			ip_event_got_ip_t *event = (ip_event_got_ip_t*)event_data;
			char buff[32];
			snprintf(buff, sizeof(buff), "STA " IPSTR, IP2STR(&event->ip_info.ip));
			ESP_LOGI(TAG, "Got ip %s", buff);
			//oled_lcd_set_wifi(buff);
		} break;
		case IP_EVENT_AP_STAIPASSIGNED: {
			ip_event_got_ip_t *event = (ip_event_got_ip_t*)event_data;
			char buff[32];
			snprintf(buff, sizeof(buff), "AP " IPSTR, IP2STR(&event->ip_info.ip));
			ESP_LOGI(TAG, "Got ip %s", buff);
			//oled_lcd_set_wifi(buff);
		} break;
		case IP_EVENT_GOT_IP6:
		case IP_EVENT_ETH_GOT_IP:
		case IP_EVENT_PPP_GOT_IP:{
			ESP_LOGI(TAG, "Got ip");
		} break;
		case IP_EVENT_STA_LOST_IP:
		case IP_EVENT_ETH_LOST_IP:
		case IP_EVENT_PPP_LOST_IP: {
			//oled_lcd_set_wifi("");
			ESP_LOGI(TAG, "lost ip");
		} break;
	}
}



int WiFi_init(void) {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_t *netif_sta = esp_netif_create_default_wifi_sta();
    esp_netif_t *netif_ap = esp_netif_create_default_wifi_ap();
    assert(netif_sta);
    assert(netif_ap);

    esp_netif_set_hostname(netif_sta, "my name");
    esp_netif_set_hostname(netif_ap, "my name");

    esp_netif_attach_wifi_station(netif_sta);
    esp_netif_attach_wifi_ap(netif_ap);

//    esp_set_default_wifi_handers();


    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onEventWifi, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, onEventIp, NULL));

    ESP_ERROR_CHECK(esp_wifi_start());
    return 0;
}
