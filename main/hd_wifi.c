/*
WIFI network manager
Copyright (c) 2018 ys1797 (yuri@rus.net)

License (MIT license):
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "math.h"
#include "time.h"
#include <string.h>
#include <sys/types.h>
#include "lwip/apps/sntp.h"
#include <cJSON.h>

//#include "captdns.h"
#include "config.h"
#include "hd_wifi.h"
#include "hd_main.h"

#define HD_MAX_STA_CONN       4
#define HD_ESP_MAXIMUM_RETRY  30

uint16_t WIFI_scanCount;
bool WIFI_scanStarted;
bool WIFI_scanComplete;
wifi_ap_record_t *WIFI_scanResult;
uint8_t WIFI_knowApCount;		// Количество известных AP
uint8_t WIFI_currentAp;
wifi_know_ap *WIFI_knowAp;
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
const int DISCONNECTED_BIT = BIT1;

static int retryNum = 0;
static char is_connected = 0;
static esp_netif_t* net_instanse_ptr;


const char * system_event_reasons[] = { "UNSPECIFIED", "AUTH_EXPIRE", "AUTH_LEAVE", "ASSOC_EXPIRE", "ASSOC_TOOMANY", "NOT_AUTHED", "NOT_ASSOCED", "ASSOC_LEAVE", "ASSOC_NOT_AUTHED", "DISASSOC_PWRCAP_BAD", "DISASSOC_SUPCHAN_BAD", "IE_INVALID", "MIC_FAILURE", "4WAY_HANDSHAKE_TIMEOUT", "GROUP_KEY_UPDATE_TIMEOUT", "IE_IN_4WAY_DIFFERS", "GROUP_CIPHER_INVALID", "PAIRWISE_CIPHER_INVALID", "AKMP_INVALID", "UNSUPP_RSN_IE_VERSION", "INVALID_RSN_IE_CAP", "802_1X_AUTH_FAILED", "CIPHER_SUITE_REJECTED", "BEACON_TIMEOUT", "NO_AP_FOUND", "AUTH_FAIL", "ASSOC_FAIL", "HANDSHAKE_TIMEOUT" };
#define reason2str(r) ((r>176)?system_event_reasons[r-176]:system_event_reasons[r-1])

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{

	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        	ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		retryNum = 0;
		is_connected = 1; // Set connected flag
		xEventGroupClearBits(wifi_event_group, DISCONNECTED_BIT);
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		ESP_LOGI(TAG, "got ip: " IPSTR "\n", IP2STR(&event->ip_info.ip));
		ESP_LOGI(TAG, "netmask: " IPSTR "\n", IP2STR(&event->ip_info.netmask));
		ESP_LOGI(TAG, "gw: " IPSTR "\n", IP2STR(&event->ip_info.gw));
		ESP_LOGI(TAG, "Initializing SNTP");
		setenv("TZ", "MSK-3", 1);
		sntp_setoperatingmode(SNTP_OPMODE_POLL);
		sntp_setservername(0, "pool.ntp.org");
		sntp_init();
		fflush(stdout);
	} else if (event_base == WIFI_EVENT) {
		switch(event_id) {
		case SYSTEM_EVENT_SCAN_DONE:

			if (WIFI_scanResult) {
				free(WIFI_scanResult);
				WIFI_scanResult = NULL;
			}
			esp_wifi_scan_get_ap_num(&WIFI_scanCount);
			if (WIFI_scanCount) {
				WIFI_scanResult = calloc(sizeof(wifi_ap_record_t), WIFI_scanCount);
				if (WIFI_scanResult) {
					esp_wifi_scan_get_ap_records(&WIFI_scanCount, WIFI_scanResult);
				} else {
					WIFI_scanCount = 0;
				}
			}
			WIFI_scanComplete = true;
			WIFI_scanStarted = false;
			break;
		case WIFI_EVENT_STA_START:
			ESP_LOGI(TAG, "WiFi start event");
			esp_wifi_connect();
			break;
		case WIFI_EVENT_STA_STOP:
			ESP_LOGI(TAG, "WiFi stop event");
			break;
		case WIFI_EVENT_STA_DISCONNECTED:
			/* This is a workaround as ESP32 WiFi libs don't currently
			   auto-reassociate. */
			{
			wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
			sntp_stop();
			ESP_LOGI(TAG, "WiFi disconnect event, resason: %d", event->reason);

			if (!is_connected && WIFI_REASON_NO_AP_FOUND == event->reason) {
				WIFI_currentAp++;
				if (WIFI_currentAp< WIFI_knowApCount) {
					wifi_know_ap *w = &WIFI_knowAp[WIFI_currentAp];
					wifi_config_t wifi_config = { 0 };
					strlcpy((char*) wifi_config.sta.ssid, w->ssid, sizeof(wifi_config.sta.ssid));
					if (strlen(w->password)) {
						strncpy((char*) wifi_config.sta.password, w->password, sizeof(wifi_config.sta.password));
					}
					wifi_config.sta.channel = 1;
					ESP_LOGI(TAG, "New WiFi STA: SSID %s", wifi_config.sta.ssid);
					ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
					ESP_ERROR_CHECK( esp_wifi_start() );
					esp_wifi_connect();
				} else {
					wifi_cmd_ap_set();
				}

			} else {
				retryNum++;
			        if (retryNum < HD_ESP_MAXIMUM_RETRY) {
					esp_wifi_connect();
					retryNum++;
					ESP_LOGI(TAG, "retry to connect to the AP, retry #%d", retryNum);
				} else {
					xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
					xEventGroupSetBits(wifi_event_group, DISCONNECTED_BIT);
					ESP_LOGI(TAG,"connect to the AP fail");
			        }
			}
			}
			break;
		case WIFI_EVENT_AP_STACONNECTED:
			{
			wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
			ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
			}
			break;
		case WIFI_EVENT_AP_STADISCONNECTED:
			{
			wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
			ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
			}
			break;
		default:
			break;
		}
	}
}

/*
 * Start scan WiFi networks available
 */
int8_t scanWifiNetworks(bool async, bool show_hidden, bool passive, uint32_t max_ms_per_chan)
{
	wifi_scan_config_t config;

	if (WIFI_scanStarted) return -2;

	config.ssid = 0;
	config.bssid = 0;
	config.channel = 0;
	config.show_hidden = show_hidden;
	if (passive){
		config.scan_type = WIFI_SCAN_TYPE_PASSIVE;
		config.scan_time.passive = max_ms_per_chan;
	} else {
	        config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
        	config.scan_time.active.min = 100;
	        config.scan_time.active.max = max_ms_per_chan;
	}

	if (esp_wifi_scan_start(&config, false) == ESP_OK) {
		WIFI_scanComplete = false;
		WIFI_scanStarted = true;
		if (async) return 0;
		while (!WIFI_scanComplete) ets_delay_us(10000);
		return WIFI_scanCount;
        }
	return -1;
}

const char *wifi_auth_string(wifi_auth_mode_t auth)
{
	switch (auth) {
	case WIFI_AUTH_OPEN: 		return "open";
	case WIFI_AUTH_WEP: 		return "WEP";
	case WIFI_AUTH_WPA_PSK:		return "WPA_PSK";
	case WIFI_AUTH_WPA2_PSK:	return "WPA2_PSK";
	case WIFI_AUTH_WPA_WPA2_PSK:	return "WPA_WPA2_PSK";
	case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
	default: return "Unknow";
	}
}

// Получение сохраненной конфигурации WiFi точек
int get_wifi_config(void)
{
	FILE *f;
	struct stat st;
	char *buff = NULL;
	size_t size;

	WIFI_knowApCount = 0;
	WIFI_currentAp = 0;
	if (WIFI_knowAp) {
		free(WIFI_knowAp);
		WIFI_knowAp = NULL;	
	}
	if (stat(WIFI_CONFIGURATION, &st) != 0) return -1;
	f = fopen(WIFI_CONFIGURATION, "r");
	if (!f) return -1;
	buff = malloc(st.st_size+2);
	if (!buff) {
		fclose(f);
		return -2;
	}
	size = fread(buff, 1, st.st_size, f);
	buff[size]=0;
	fclose(f);

	cJSON *root = cJSON_Parse(buff);
	free(buff);
	if (!root) return -3;

	WIFI_knowApCount = cJSON_GetArraySize(root);
	WIFI_knowAp = calloc(sizeof(wifi_know_ap), WIFI_knowApCount);
	for (int i=0; i<WIFI_knowApCount; i++) {
		cJSON *ap = cJSON_GetArrayItem(root,i);
		cJSON *s = cJSON_GetObjectItem(ap, "ssid");
		cJSON *p = cJSON_GetObjectItem(ap, "pass");
		if (s && p && s->valuestring && p->valuestring) {
			strncpy(WIFI_knowAp[i].ssid, s->valuestring, 33);
			strncpy(WIFI_knowAp[i].password, p->valuestring, 64);
		}
				
	}
	cJSON_Delete(root);
	return 0;
}

int wifi_cmd_ap_set(void)
{
	wifi_config_t wifi_config = {
	        .ap = {
			.ssid_len = strlen(Hostname),
			.max_connection = HD_MAX_STA_CONN,
			.password = "",
			.authmode = WIFI_AUTH_OPEN
	        },
	};
	net_instanse_ptr = esp_netif_create_default_wifi_ap();

	strlcpy((char*) wifi_config.ap.ssid, Hostname, sizeof(wifi_config.sta.ssid));


//	esp_wifi_disconnect();
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
//	captdnsInit();
	return 0;
}


/*
 * Настройка wifi
 */
esp_err_t wifiSetup(void)
{
	net_instanse_ptr = NULL;
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	wifi_event_group = xEventGroupCreate();

	/* Получение конфигурации wifi */
	get_wifi_config();

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));


	if (!WIFI_knowApCount) {
		// Нет заданных точек доступа - переходим в режим AP
		wifi_cmd_ap_set();
	} else {
		wifi_know_ap *w = &WIFI_knowAp[WIFI_currentAp];
		wifi_config_t wifi_config = { 0 };

		net_instanse_ptr = esp_netif_create_default_wifi_sta();

		strlcpy((char*) wifi_config.sta.ssid, w->ssid, sizeof(wifi_config.sta.ssid));
		if (strlen(w->password)) {
			strncpy((char*) wifi_config.sta.password, w->password, sizeof(wifi_config.sta.password));
		}
		wifi_config.sta.channel = 1;


		ESP_LOGI(TAG, "WiFi STA SSID: %s", wifi_config.sta.ssid);
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
		ESP_ERROR_CHECK(esp_wifi_start());
	}
	return ESP_OK;
}

esp_netif_t *getNetHandle(void){
	return net_instanse_ptr;
}
