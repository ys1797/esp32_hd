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
#include "esp_event_loop.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "math.h"
#include "time.h"
#include <string.h>
#include <sys/types.h>
#include "lwip/apps/sntp.h"
#include <cJSON.h>

#include "captdns.h"
#include "config.h"
#include "hd_wifi.h"
#include "hd_main.h"

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

// Получение конфигурации сетевого доступа
int get_network_config(void)
{
	struct stat st;
	char *buff = NULL;
	size_t size;
	FILE *f = fopen(NET_CONFIGURATION, "r");
	cJSON *v;

	if (!f || stat(NET_CONFIGURATION, &st)) {
		// По умолчанию
		Hostname = strdup(DEFAULT_HOST);
		httpUser = strdup(DEFAULT_USERNAME);
		httpPassword = strdup(DEFAULT_PASSWORD);
		httpSecure = 0;
		useSmsc = 0;
		return 0;
	}

	buff = malloc(st.st_size);
	if (!buff) {
		fclose(f);
		return -2;
	}
	size = fread(buff, 1, st.st_size, f);
	buff[size]=0;
	fclose(f);

	cJSON *root = cJSON_Parse(buff);
	free(buff);

	v = cJSON_GetObjectItem(root, "host");
	if (v && v->valuestring) Hostname = strdup(v->valuestring);

	v = cJSON_GetObjectItem(root, "user");
	if (v && v->valuestring) httpUser = strdup(v->valuestring);

	v = cJSON_GetObjectItem(root, "pass");
	if (v && v->valuestring) httpPassword = strdup(v->valuestring);

	v = cJSON_GetObjectItem(root, "secure");
	if (v) httpSecure = v->valueint;

	v = cJSON_GetObjectItem(root, "smscUser");
	if (v && v->valuestring) smscUser = strdup(v->valuestring);

	v = cJSON_GetObjectItem(root, "smscHash");
	if (v && v->valuestring) smscHash = strdup(v->valuestring);

	v = cJSON_GetObjectItem(root, "smscPhones");
	if (v && v->valuestring) smscPhones = strdup(v->valuestring);

	v = cJSON_GetObjectItem(root, "useSmsc");
	if (v)  useSmsc = v->valueint;

	cJSON_Delete(root);
	return 0;
}

// Сохранение конфигурации сетевого доступа
int set_network_config(char *host, char *user, char *pass, int secure,
	char *su, char *sh, char *sp, int usesmsc, int wsperiod)
{
	cJSON *ja;
	if (!host || !user || !pass) return -1;

	ja = cJSON_CreateObject();
	cJSON_AddItemToObject(ja, "host", cJSON_CreateString(host));
	cJSON_AddItemToObject(ja, "user", cJSON_CreateString(user));
	cJSON_AddItemToObject(ja, "pass", cJSON_CreateString(pass));
	cJSON_AddItemToObject(ja, "secure", cJSON_CreateNumber(secure));
	if (su) cJSON_AddItemToObject(ja, "smscUser", cJSON_CreateString(su));
	if (sh) cJSON_AddItemToObject(ja, "smscHash", cJSON_CreateString(sh));
	if (sp) cJSON_AddItemToObject(ja, "smscPhones", cJSON_CreateString(sp));
	cJSON_AddItemToObject(ja, "useSmsc", cJSON_CreateNumber(usesmsc));
	cJSON_AddItemToObject(ja, "wsPeriod", cJSON_CreateNumber(wsperiod));

	FILE *f = fopen(NET_CONFIGURATION, "w");
	if (f) {
		char *r=cJSON_Print(ja);
		fprintf(f, "%s", r);
		if (r) free(r);
		fclose(f);
	}
	cJSON_Delete(ja);	
	if (Hostname) free(Hostname);
	if (httpUser) free(httpUser);
	if (httpPassword) free(httpPassword);
	if (smscUser) free(smscUser);
	if (smscHash) free(smscHash);
	if (smscPhones) free(smscPhones);

	Hostname = strdup(host);
	httpUser = strdup(user);
	httpPassword = strdup(pass);
	httpSecure = secure;
	if (su) smscUser = strdup(su);
	if (sh) smscHash = strdup(sh);
	if (sp) smscPhones = strdup(sp);
	useSmsc = usesmsc;
	wsPeriod = wsperiod;
	return 0;
}

const char * system_event_reasons[] = { "UNSPECIFIED", "AUTH_EXPIRE", "AUTH_LEAVE", "ASSOC_EXPIRE", "ASSOC_TOOMANY", "NOT_AUTHED", "NOT_ASSOCED", "ASSOC_LEAVE", "ASSOC_NOT_AUTHED", "DISASSOC_PWRCAP_BAD", "DISASSOC_SUPCHAN_BAD", "IE_INVALID", "MIC_FAILURE", "4WAY_HANDSHAKE_TIMEOUT", "GROUP_KEY_UPDATE_TIMEOUT", "IE_IN_4WAY_DIFFERS", "GROUP_CIPHER_INVALID", "PAIRWISE_CIPHER_INVALID", "AKMP_INVALID", "UNSUPP_RSN_IE_VERSION", "INVALID_RSN_IE_CAP", "802_1X_AUTH_FAILED", "CIPHER_SUITE_REJECTED", "BEACON_TIMEOUT", "NO_AP_FOUND", "AUTH_FAIL", "ASSOC_FAIL", "HANDSHAKE_TIMEOUT" };
#define reason2str(r) ((r>176)?system_event_reasons[r-176]:system_event_reasons[r-1])

esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id) {
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
	case SYSTEM_EVENT_STA_START:
		ESP_LOGI(TAG, "WiFi start event");
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_STOP:
		ESP_LOGI(TAG, "WiFi stop event");
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		xEventGroupClearBits(wifi_event_group, DISCONNECTED_BIT);
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		ESP_LOGI(TAG, "got ip: " IPSTR "\n", IP2STR(&event->event_info.got_ip.ip_info.ip));
		ESP_LOGI(TAG, "netmask: " IPSTR "\n", IP2STR(&event->event_info.got_ip.ip_info.netmask));
		ESP_LOGI(TAG, "gw: " IPSTR "\n", IP2STR(&event->event_info.got_ip.ip_info.gw));
		ESP_LOGI(TAG, "Initializing SNTP");
		setenv("TZ", "MSK-3", 1);
		sntp_setoperatingmode(SNTP_OPMODE_POLL);
		sntp_setservername(0, "pool.ntp.org");
		sntp_init();
		fflush(stdout);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		/* This is a workaround as ESP32 WiFi libs don't currently
		   auto-reassociate. */
//		int reason = event->event_info.disconnected.reason;
		sntp_stop();
		ESP_LOGI(TAG, "WiFi disconnect event, resason: %d", event->event_info.disconnected.reason);
		
		if (WIFI_REASON_NO_AP_FOUND == event->event_info.disconnected.reason) {
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
			esp_wifi_connect();
		}
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		xEventGroupSetBits(wifi_event_group, DISCONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
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
		.max_connection = 4,
		.password = "",
		.authmode = WIFI_AUTH_OPEN
        },
	};
	esp_wifi_disconnect();
	strlcpy((char*) wifi_config.ap.ssid, Hostname, sizeof(wifi_config.sta.ssid));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
	ESP_ERROR_CHECK( esp_wifi_start() );
//	captdnsInit();
	return 0;
}


/*
 * Настройка wifi
 */
esp_err_t wifi_setup(void)
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	/* Получение конфигурации wifi */
	get_wifi_config();

	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );

	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

	if (!WIFI_knowApCount) {
		// Нет заданных точек доступа - переходим в режим AP
		wifi_cmd_ap_set();
	} else {
		wifi_know_ap *w = &WIFI_knowAp[WIFI_currentAp];

		wifi_config_t wifi_config = { 0 };
		strlcpy((char*) wifi_config.sta.ssid, w->ssid, sizeof(wifi_config.sta.ssid));
		if (strlen(w->password)) {
			strncpy((char*) wifi_config.sta.password, w->password, sizeof(wifi_config.sta.password));
		}
		wifi_config.sta.channel = 1;

		ESP_LOGI(TAG, "WiFi STA SSID: %s", wifi_config.sta.ssid);
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
//		ESP_ERROR_CHECK(esp_wifi_set_auto_connect(true));
		ESP_ERROR_CHECK(esp_wifi_start());
//		ESP_ERROR_CHECK(esp_wifi_connect());
	}
	return ESP_OK;
}



