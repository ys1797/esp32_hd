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

#include "esp_wifi.h"
#define DEFAULT_SCAN_LIST_SIZE (20)

typedef struct {
	char ssid[33];		// SSID of AP
	char password[64];	// Password of ESP32 AP
} wifi_know_ap;

extern esp_netif_t *sta_netif;
extern uint16_t WIFI_scanCount;
extern wifi_ap_record_t WIFI_scanResult[DEFAULT_SCAN_LIST_SIZE];

/* Получение сохраненной конфигурации WiFi точек */
int get_wifi_config(void);
int wifi_cmd_ap_set(void);
/* Start scan WiFi networks available */
int8_t scanWifiNetworks(bool async, bool show_hidden, bool passive, uint32_t max_ms_per_chan);
const char *wifi_auth_string(wifi_auth_mode_t auth);
esp_err_t wifiSetup(void); // Настройка wifi


