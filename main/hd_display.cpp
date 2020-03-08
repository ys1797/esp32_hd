/*
esp32_hd display driver
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
#include <sys/types.h>
#include "esp_log.h"
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include <cJSON.h>
#include "hd_main.h"
#include "ds.h"
#include "hd_bmp180.h"
#include "hd_wifi.h"
#include "sh1106.h"
#include "hd_spi_i2c.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

/* I2C address */
#ifndef DISPLAY_I2C_ADDR
#define DISPLAY_I2C_ADDR         0x3C//0x78
#endif

#define screenW 128
#define screenH 64

Adafruit_ILI9341 *Tft;

int clockCenterX = screenW/2;
int clockCenterY = ((screenH-16)/2)+16;   // top yellow part is 16 px height
int clockRadius = 23;

extern volatile uint32_t uptime_counter;
extern volatile int32_t Hpoint;


/*
 * Функция вывода информации на первый экран
 */
void mainFrame(OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	char b[80];
	double f;
	oledSetTextAlignment(TEXT_ALIGN_LEFT);
	oledSetFont(ArialMT_Plain_16);
	sprintf(b, "U: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
	oledDrawString(x , 14 + y, b);
	f = getCubeTemp();
	sprintf(b, "Cube: %0.1f", f);
	oledDrawString(x , 30 + y, b);
	sprintf(b, "P: %d (h: %d)", CurPower, Hpoint);
	oledDrawString(x , 46 + y, b);
}

void addressFrame(OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	wifi_mode_t mode;
	tcpip_adapter_if_t ifx = TCPIP_ADAPTER_IF_AP;
	tcpip_adapter_ip_info_t ip;
	char b[255];

	oledSetTextAlignment(TEXT_ALIGN_LEFT);
	oledSetFont(ArialMT_Plain_10);

	esp_wifi_get_mode(&mode);
	if (WIFI_MODE_STA == mode) ifx = TCPIP_ADAPTER_IF_STA;

	memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
	if (tcpip_adapter_get_ip_info(ifx, &ip) == 0) {
		sprintf(b, "IP:" IPSTR, IP2STR(&ip.ip));
		oledDrawString(x , 13 + y, b);
		sprintf(b, "MASK:" IPSTR, IP2STR(&ip.netmask));
		oledDrawString(x , 24 + y, b);
		sprintf(b, "GW:" IPSTR, IP2STR(&ip.gw));
		oledDrawString(x , 35 + y, b);
	}
	sprintf(b, "Uptime: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
	oledDrawString(x , 46 + y, b);
}

void temperatureFrame(OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	char b[255];
	uint8_t offset=1;

	oledSetTextAlignment(TEXT_ALIGN_LEFT);
	oledSetFont(ArialMT_Plain_10);
	for (int i=0; i<MAX_DS; i++) {
		DS18 *d = &ds[i];
		if (!d->is_connected) continue;
		sprintf(b, " %d %s: %02.1f", d->id+1, getDsTypeStr(d->type), d->Ce);
		oledDrawString(x , offset*11 + y+2, b);
		offset++;
	}

}


// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = {mainFrame, addressFrame, temperatureFrame};
// how many frames are there?
int frameCount = 3;




void display_task(void *pvParameter)
{
	char b[80];

	if (lcd_type == LCD_TYPE_ILI) {
		Tft->setTextColor(ILI9341_WHITE);
		Tft->setTextSize(2);
	} else {
		setTargetFPS(10);
		setIndicatorPosition(TOP);
		setIndicatorDirection(LEFT_RIGHT);
		setFrameAnimation(SLIDE_LEFT);
		setFrames(frames, frameCount);
		oledFlipScreenVertically();
	}

	while (1) {
		if (lcd_type == LCD_TYPE_ILI) {
			sprintf(b, "Uptime: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
			Tft->fillRect(0, 0, 320, 22, ILI9341_BLACK);
			Tft->setCursor(0, 0);
			Tft->printf("%s", b);

			sprintf(b, "Cube: %0.1f", getCubeTemp());
			Tft->fillRect(0, 23, 320, 22, ILI9341_BLACK);
			Tft->setCursor(0, 23);
			Tft->printf("%s", b);

			sprintf(b, "P: %d (h: %d)", CurPower, Hpoint);
			Tft->fillRect(0, 46, 320, 22, ILI9341_BLACK);
			Tft->setCursor(0, 46);
			Tft->printf("%s", b);
			vTaskDelay(2000/portTICK_PERIOD_MS);
		} else {
			UIupdate();
			vTaskDelay(100/portTICK_PERIOD_MS);
		}

	}
}

extern "C"
{

// Запуск обработчика дисплея
int hd_display_init()	
{
	/* Настройка SPI */
	spi_setup();

	if (0 && lcd_type == LCD_TYPE_ILI) {
		ESP_LOGI(TAG, "Init ILI9341 display");
		Tft = new(Adafruit_ILI9341);
		Tft->begin();
		Tft->setRotation(1);
		Tft->fillScreen(ILI9341_BLACK);
	} else {
        	// none-zero, ST
		if (I2C_detect[DISPLAY_I2C_ADDR]) Display_Init(DISPLAY_I2C_ADDR);
		else Display_Init(0);
        }
	/* Запуск отображения на дисплее */
	xTaskCreate(&display_task, "display_task", 2048, NULL, 1, NULL);
	return 0;
}

}