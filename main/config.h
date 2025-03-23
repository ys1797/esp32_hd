/*
esp32_hd configuration
Copyright (c) 2018-2025 ys1797 (yuri@rus.net)

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

#ifndef true
#define true 1
#define false 0
#endif


#define TAG "hd_esp32"

#define DEBUG				// debug mode

#define MAX_OBJ_NAME		(32)
#define MAX_KLP			(4)	// Максимальное число клапанов = 4

#define CONFIG_REALM		"ESP32"
#define DEFAULT_HOST		"hd_esp32"
#define DEFAULT_USERNAME	"yuri"
#define DEFAULT_PASSWORD	"12345"

#define FIRMWARE_UPDATE_URL	"http://hd.rus.net/"
#define ESP32_VERSION		"0.8.1"

#define WiFiStateConnected      (0)
#define WiFiStateWaitToConnect  (1)
#define WiFiStateConnecting     (2)
#define WiFiStateDisconnected   (3)

#define TIME_WAIT_TO_CONNECT 	(20000)
#define TIME_RECONNECT_TIMEOUT	(20000)

#define NET_CONFIGURATION	"/s/net.cfg"
#define WIFI_CONFIGURATION	"/s/wifi.cfg"
#define RECT_CONFIGURATION	"/s/rectparam.json"
#define SENS_CONFIGURATION	"/s/sensors.json"
#define HISTORY_PATH		"/s/history.txt"
#define KEY_CONNECTION_INFO	"connectionInfo"	// Key used in NVS for connection info
#define WIFI_NAMESPACE		"hdwifi"		// Namespace in NVS for wifi
#define SSID_SIZE		(32)                	// Maximum SSID size
#define PASSWORD_SIZE		(64)                	// Maximum password size

#define DISP_TYPE_SH1106	(0)
#define DISP_TYPE_ILI9341	(1)
#define DISP_TYPE_ILI9488	(2)
#define DISP_TYPE_ST7789V	(3)
#define DISP_TYPE_ST7735	(4)
#define DISP_TYPE_ST7735R	(5)
#define DISP_TYPE_ST7735B	(6)
#define DISP_TYPE_MAX		(7)


/* GPIO */
#define I2C_MASTER_SCL_IO    	(4)			// gpio number for I2C master cloc
#define I2C_MASTER_SDA_IO    	(5)			//  gpio number for I2C master data

#define PIN_DS18B20 		(16)			// Pin used by onwire ds18b20 in gpio mode

#define GPIO_DETECT_ZERO	(15) 			// Zero cross voltage detect pin
#define GPIO_TRIAC		(2)			// triak controll pin
#define GPIO_BEEP		(17) 			// Beeper gpio number
#define GPIO_ALARM		(10)			// External alarm gpio#

#define PZEM_TXD		(14)			// Pzem tx serial data pin
#define PZEM_RXD		(13)			// Pzem rx serial data pin

#define SPI_PIN_MOSI		(23)
#define SPI_PIN_MISO		(19)
#define SPI_PIN_CLK		(18)
#define SPI_PIN_DC		(21)
#define SPI_PIN_CS		(22)
#define SPI_PIN_RST		(25)

// Тип переменной
enum variable_type {
        VARIABLE_CHECKBOX,
        VARIABLE_STRING,
        VARIABLE_INT,
	VARIABLE_FLOAT
};

// Определение переменной
typedef struct  {
        char *name;
        enum variable_type type;
        int min;
        int max;
	char *default_val;
	char *val;
} vaiable_list;

extern	vaiable_list NET_PARAMS[];	// Сетевые параметры
extern	vaiable_list DEFL_PARAMS[];	// Пераметры устройства

int 	param_default(vaiable_list list[], const char *fname);	// Сброс параметров в значение по умолчанию
int 	param_load(vaiable_list list[], const char *finename);	// Загрузка и установка параметров работы
int 	param_save(vaiable_list list[], const char *finename);	// Сохранение параметров работы
int 	checkParam(vaiable_list list[], char *name);		// Проверка существования параметра
int 	setParam(vaiable_list list[], char *name, char *value); // Установка  переменной
char 	*getStringParam(vaiable_list list[], char *name);	// Получение текстовой переменной
int  	getIntParam(vaiable_list list[], char *name);		// Получение переменной типа int
float	getFloatParam(vaiable_list list[], char *name);		// Получение переменной типа float

