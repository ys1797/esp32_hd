
#ifndef true
#define true 1
#define false 0
#endif


#define TAG "hd_esp32"

#define MAX_OBJ_NAME	32
#define MAX_KLP		4		// Максимальное число клапанов = 4

#define CONFIG_REALM "ESP32"
#define DEFAULT_HOST "hd_esp32"
#define DEFAULT_USERNAME "yuri"
#define DEFAULT_PASSWORD "12345"

#define FIRMWARE_UPDATE_URL "http://hd.rus.net/"
#define ESP32_VERSION "0.1.0"

#define WiFiStateConnected      0
#define WiFiStateWaitToConnect  1
#define WiFiStateConnecting     2
#define WiFiStateDisconnected   3

#define TIME_WAIT_TO_CONNECT 20000
#define TIME_RECONNECT_TIMEOUT 20000

#define NET_CONFIGURATION  "/s/net.cfg"
#define WIFI_CONFIGURATION  "/s/wifi.cfg"
#define RECT_CONFIGURATION  "/s/rectparam.json"
#define SENS_CONFIGURATION  "/s/sensors.json"
#define HISTORY_PATH "/s/history.txt"
#define KEY_CONNECTION_INFO "connectionInfo"    // Key used in NVS for connection info
#define WIFI_NAMESPACE  "hdwifi"		// Namespace in NVS for wifi
#define SSID_SIZE           (32)                // Maximum SSID size
#define PASSWORD_SIZE       (64)                // Maximum password size

/* GPIO */
#define I2C_MASTER_SCL_IO    	4    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    	5    /*!< gpio number for I2C master data  */

#define PIN_DS18B20 		16

#define GPIO_DETECT_ZERO	15 
#define GPIO_TRIAC		2 
#define GPIO_BEEP		0 

#define PZEM_TXD  (14)
#define PZEM_RXD  (13)

#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_DC   21
#define PIN_NUM_RST  25
