// Combined include file for esp8266
// Actually misnamed, as it also works for ESP32.
// ToDo: Figure out better name


#define FREERTOS 1
#define ESP32 1
#include "sdkconfig.h"
#define HTTPD_MAX_CONNECTIONS CONFIG_ESPHTTPD_MAX_CONNECTIONS
#define HTTPD_STACKSIZE CONFIG_ESPHTTPD_STACK_SIZE
#include "stdint.h"
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;

#define ICACHE_RODATA_ATTR


#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_spi_flash.h"


#include "platform.h"
#include "espmissingincludes.h"

