/*
esp32_hd bmp85/bmp180 interface
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
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include <unistd.h>
#include "hd_spi_i2c.h"

/* Operating Modes */
#define BMP085_ULTRALOWPOWER    0
#define BMP085_STANDARD         1
#define BMP085_HIGHRES          2
#define BMP085_ULTRAHIGHRES     3

/* BMP085 Registers */
#define BMP085_CAL_AC1          0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2          0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3          0xAE  // R   Calibration data (16 bits)
#define BMP085_CAL_AC4          0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5          0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6          0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1           0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2           0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB           0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC           0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD           0xBE  // R   Calibration data (16 bits)
#define BMP085_CONTROL          0xF4
#define BMP085_TEMPDATA         0xF6
#define BMP085_PRESSUREDATA     0xF6

// BMP085 Modes
#define MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define MODE_STANDARD           1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define MODE_HIGHRES            2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define MODE_ULTRA_HIGHRES      3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25


 /*
  *  "Sampling rate can be increased to 128 samples per second
  *  (standard mode) for dynamic measurement.In this case
  *  it is sufficient to measure temperature only once per second
  *  and to use this value for all pressure measurements during period."
  *  (from BMP085 datasheet Rev1.2 page 10).
  *  To use dynamic measurement set AUTO_UPDATE_TEMPERATURE
  *  to false and call calcTrueTemperature() from your code.
  */

// Control register
#define READ_TEMPERATURE        0x2E
#define READ_PRESSURE           0x34


#define BMP_DEBUG 0

int16_t cal_AC1;
int16_t cal_AC2;
int16_t cal_AC3;
uint16_t cal_AC4;
uint16_t cal_AC5;
uint16_t cal_AC6;
int16_t cal_B1;
int16_t cal_B2;
int16_t cal_MB;
int16_t cal_MC;
int16_t cal_MD;

int pressure_waittime[4];
int32_t cm_Offset, Pa_Offset;
long oldEMA;

static uint8_t	bmp180_Address = 0x77;
static int 	BMP085_mode = 3;
double bmpTemperature = -1;
double bmpTruePressure = -1;



static int set_i2c_register(uint8_t reg , uint8_t value)
{
	esp_err_t ret;
	uint8_t buf[4];
	buf[0] = reg;
	buf[1] = value;
	ret = I2CWrite(bmp180_Address, buf, 2);
	if (ret == ESP_FAIL) {
		printf("set_i2c_register write fail\n");
		return 1;
	}
	vTaskDelay(10/portTICK_PERIOD_MS);
	return 0;
}

static int get_i2c_register(uint8_t reg, uint8_t *val)
{
	esp_err_t ret;
	uint8_t buf[4];

	/*
         * In order to read a register, we first do a "dummy write" by writing
         * 0 bytes to the register we want to read from.  This is similar to
         * the packet in set_i2c_register, except it's 1 byte rather than 2.
         */
	buf[0] = reg;
	ret = I2CWrite(bmp180_Address, buf, 1);
	if (ret == ESP_FAIL) {
		printf("get_i2c_register dummy write fail\n");
		return 1;
	}

	ret = I2CRead(bmp180_Address, buf, 0x1);
	if (ret == ESP_FAIL) {
		printf("get_i2c_register read Fail\n");
		return 1;
	}
	*val = buf[0];
	return 0;
}

static int get_i2c_word(uint8_t reg, uint16_t *val)
{
        uint8_t h, l;

	if (get_i2c_register(reg, &h)) {
                printf("BMP085 I2C: Unable to get register (high)!");
                return -1;
        }
        if (get_i2c_register(reg+1, &l)) {
                printf("BMP085 I2C: Unable to get register (low)!");
                return -1;
        }

	*val = (h<<8)+l;

	return 0;
}

static int readCalibrationData(void)
{
#if BMP_DEBUG
	ESP_LOGD(TAG, "BMP085 Reads the calibration data from the IC.\n");
#endif
	if (get_i2c_word(BMP085_CAL_AC1, (uint16_t*)&cal_AC1)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_AC1");
                return -1;
        }
	if (get_i2c_word(BMP085_CAL_AC2, (uint16_t*)&cal_AC2)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_AC2");
                return -1;
        }

	if( get_i2c_word(BMP085_CAL_AC3, (uint16_t*)&cal_AC3)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_AC3");
                return -1;
        }
        if (get_i2c_word(BMP085_CAL_AC4, (uint16_t*)&cal_AC4)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_AC4");
                return -1;
        }
	if (get_i2c_word(BMP085_CAL_AC5, (uint16_t*)&cal_AC5)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_AC5");
                return -1;
        }
        if (get_i2c_word(BMP085_CAL_AC6, (uint16_t*)&cal_AC6)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_AC6");
                return -1;
        }
	if (get_i2c_word(BMP085_CAL_B1, (uint16_t*)&cal_B1)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_B1");
                return -1;
        }
        if (get_i2c_word(BMP085_CAL_B2, (uint16_t*)&cal_B2)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_B2");
                return -1;
        }
	if (get_i2c_word(BMP085_CAL_MB, (uint16_t*)&cal_MB)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_MB");
                return -1;
        }
        if (get_i2c_word(BMP085_CAL_MC, (uint16_t*)&cal_MC)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_MC");
                return -1;
        }
	if (get_i2c_word(BMP085_CAL_MD, (uint16_t*)&cal_MD)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get CAL_MD");
                return -1;
        }
	return 0;
}

static int showCalibrationData(void)
{
#if BMP_DEBUG
        ESP_LOGI(TAG, "Displays the calibration values for debugging purposes");
        ESP_LOGI(TAG, "DBG: AC1: %6d", cal_AC1);
        ESP_LOGI(TAG, "DBG: AC2: %6d", cal_AC2);
        ESP_LOGI(TAG, "DBG: AC3: %6d", cal_AC3);
        ESP_LOGI(TAG, "DBG: AC4: %6d", cal_AC4);
        ESP_LOGI(TAG, "DBG: AC5: %6d", cal_AC5);
        ESP_LOGI(TAG, "DBG: AC6: %6d", cal_AC6);
        ESP_LOGI(TAG, "DBG:  B1: %6d", cal_B1);
        ESP_LOGI(TAG, "DBG:  B2: %6d", cal_B2);
        ESP_LOGI(TAG, "DBG:  MB: %6d", cal_MB);
        ESP_LOGI(TAG, "DBG:  MC: %6d", cal_MC);
        ESP_LOGI(TAG, "DBG:  MD: %6d", cal_MD);
#endif
        return 0;
}

static long readRawTemp(void)
{
        long raw = 0;
#if BMP_DEBUG
        ESP_LOGD(TAG, "Reads the raw (uncompensated) temperature from the sensor");
#endif
	if (set_i2c_register(BMP085_CONTROL, READ_TEMPERATURE)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get BMP085_CONTROL");
                return -1;
        }
        if (get_i2c_word(BMP085_TEMPDATA, (uint16_t*)&raw)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get BMP085_TEMPDATA");
                return -1;
        }
#if BMP_DEBUG
        ESP_LOGD(TAG, "DBG: Raw Temp: 0x%04lX (%ld)", raw & 0xFFFF, raw);
#endif
        return raw;
}

static long readRawPressure(void)
{
        long raw;
        unsigned char msb, lsb, xlsb;

#if BMP_DEBUG
	ESP_LOGD(TAG, "Reads the raw (uncompensated) pressure level from the sensor");
#endif

	if (set_i2c_register(BMP085_CONTROL, READ_PRESSURE+(BMP085_mode<<6) )) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get BMP085_CONTROL");
                return -1;
        }
        usleep(pressure_waittime[BMP085_mode]);

	if (get_i2c_register(BMP085_PRESSUREDATA, &msb)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get BMP085_PRESSUREDATA");
                return -1;
        }
        if (get_i2c_register(BMP085_PRESSUREDATA+1, &lsb)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get BMP085_PRESSUREDATA");
                return -1;
        }
        if (get_i2c_register(BMP085_PRESSUREDATA+2, &xlsb)) {
                ESP_LOGE(TAG, "BMP085 I2C: unable to get BMP085_PRESSUREDATA");
                return -1;
        }

	// uncompensated pressure value
        raw = ((((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >> (8-BMP085_mode));

#if BMP_DEBUG
        ESP_LOGD(TAG, "DBG: Raw Pressure: 0x%04lX (%ld)",raw & 0xFFFF, raw);
#endif
        return raw;
}

double getTemperature(void) {
        long ut, x1, x2, b5;

        /* Read raw temp before aligning it with the calibration values */
        ut = readRawTemp();
        if (ut == -1) return -1;
        x1 = ((ut-cal_AC6) * cal_AC5) >> 15;
        x2 = (cal_MC << 11) / (x1 + cal_MD);
        b5 = x1 + x2;
        return ((b5 + 8) >> 4) /10.0;
}

double readPressure(void)
{
	long ut, up, x1, x2, x3, b3, b5, b6, p;
        unsigned long b4, b7;
        int32_t tmp;

        // Calculate b5 (True Temperature Calculations)
        ut = readRawTemp();
        if (ut == -1) return -1;
        x1 = ((ut-cal_AC6) * cal_AC5) >> 15;
        x2 = (cal_MC << 11) / (x1 + cal_MD);
        b5 = x1 + x2;

#if BMP_DEBUG
        ESP_LOGD(TAG, "DBG: True temperature:%.2f C", ((b5 + 8) >> 4) / 10.0);
#endif

        up = readRawPressure();

	// calculate true pressure
        b6 = b5 - 4000; // b5 is updated before
        x1 = (cal_B2 * (b6 * b6) >> 12) >> 11;
        x2 = (cal_AC2 * b6) >> 11;
        x3 = x1 + x2;
        tmp = cal_AC1;
        tmp = (tmp * 4 + x3) << BMP085_mode;
        b3 = (tmp + 2) >> 2;
/*
	printf("DBG: B6 = %d\n", b6);
	printf("DBG: X1 = %d\n", x1);
	printf("DBG: X2 = %d\n", x2);
	printf("DBG: X3 = %d\n", x3);
	printf("DBG: B3 = %d\n", b3);
*/
	x1 = (cal_AC3 * b6) >> 13;
        x2 = (cal_B1 * ((b6 * b6) >> 12)) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        b4 = (cal_AC4 * (uint32_t)(x3 + 32768)) >> 15;
        b7 = ((uint32_t)up - b3) * (50000 >> BMP085_mode);

/*
	printf("DBG: X1 = %d\n", x1);
	printf("DBG: X2 = %d\n", x2);
	printf("DBG: X3 = %d\n", x3);
	printf("DBG: B4 = %d\n", b4);
	printf("DBG: B7 = %d\n", b7);
*/
        p = (b7 < 0x80000000) ? (b7<<1) / b4 : (b7 / b4)<<1;
        x1 = (p >> 8) * (p >> 8);
        x1 = (x1 * 3038) >> 16;
        x2 = (-7357 * p) >> 16;
/*
	printf("DBG: p  = %d\n", p);
	printf("DBG: X1 = %d\n", x1);
	printf("DBG: X2 = %d\n", x2);
*/
        return  p + ((x1 + x2 + 3791) >> 4);
}

void bmp_task(void *arg)
{
	while(1) {
		bmpTemperature = getTemperature();
		bmpTruePressure = readPressure();

		vTaskDelay(1000/portTICK_PERIOD_MS);
	}

}


int initBMP085(void)
{
	pressure_waittime[0] = 5000; // These are maximum convertion times. (us)
        pressure_waittime[1] = 8000; // It is possible to use pin EOC (End Of Conversion)
        pressure_waittime[2] = 14000;// to check if conversion is finished (logic 1)
        pressure_waittime[3] = 26000;// or running (logic 0) insted of waiting for convertion times.
        cm_Offset = 0;
        Pa_Offset = 0;               // 1hPa = 100Pa = 1mbar
        oldEMA = 0;

	readCalibrationData();
	showCalibrationData();


	xTaskCreate(&bmp_task, "bmp_task", 4096, NULL, 1, NULL);

	ESP_LOGI(TAG, "BMP085 service started");
        return 0;
}