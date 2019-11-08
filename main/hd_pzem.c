/*
esp32_hd
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
#include "freertos/portmacro.h"
#include "freertos/xtensa_api.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "driver/ledc.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "soc/ledc_struct.h"
#include "math.h"
#include "time.h"
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>
#include <cJSON.h>
#include "lwip/apps/sntp.h"
#include "esp_platform.h"
#include "config.h"
#include "hd_main.h"


// PZEM command and responce
struct PZEMCommand {
	uint8_t command;
	uint8_t addr[4];
	uint8_t data;
	uint8_t crc;
};


// PZEM v3 Measured values
struct {
	float voltage;
	float current;
	float power;
	float energy;
	float frequeny;
	float pf;
	uint16_t alarms;
} PZEMv3_Values; 


#define PZEM_VOLTAGE (uint8_t)0xB0
#define RESP_VOLTAGE (uint8_t)0xA0
#define PZEM_CURRENT (uint8_t)0xB1
#define RESP_CURRENT (uint8_t)0xA1
#define PZEM_POWER   (uint8_t)0xB2
#define RESP_POWER   (uint8_t)0xA2
#define PZEM_ENERGY  (uint8_t)0xB3
#define RESP_ENERGY  (uint8_t)0xA3
#define PZEM_SET_ADDRESS (uint8_t)0xB4
#define RESP_SET_ADDRESS (uint8_t)0xA4
#define PZEM_POWER_ALARM (uint8_t)0xB5
#define RESP_POWER_ALARM (uint8_t)0xA5

#define PZEM_RESPONSE_SIZE sizeof(struct PZEMCommand)
#define PZEM_RESPONSE_DATA_SIZE PZEM_RESPONSE_SIZE - 2

#define PZEM_BAUD_RATE 9600
#define PZEM_DEFAULT_READ_TIMEOUT 1000

#define PZEM_ERROR_VALUE -1.0


/* version 3.0 constant */
#define REG_VOLTAGE     0x0000
#define REG_CURRENT_L   0x0001
#define REG_CURRENT_H   0X0002
#define REG_POWER_L     0x0003
#define REG_POWER_H     0x0004
#define REG_ENERGY_L    0x0005
#define REG_ENERGY_H    0x0006
#define REG_FREQUENCY   0x0007
#define REG_PF          0x0008
#define REG_ALARM       0x0009

#define CMD_RHR         0x03
#define CMD_RIR         0X04
#define CMD_WSR         0x06
#define CMD_CAL         0x41
#define CMD_REST        0x42


#define WREG_ALARM_THR   0x0001
#define WREG_ADDR        0x0002

#define UPDATE_TIME     200

#define RESPONSE_SIZE 32
#define READ_TIMEOUT 100

#define PZEM_DEFAULT_ADDR    0xF8


uint8_t PZEM_ip[4] = {192,168,1,1};
uint8_t PZEMv3_addr = PZEM_DEFAULT_ADDR;	// Device address
uint8_t PZEM_Version = 0;			// Device version 3.0 in use ?





/* Расчет CRC для pzem */
static uint8_t PZEM_crc8(uint8_t *data, uint8_t sz)
{
	uint16_t crc = 0;
	uint8_t i;
	for (i=0; i<sz; i++) crc += *data++;
	return (uint8_t)(crc & 0xFF);
}

static uint16_t CRC16(const uint8_t *data, uint16_t len)
{
    // Pre computed CRC table
    static const uint16_t crcTable[] = {
        0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
        0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
        0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
        0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
        0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
        0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
        0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
        0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
        0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
        0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
        0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
        0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
        0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
        0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
        0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
        0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
        0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
        0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
        0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
        0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
        0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
        0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
        0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
        0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
        0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
        0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
        0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
        0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
        0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
        0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
        0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
        0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

    uint8_t nTemp; // CRC table index
    uint16_t crc = 0xFFFF; // Default value

    while (len--)
    {
        nTemp = *data++ ^ crc;
        crc >>= 8;
        crc ^= crcTable[nTemp];
    }
    return crc;
}

static bool PZEM_checkCRC16(const uint8_t *buf, uint16_t len)
{
    if(len <= 2) return false; // Sanity check
    uint16_t crc = CRC16(buf, len - 2); // Compute CRC of data
    return ((uint16_t)buf[len-2]  | (uint16_t)buf[len-1] << 8) == crc;
}

static void PZEM_setCRC16(uint8_t *buf, uint16_t len)
{
    if(len <= 2) return; // Sanity check
    uint16_t crc = CRC16(buf, len - 2); // CRC of data

    // Write high and low byte to last two positions
    buf[len - 2] = crc & 0xFF; // Low byte first
    buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
}


static int PZEM_command(uint8_t cmd, uint8_t data, uint8_t resp, uint8_t *result)
{
	struct PZEMCommand pzem;
	uint8_t *bytes;
	uint8_t buf[PZEM_RESPONSE_SIZE];
	int len, i, readed = 0, ret = -2;

	pzem.command = cmd;
        for(i=0; i<sizeof(pzem.addr); i++) pzem.addr[i] = PZEM_ip[i];
        pzem.data = data;
        bytes = (uint8_t*)&pzem;
        pzem.crc = PZEM_crc8(bytes, sizeof(pzem) - 1);

	uart_flush(UART_NUM_1);
	uart_write_bytes(UART_NUM_1, (const char*) bytes, sizeof(pzem));

	for (i = 0; i < 2; i++) {
		len = uart_read_bytes(UART_NUM_1, &buf[readed], sizeof(buf)-readed,
			PZEM_DEFAULT_READ_TIMEOUT / portTICK_RATE_MS);
                readed += len;
                if (readed < PZEM_RESPONSE_SIZE) continue;
                if (buf[6] != PZEM_crc8(buf, readed-1)) {
                        ret = -1;
                        break;
                }
                if (buf[0] != resp) {
                        ret = -1;
                        break;
                }
		if (result) {
                        for (i=0; i<PZEM_RESPONSE_DATA_SIZE; i++) {
                                result[i] = buf[1 + i];
                        }
                }
                ret = 0;
                break;
	}
	return ret;
}

static int PZEMv30_recieve(uint8_t *resp, uint16_t size)
{
	int len, i, readed = 0;

	for (i = 0; i < 2; i++) {
		len = uart_read_bytes(UART_NUM_1, &resp[readed], size-readed,
			PZEM_DEFAULT_READ_TIMEOUT / portTICK_RATE_MS);
	        readed += len;
		if (readed < size) continue;

		// Check CRC with the number of bytes read
		if (!PZEM_checkCRC16(resp, readed)) return 0;
	}
	return readed;
}

/*!
 * Prepares the 8 byte command buffer and sends
 *
 * @param[in] cmd - Command to send (position 1)
 * @param[in] rAddr - Register address (postion 2-3)
 * @param[in] val - Register value to write (positon 4-5)
 * @param[in] check - perform a simple read check after write
 *
 * @return success
*/
static int PZEMv30_sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check){
	uint8_t sendBuffer[8]; // Send buffer
	uint8_t respBuffer[8]; // Response buffer (only used when check is true)

	sendBuffer[0] = PZEMv3_addr;             // Set slave address
	sendBuffer[1] = cmd;                     // Set command
	sendBuffer[2] = (rAddr >> 8) & 0xFF;     // Set high byte of register address
	sendBuffer[3] = (rAddr) & 0xFF;          // Set low byte =//=
	sendBuffer[4] = (val >> 8) & 0xFF;       // Set high byte of register value
	sendBuffer[5] = (val) & 0xFF;            // Set low byte =//=
	PZEM_setCRC16(sendBuffer, 8);                   // Set CRC of frame

	uart_flush(UART_NUM_1);
	uart_write_bytes(UART_NUM_1, (const char*) sendBuffer, 8); // send frame

	if (check) {
		// if check enabled, read the response
	        if (!PZEMv30_recieve(respBuffer, 8)) return false;

		// Check if response is same as send
		for (int i = 0; i < 8; i++){
	            if (sendBuffer[i] != respBuffer[i]) return false;
		}
	}
	return true;
}


float PZEM_setAddress(void)
{
	return PZEM_command(PZEM_SET_ADDRESS, 0, RESP_SET_ADDRESS, NULL);
}

float PZEM_voltage(void)
{
        uint8_t data[PZEM_RESPONSE_DATA_SIZE];
        if (PZEM_command(PZEM_VOLTAGE, 0, RESP_VOLTAGE, data)) {
                return PZEM_ERROR_VALUE;
        }
        return (data[0] << 8) + data[1] + (data[2] / 10.0);
}

float PZEM_current(void)
{
        uint8_t data[PZEM_RESPONSE_DATA_SIZE];
        if (PZEM_command(PZEM_CURRENT, 0, RESP_CURRENT, data)) {
                return PZEM_ERROR_VALUE;
        }
        return (data[0] << 8) + data[1] + (data[2] / 100.0);
}

float PZEM_power(void)
{
        uint8_t data[PZEM_RESPONSE_DATA_SIZE];
        if (PZEM_command(PZEM_POWER, 0, RESP_POWER, data)) {
                return PZEM_ERROR_VALUE;
        }
        return (data[0] << 8) + data[1];
}

float PZEM_energy(void)
{
        uint8_t data[PZEM_RESPONSE_DATA_SIZE];
        if (PZEM_command(PZEM_ENERGY, 0, RESP_ENERGY, data)) {
                return PZEM_ERROR_VALUE;
        }
        return ((uint32_t)data[0] << 16) + ((uint16_t)data[1] << 8) + data[2];
}




void PZEM_init(void)
{
	uint8_t addr;
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 122,
	};
	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, PZEM_TXD, PZEM_RXD, -1, -1);
	uart_driver_install(UART_NUM_1, UART_FIFO_LEN+2, 0, 0, NULL, 0);


	PZEM_Version = getIntParam(DEFL_PARAMS, "pzemVersion");
	PZEMv3_addr = PZEM_DEFAULT_ADDR; // Sanity check of address

	if (!PZEM_Version) {
		// Old protocol
		PZEM_setAddress();
	} else {
	}

}

bool PZEMv30_setAddress(uint8_t addr)
{
	if (addr < 0x01 || addr > 0xF7) return false; // sanity check

	// Write the new address to the address register
	if (!PZEMv30_sendCmd8(CMD_WSR, WREG_ADDR, addr, true)) return false;
	PZEMv3_addr = addr; // If successful, update the current slave address
	return true;
}

int8_t PZEMv30_getAddress()
{
	return PZEMv3_addr;
}


bool PZEMv30_setPowerAlarm(uint16_t watts)
{
	if (watts > 25000) {
		// Sanitych check
		watts = 25000;
	}

	// Write the watts threshold to the Alarm register
	if (!PZEMv30_sendCmd8(CMD_WSR, WREG_ALARM_THR, watts, true)) return false;
	return true;
}


/*!
 *
 * Read all registers of device and update the local values
 *
 * @return success
*/
bool PZEMv30_updateValues(void)
{
	// static uint8_t buffer[] = {0x00, CMD_RIR, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00};
	static uint8_t response[25];

	// Read 10 registers starting at 0x00 (no check)
	PZEMv30_sendCmd8(CMD_RIR, 0x00, 0x0A, false);


	if (PZEMv30_recieve(response, 25) != 25) {
		// Something went wrong
		return false;
	}



	// Update the current values
	PZEMv3_Values.voltage = ((uint32_t)response[3] << 8 | // Raw voltage in 0.1V
                              (uint32_t)response[4])/10.0;

	PZEMv3_Values.current = ((uint32_t)response[5] << 8 | // Raw current in 0.001A
                              (uint32_t)response[6] |
                              (uint32_t)response[7] << 24 |
                              (uint32_t)response[8] << 16) / 1000.0;

	PZEMv3_Values.power =   ((uint32_t)response[9] << 8 | // Raw power in 0.1W
                              (uint32_t)response[10] |
                              (uint32_t)response[11] << 24 |
                              (uint32_t)response[12] << 16) / 10.0;

	PZEMv3_Values.energy =  ((uint32_t)response[13] << 8 | // Raw Energy in 1Wh
                              (uint32_t)response[14] |
                              (uint32_t)response[15] << 24 |
                              (uint32_t)response[16] << 16) / 1000.0;

	PZEMv3_Values.frequeny =((uint32_t)response[17] << 8 | // Raw Frequency in 0.1Hz
                              (uint32_t)response[18]) / 10.0;

	PZEMv3_Values.pf    = ((uint32_t)response[19] << 8 | // Raw pf in 0.01
                              (uint32_t)response[20])/100.0;

 	PZEMv3_Values.alarms =((uint32_t)response[21] << 8 | // Raw alarm value
				(uint32_t)response[22]);

 	 return true;
}