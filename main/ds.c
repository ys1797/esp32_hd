/*
ds18b20 interface
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
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp32/rom/ets_sys.h"
#include <sys/stat.h>
#include "argtable3/argtable3.h"
#include <cJSON.h>
#include <string.h>
#include "config.h"
#include "hd_main.h"
#include "hd_spi_i2c.h"
#include "ds.h"


#define EXAMPLE_ONEWIRE_MAX_DS18B20 2

#ifndef max
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#endif

/* DS2482 */
#define PTR_STATUS 0xf0
#define PTR_READ 0xe1
#define PTR_CONFIG 0xc3

#define DS2482_CONFIG_APU (1<<0)
#define DS2482_CONFIG_PPM (1<<1)
#define DS2482_CONFIG_SPU (1<<2)
#define DS2484_CONFIG_WS  (1<<3)

#define DS2482_STATUS_BUSY 	(1<<0)
#define DS2482_STATUS_PPD 	(1<<1)
#define DS2482_STATUS_SD	(1<<2)
#define DS2482_STATUS_LL	(1<<3)
#define DS2482_STATUS_RST	(1<<4)
#define DS2482_STATUS_SBR	(1<<5)
#define DS2482_STATUS_TSB	(1<<6)
#define DS2482_STATUS_DIR	(1<<7)
// -----commands
#define DS2482_CMD_RESET 	0xF0
#define DS2482_CMD_SET_PTR 	0xE1
#define DS2482_CMD_WRITE_CFG 	0xD2
#define DS2482_CMD_1W_RESET 	0xB4
#define DS2482_CMD_1W_WBIT	0x87
#define DS2482_CMD_1W_WBYTE	0xA5
#define DS2482_CMD_1W_RBYTE	0x96
// for Search-ROM. generates 2 Read-slot+1write slot
#define DS2482_CMD_1W_TRIPLET	0x78
//#define DS2482_CMD_ 0x
//#define DS2482_CMD_0x

enum {
	DS18S20 = (uint8_t) 0x10,
	DS18B20 = (uint8_t) 0x28,
	DS1822	= (uint8_t) 0x22,
	DS1825	= (uint8_t) 0x3B
} TemperatureSensors;

enum {
	DS2406 	= (uint8_t) 0x12
} SwitchSensors;

static uint8_t ds2482_Address;
static uint8_t ds2482_Timeout;
static uint8_t searchAddress[8]; 
static uint8_t searchLastDisrepancy; 
static uint8_t searchExhausted; 


DS18		ds[MAX_DS];             // ds18b20 devices
uint8_t		ds18_devices;	// count of devices on the bus
uint8_t		maxBitResolution = 9;

onewire_bus_handle_t	ow_bus;

uint8_t emulate_devices=0;	// –†–µ–∂–∏–º —ç–º—É–ª—è—Ü–∏–∏ —Ç–µ–º–ø–µ—Ä–∞—Ç—É—Ä—ã
double testCubeTemp = 20;	// –¢–µ—Å—Ç–æ–≤–æ–µ –∑–Ω–∞—á–µ–Ω–∏–µ –∫—É–±–æ–≤–æ–π —Ç–µ–º–ø–µ—Ä–∞—Ç—É—Ä—ã

void ow_init(int GPIO)
{
	DS18 *d;
	FILE *f;
	struct stat st;
	char *buff = NULL;
	size_t size;
	cJSON *root = NULL;
	cJSON *p;

	onewire_bus_config_t bus_config = {
        	.bus_gpio_num = GPIO,
	};
	onewire_bus_rmt_config_t rmt_config = {
        	.max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
	};
	ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &ow_bus));

	onewire_device_iter_handle_t iter = NULL;
	onewire_device_t next_onewire_device;
	esp_err_t search_result = ESP_OK;

	if (0 == stat(SENS_CONFIGURATION, &st) && (f = fopen(SENS_CONFIGURATION, "r"))) {
		buff = malloc(st.st_size+2);
		if (buff) {
			size = fread(buff, 1, st.st_size, f);
			buff[size]=0;
			fclose(f);
			root = cJSON_Parse(buff);
			free(buff);
		}
	}

	// create 1-wire device iterator, which is used for device search
	ESP_ERROR_CHECK(onewire_new_device_iter(ow_bus, &iter));
	ESP_LOGI(TAG, "Device iterator created, start searching...");
	do {
		d = &ds[ds18_devices];
		if (d->is_connected) {
			search_result = ESP_OK;
			continue;
		}
		search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
		if (ESP_OK == search_result) {		
			// found a new device, let's check if we can upgrade it to a DS18B20
			ds18b20_config_t ds_cfg = {};
			// check if the device is a DS18B20, if so, return the ds18b20 handle
			if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &d->handle) == ESP_OK) {
				sprintf(d->address, "%016llX", next_onewire_device.address);				
                		ESP_LOGI(TAG, "Found a DS18B20[%d], address: %s", ds18_devices, d->address);
				d->is_connected = true;
				//search configuration parameters
				for (int i=0; root && i<cJSON_GetArraySize(root); i++) {
					cJSON *ds = cJSON_GetArrayItem(root, i);
					p = cJSON_GetObjectItem(ds, "rom");
					if (p && p->valuestring && !strcmp(d->address, p->valuestring)) {
						p = cJSON_GetObjectItem(ds, "type");
						if (p) d->type = p->valueint;
						if (d->type > DS_UNKNOW) d->type = DS_UNKNOW;
						p = cJSON_GetObjectItem(ds, "corr");
						if (p) d->corr = p->valuedouble;
						p = cJSON_GetObjectItem(ds, "talert");
						if (p) d->talert = p->valuedouble;
						p = cJSON_GetObjectItem(ds, "descr");
						if (p && p->valuestring) d->description = strdup(p->valuestring);
					}
				}
        	                ds18_devices++;
			} else {
				ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
			}
		}
	} while (search_result != ESP_ERR_NOT_FOUND && ds18_devices < MAX_DS);
	ESP_ERROR_CHECK(onewire_del_device_iter(iter));
	cJSON_Delete(root);
	ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", ds18_devices);
}



// convert from raw to Celsius
float rawToCelsius(int16_t raw){

	if (raw <= DEVICE_DISCONNECTED_RAW) return DEVICE_DISCONNECTED_C;
	// C = RAW/128
	return (float)raw * 0.0078125;
}



void ds2482_init(void)
{
	uint8_t buf[2];
	bool detected = false;

	for (ds2482_Address=0x18; ds2482_Address<0x1B; ds2482_Address++) {
		if (I2C_detect[ds2482_Address]) {
			detected = true;
			break;
		}
	}

	if (!detected) {
		ds2482_Address = 0;
		ESP_LOGI(TAG, "NO DS2482 chip detected!");
		return;
	}

	buf[0] = DS2482_CMD_RESET;// 0xf0;
	if (I2CWrite(ds2482_Address, buf, 1) == ESP_FAIL) {
		ESP_LOGW(TAG, "DS2482 reset failed!");
	}
	else
  /*
   * The  Active Pullup (APU) bit  controls  whether  an  active  pullup  (con-trolled  slew-rate  transistor)  or
   * a  passive  pullup  (RWPUresistor) is used to drive a 1-Wire line from low to high.
   * When  APU  =  0,  active  pullup  is  disabled  (resistormode).
   * (!!!)Active pullup should always be selected unlessthere  is  only  a  single  slave  on  the  1-Wire  line
   */
		if (!ds2482_configure(DS2482_CONFIG_APU)) // APU
			ESP_LOGW(TAG, "DS2482 set cfg failed!");

	ESP_LOGI(TAG, "DS2482 init complete, address: 0x%x", ds2482_Address);	
}

static void ds2482_setReadPtr(uint8_t readPtr)
{
	esp_err_t ret;
	uint8_t buf[4];
	buf[0] = DS2482_CMD_SET_PTR; //0xe1;
	buf[1] = readPtr;
	ret = I2CWrite(ds2482_Address, buf, 2);
	if (ret == ESP_FAIL) {
		printf("DS2482 I2C write Fail\n");
	}
}

uint8_t ds2482_readByte() {
	esp_err_t ret;
	uint8_t buf[8];
	ret = I2CRead(ds2482_Address, buf, 0x1);
	if (ret == ESP_FAIL) {
		printf("DS2482 I2C read Fail\n");
		return 0;
	}
	return buf[0];
}

uint8_t ds2482_readStatus(bool setPtr)
{
	if (setPtr) ds2482_setReadPtr(PTR_STATUS);
	return ds2482_readByte();
}


static uint8_t ds2482_busyWait(bool setReadPtr)
{
	uint8_t status;
	int loopCount = 1000;
	while ((status = ds2482_readStatus(setReadPtr)) & DS2482_STATUS_BUSY) {
		if (--loopCount <= 0) {
			ds2482_Timeout = 1;
			break;
		}
		//ets_delay_us(2);
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
	return status;
}

bool ds2482_wireReset()
{
	esp_err_t ret;
	uint8_t buf[4];
	ds2482_busyWait(true);

	buf[0] = DS2482_CMD_1W_RESET; //0xb4;
	ret = I2CWrite(ds2482_Address, buf, 1);
	if (ret == ESP_FAIL) {
		ESP_LOGW(TAG, "DS2482 I2C wire reset fail");
	}
	uint8_t status = ds2482_busyWait(true);
	return status & DS2482_STATUS_PPD ? true : false;
}

bool ds2482_configure(uint8_t config)
{
	esp_err_t ret;
	uint8_t buf[4];

	ds2482_busyWait(true);
	buf[0] = DS2482_CMD_WRITE_CFG;
	/*
	 * (!!!)When  writing  to  the  Configuration  Register,
	 *  the new data is accepted ONLY if the upper nibble (bits 7to 4) is the one‚Äôs complement of the lower nibble (bits 3to 0).
	 *  When read, the upper nibble is always 0h.
	 */
	buf[1] = config | (~config)<<4;

	ret = I2CWrite(ds2482_Address, buf, 2);
	if (ret == ESP_FAIL) {
		printf("DS2482 I2C configure fail\n");
	}
	return ds2482_readByte() == config;
}

bool ds2482_selectChannel(uint8_t channel)
{	
	uint8_t ch, ch_read;
	esp_err_t ret;
	uint8_t buf[4];

	switch (channel) {
	case 0:
	default:  
		ch = 0xf0; 
		ch_read = 0xb8; 
		break;
	case 1: 
		ch = 0xe1; 
		ch_read = 0xb1; 
		break;
	case 2: 
		ch = 0xd2; 
		ch_read = 0xaa; 
		break;
	case 3: 
		ch = 0xc3; 
		ch_read = 0xa3; 
		break;
	case 4: 
		ch = 0xb4; 
		ch_read = 0x9c; 
		break;
	case 5: 
		ch = 0xa5; 
		ch_read = 0x95; 
		break;
	case 6: 
		ch = 0x96; 
		ch_read = 0x8e; 
		break;
	case 7: 
		ch = 0x87; 
		ch_read = 0x87; 
		break;
	};

	ds2482_busyWait(true);
	buf[0] = 0xc3;
	buf[1] = ch;
	ret = I2CWrite(ds2482_Address, buf, 2);
	if (ret == ESP_FAIL) {
		printf("DS2482 I2C select ch fail %d\n", ret);
	}
//	ds2482_busyWait(true);
	uint8_t check = ds2482_readByte();
	return check == ch_read;
}

void ds2482_wireWriteByte(uint8_t b)
{
	esp_err_t ret;
	uint8_t buf[4];
	ds2482_busyWait(true);
	buf[0] = DS2482_CMD_1W_WBYTE;// 0xa5;
	buf[1] = b;
	ret = I2CWrite(ds2482_Address, buf, 2);
	if (ret == ESP_FAIL) {
		printf("DS2482 I2C ow write byte fail\n");
	}
}

uint8_t ds2482_wireReadByte()
{
	esp_err_t ret;
	uint8_t buf[4];
	ds2482_busyWait(true);
	buf[0] = DS2482_CMD_1W_RBYTE; //0x96;
	ret = I2CWrite(ds2482_Address, buf, 1);
	if (ret == ESP_FAIL) {
		printf("DS2482 I2C ow write byte fail\n");
	}
	ds2482_busyWait(true);

	ds2482_setReadPtr(PTR_READ);
	return ds2482_readByte();
}

void ds2482_wireWriteBit(uint8_t bit)
{
	esp_err_t ret;
	uint8_t buf[4];

	ds2482_busyWait(true);
	buf[0] = DS2482_CMD_1W_WBIT; //0x87;
	buf[1] = bit ? 0x80 : 0;
	ret = I2CWrite(ds2482_Address, buf, 1);
	if (ret == ESP_FAIL) {
		printf("DS2482 I2C ow write bit fail\n");
	}

}

uint8_t ds2482_wireReadBit()
{
	ds2482_wireWriteBit(1);
	uint8_t status = ds2482_busyWait(true);
	return status & DS2482_STATUS_SBR ? 1 : 0;
}

void ds2482_wireSkip()
{
	ds2482_wireWriteByte(0xcc);
}

void ds2482_wireSelect(uint8_t rom[8])
{
	ds2482_wireWriteByte(0x55);
	for (int i=0;i<8;i++) {
		ds2482_wireWriteByte(rom[i]);
	}
}

void ds2482_wireResetSearch() 
{ 
	searchExhausted = 0; 
 	searchLastDisrepancy = 0; 
	for (uint8_t i = 0; i<8; i++)  {
		searchAddress[i] = 0;
	}
} 

uint8_t ds2482_wireSearch(uint8_t *newAddr)
{
	uint8_t i;
	uint8_t direction;
	uint8_t last_zero=0;
	esp_err_t ret;
	uint8_t buf[2];

	
	if (searchExhausted) return 0;
	if (!ds2482_wireReset()) return 0;

	ds2482_busyWait(true);
	ds2482_wireWriteByte(0xf0);
	
	for (i=1;i<65;i++) {
		int romByte = (i-1)>>3;
		int romBit = 1<<((i-1)&7);
		
		if (i < searchLastDisrepancy) {
			direction = searchAddress[romByte] & romBit;
		} else {
			direction = i == searchLastDisrepancy;
		}
		
		ds2482_busyWait(true);

		buf[0] = DS2482_CMD_1W_TRIPLET; //0x78; for Search-ROM. generates 2 Read-slot+1write slot
		buf[1] = (direction ? 0x80 : 0);
		ret = I2CWrite(ds2482_Address, buf, 2);
		if (ret == ESP_FAIL) {
			printf("DS2482 I2C search fail\n");
		}
		uint8_t status = ds2482_busyWait(true);
		
		uint8_t id = status & DS2482_STATUS_SBR;
		uint8_t comp_id = status & DS2482_STATUS_TSB;
		direction = status & DS2482_STATUS_DIR;
		
		if (id && comp_id) {
			return 0;
		} else { 
			if (!id && !comp_id && !direction) last_zero = i;
		}
		
		if (direction) {
			searchAddress[romByte] |= romBit;
		} else {
			searchAddress[romByte] &= (uint8_t)~romBit;
		}
	}

	searchLastDisrepancy = last_zero;

	if (last_zero == 0) searchExhausted = 1;
	
	for (i=0;i<8;i++)  {
		newAddr[i] = searchAddress[i];
	}
	
	return 1;  
}

uint8_t ds2482_devicesCount(bool printAddress)
{
	uint8_t address[8];
	uint8_t count = 0;
	uint8_t	i;

	ds2482_wireResetSearch();
	while (ds2482_wireSearch(address)) {
		count++;
		if (printAddress){
			printf("found: ");
			for (i=0; i<8; i++) {
				printf("%02X", address[i]);
				if (i < 7) printf("-");
			}
			printf("\n");
		}
	}
	return count;
}

/*
 * Compute a Dallas Semiconductor 8 bit CRC directly.
 */
uint8_t ow_crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;
        while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

// returns true if address is valid
static bool validAddress(const uint8_t* deviceAddress)
{
       return (ow_crc8(deviceAddress, 7) == deviceAddress[7]);
}


static bool validFamily(const uint8_t* deviceAddress)
{
	switch (deviceAddress[0]) {
        case DS18S20MODEL:
        case DS18B20MODEL:
        case DS1822MODEL:
        case DS1825MODEL:
        case DS28EA00MODEL:
		return true;
        default:
		return false;
	}
}


// initialise the bus
int ds_init(int argc, char** argv)
{
	uint8_t i;
	DS18 *d;
	DeviceAddress deviceAddress;
	FILE *f;
	struct stat st;
	char *buff = NULL;
	size_t size;
	cJSON *root = NULL;
	cJSON *p;

	ds18_devices = 0;

	if (!ds2482_Address) return -1;



	if (0 == stat(SENS_CONFIGURATION, &st) && (f = fopen(SENS_CONFIGURATION, "r"))) {
		buff = malloc(st.st_size+2);
		if (buff) {
			size = fread(buff, 1, st.st_size, f);
			buff[size]=0;
			fclose(f);
			root = cJSON_Parse(buff);
			free(buff);
		}
	}
	ds2482_wireResetSearch();
	while (ds2482_wireSearch(deviceAddress)) {

		if (validAddress(deviceAddress) && validFamily(deviceAddress)) {
			char b[3];
			d = &ds[ds18_devices];
			memset(d->address, 0, sizeof(d->address));
			for (i=0;i<8;i++) {
				d->deviceAddress[i] = deviceAddress[i];
				sprintf(b,"%02X", deviceAddress[i]);
				strcat(d->address, b);
			}
			d->is_connected = true;
			d->is_ds2482 = true;
			if (ds_readPowerSupply(deviceAddress)) d->parasite = true;
			d->bitResolution = max(d->bitResolution, ds_getResolution(d));
			if(maxBitResolution < d->bitResolution) maxBitResolution=d->bitResolution;

			//search configuration parameters
			for (int i=0; root && i<cJSON_GetArraySize(root); i++) {
				cJSON *ds = cJSON_GetArrayItem(root, i);
				p = cJSON_GetObjectItem(ds, "rom");
				if (p && p->valuestring && !strcmp(d->address, p->valuestring)) {
					p = cJSON_GetObjectItem(ds, "type");
					if (p) d->type = p->valueint;
					if (d->type > DS_UNKNOW) d->type = DS_UNKNOW;
					p = cJSON_GetObjectItem(ds, "corr");
					if (p) d->corr = p->valuedouble;
					p = cJSON_GetObjectItem(ds, "talert");
					if (p) d->talert = p->valuedouble;
					p = cJSON_GetObjectItem(ds, "descr");
					if (p && p->valuestring) d->description = strdup(p->valuestring);

				}
			}
			ESP_LOGI(TAG, "DS sensor found: %s", d->address);
			ds18_devices++;
		}
	}
	ESP_LOGI(TAG, "DS18B20 search complete, count: %d", ds18_devices);
	cJSON_Delete(root);
	return 0;
}

// returns the number of devices found on the bus
uint8_t ds_getDeviceCount(void)
{
    return ds18_devices;
}

// attempt to determine if the device at the given address is connected to the bus
bool ds_isConnected(DS18 *ds, uint8_t* scratchPad)
{
	bool b = ds_readScratchPad(ds, scratchPad);
	return b && (ow_crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);
}


bool ds_readScratchPad(DS18 *ds, uint8_t* scratchPad)
{
	// send the reset command and fail fast
	int b = ds2482_wireReset();
	if (b == 0) return false;

	ds2482_wireSelect(ds->deviceAddress);
	ds2482_wireWriteByte(READSCRATCH);

	// Read all registers in a simple loop
	// byte 0: temperature LSB
	// byte 1: temperature MSB
	// byte 2: high alarm temp
	// byte 3: low alarm temp
	// byte 4: DS18S20: store for crc
	//         DS18B20 & DS1822: configuration register
	// byte 5: internal use & crc
	// byte 6: DS18S20: COUNT_REMAIN
	//         DS18B20 & DS1822: store for crc
	// byte 7: DS18S20: COUNT_PER_C
	//         DS18B20 & DS1822: store for crc
	// byte 8: SCRATCHPAD_CRC
	for (uint8_t i = 0; i < 9; i++){
		scratchPad[i] = ds2482_wireReadByte();
	}

	b = ds2482_wireReset();
	return (b == 1);
}


void ds_writeScratchPad(DS18 *ds, const uint8_t* scratchPad)
{
	ds2482_wireReset();
	ds2482_wireSelect(ds->deviceAddress);
	ds2482_wireWriteByte(WRITESCRATCH);
	ds2482_wireWriteByte(scratchPad[HIGH_ALARM_TEMP]); // high alarm temp
	ds2482_wireWriteByte(scratchPad[LOW_ALARM_TEMP]); // low alarm temp

	// DS1820 and DS18S20 have no configuration register
	if (ds->deviceAddress[0] != DS18S20MODEL) {
		ds2482_wireWriteByte(scratchPad[CONFIGURATION]);
	}

	ds2482_wireReset();

	// save the newly written values to eeprom
	ds2482_wireSelect(ds->deviceAddress);
	//ds2482_wireWriteByte(COPYSCRATCH, parasite);
	ds2482_wireWriteByte(COPYSCRATCH);
	//ets_delay_us(20000);  // <--- added 20ms delay to allow 10ms long EEPROM write operation (as specified by datasheet)
	vTaskDelay(20/portTICK_PERIOD_MS);

	if (ds->parasite) // ets_delay_us(10000); // 10ms delay
		vTaskDelay(10/portTICK_PERIOD_MS);
	ds2482_wireReset();
}

bool ds_readPowerSupply(const uint8_t* deviceAddress)
{
	bool ret = false;
	ds2482_wireReset();

	ds2482_wireSelect((uint8_t*) deviceAddress);
	ds2482_wireWriteByte(READPOWERSUPPLY);
	if (ds2482_wireReadBit() == 0) ret = true;
	ds2482_wireReset();
	return ret;
}

// returns the current resolution of the device, 9-12
// returns 0 if device not found
uint8_t ds_getResolution(DS18 *ds)
{
	ScratchPad scratchPad;

	// DS1820 and DS18S20 have no resolution configuration register
	if (ds->deviceAddress[0] == DS18S20MODEL) return 12;


	if (ds_isConnected(ds, scratchPad)) {
		switch (scratchPad[CONFIGURATION]) {
		case TEMP_12_BIT:
			return 12;
		case TEMP_11_BIT:
			return 11;
		case TEMP_10_BIT:
			return 10;
		case TEMP_9_BIT:
			return 9;
		}
	}
	return 0;
}

// sets the value of the waitForConversion flag
// TRUE : function requestTemperature() etc returns when conversion is ready
// FALSE: function requestTemperature() etc returns immediately (USE WITH CARE!!)
//        (1) programmer has to check if the needed delay has passed
//        (2) but the application can do meaningful things in that time
void ds_setWaitForConversion(DS18 *ds, bool flag)
{
	ds->waitForConversion = flag;
}

// gets the value of the waitForConversion flag
bool ds_getWaitForConversion(DS18 *ds)
{
	return ds->waitForConversion;
}

// returns true if the bus requires parasite power
bool ds_isParasitePowerMode(DS18 *ds)
{
 	return ds->parasite;
}

bool ds_isConversionComplete(void)
{
   uint8_t b = ds2482_wireReadBit();
   return (b == 1);
}

// returns number of milliseconds to wait till conversion is complete (based on IC datasheet)
static int16_t millisToWaitForConversion(uint8_t bitResolution)
{

	switch (bitResolution){
	case 9:
		return 94;
	case 10:
		return 188;
	case 11:
		return 375;
	default:
		return 750;
	}
}

// Continue to check if the IC has responded with a temperature
void ds_blockTillConversionComplete(void)
{
	int delms = millisToWaitForConversion(maxBitResolution);
	//ets_delay_us(delms*1000);
	vTaskDelay((delms + portTICK_PERIOD_MS)/portTICK_PERIOD_MS) ;
}

// reads scratchpad and returns fixed-point temperature, scaling factor 2^-7
int16_t ds_calculateTemperature(DS18 *ds, uint8_t* scratchPad)
{

	int16_t fpTemperature =
		(((int16_t) scratchPad[TEMP_MSB]) << 11) |
		(((int16_t) scratchPad[TEMP_LSB]) << 3);

    /*
    DS1820 and DS18S20 have a 9-bit temperature register.
    Resolutions greater than 9-bit can be calculated using the data from
    the temperature, and COUNT REMAIN and COUNT PER ∞C registers in the
    scratchpad.  The resolution of the calculation depends on the model.
    While the COUNT PER ∞C register is hard-wired to 16 (10h) in a
    DS18S20, it changes with temperature in DS1820.
    After reading the scratchpad, the TEMP_READ value is obtained by
    truncating the 0.5∞C bit (bit 0) from the temperature data. The
    extended resolution temperature can then be calculated using the
    following equation:
                                    COUNT_PER_C - COUNT_REMAIN
    TEMPERATURE = TEMP_READ - 0.25 + --------------------------
                                            COUNT_PER_C
    Hagai Shatz simplified this to integer arithmetic for a 12 bits
    value for a DS18S20, and James Cameron added legacy DS1820 support.
    See - http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html
    */

	if (ds->deviceAddress[0] == DS18S20MODEL) {
		fpTemperature = ((fpTemperature & 0xfff0) << 3) - 16 +
		(
                ((scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN]) << 7) /
			scratchPad[COUNT_PER_C]
		);
	}

	return fpTemperature;
}


// returns temperature in 1/128 degrees C or DEVICE_DISCONNECTED_RAW if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_RAW is defined in
// DS18B20_DS2482.h. It is a large negative number outside the
// operating range of the device
int16_t ds_getTemp(DS18 *ds)
{
	ScratchPad scratchPad;
	if (ds_isConnected(ds, scratchPad)) {
		return ds_calculateTemperature(ds, scratchPad);
	}
	return DEVICE_DISCONNECTED_RAW;
}

// returns temperature in degrees C or DEVICE_DISCONNECTED_C if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_C is defined in
// DS18B20_DS2482.h. It is a large negative number outside the
// operating range of the device
float ds_getTempC(DS18 *ds)
{
	if (!ds) return -1;
	int16_t newT=ds_getTemp(ds);
	if  (newT==DEVICE_DISCONNECTED_RAW){ // ÂÒÎË Ó¯Ë·Í‡ ËÁÏÂÂÌËˇ
		if  (ds->errcount<DS_ERR_LIMIT) {			// Ë ÎËÏËÚ Ó¯Ë·ÓÍ ÌÂ ÔÂ‚˚¯ÂÌ
			ds->errcount++; 				// ÔË·‡‚ËÏ Ò˜ÂÚ˜ËÍ Ó¯Ë·ÓÍ
			return ds->Ce;					// Ë ‚ÂÌÂÏ ÔÂ‰˚‰Û˘ÂÂ ËÁÏÂÂÌÌÓÂ ÁÌ‡˜ÂÌËÂ
		}
		else															// ÂÒÎË ÎËÏËÚ Ó¯Ë·ÓÍ ÔÂ‚˚¯ÂÌ
			return (ds->Ce = rawToCelsius(newT));	// ‚ÓÁ‚‡˘‡ÂÏ “ "Í‡Í ÂÒÚ¸"
	} else  // Ó¯Ë·ÍË ÌÂÚ
		if (ds->errcount) ds->errcount=0; // Ò·ÓÒËÏ Ò˜ÂÚ˜ËÍ Ó¯Ë·ÓÍ
	ds->Ce = rawToCelsius(newT) + ds->corr;
	return ds->Ce;
}

// sends command for all devices on the bus to perform a temperature conversion
void ds_requestTemperatures(void)
{
	if (!ds2482_Address) return;
	ds2482_wireReset();
	ds2482_wireSkip();
	ds2482_wireWriteByte(STARTCONVO);
	ds_blockTillConversionComplete();
}

int16_t ds_getUserData(DS18 *ds)
{
	int16_t data = 0;
	ScratchPad scratchPad;
	if (ds_isConnected(ds, scratchPad)) {
		data = scratchPad[HIGH_ALARM_TEMP] << 8;
        	data += scratchPad[LOW_ALARM_TEMP];
	}
	return data;
}


// note if device is not connected it will fail writing the data.
void ds_setUserData(DS18 *ds, int16_t data)
{
	// return when stored value == new value
	if (ds_getUserData(ds) == data) return;

	ScratchPad scratchPad;
	if (ds_isConnected(ds, scratchPad)) {
		scratchPad[HIGH_ALARM_TEMP] = data >> 8;
		scratchPad[LOW_ALARM_TEMP] = data & 255;
		ds_writeScratchPad(ds, scratchPad);
	}
}



const char *getDsTypeStr(DsType t)
{
	switch(t) {
	case DS_CUB: return "Cube";
	case DS_TUBE20:	return "Tube 20%";
	case DS_TUBE: return "Tube";
	case DS_DEFL: return "Deflegmator";
	case DS_WATER_IN: return "Water IN";
	case DS_WATER_OUT: return "Water Out";
	case DS_ALARM: return "Alarm";
	default: return "Unknow";
	}
}

/*
static void sensor_detect(void)
{
    // install 1-wire bus
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TAG, "Device iterator created, start searching...");
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &s_ds18b20s[s_ds18b20_device_num]) == ESP_OK) {
                ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", s_ds18b20_device_num, next_onewire_device.address);
                s_ds18b20_device_num++;
            } else {
                ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", s_ds18b20_device_num);
}
*/


alarm_mode SavedAlarmMode;
void ds_task(void *arg)
{
	DS18 *d;
	for (int i=0; i<MAX_DS; i++) {
		d = &ds[i];
		d->id = i;
		d->deviceAddress[0] = 0;
		d->is_connected = false;
		d->parasite = false;
		d->bitResolution = 9;
		d->waitForConversion = true;
		d->type = DS_UNKNOW;
		d->talert = 0;
		d->corr = 0;
		d->Ce = 0;
		d->errcount=0;
		if(d->description) { free(d->description); d->description = NULL; }
	}
	maxBitResolution = 9;

	ds2482_init();	// Detect and init DS2482 chip
	ds_init(0, NULL); // Detect any DS18B20 connected to Ds2482
	if (PIN_DS18B20>=0) ow_init(PIN_DS18B20);

	const esp_console_cmd_t cmd = {
        	.command = "scands",
		.help = "Rescan onewire bus for temperature sensor",
		.hint = NULL,
		.func = &ds_init,
	};
	ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );


	while(1) {
		if (!ds18_devices){
			vTaskDelay(1000/portTICK_PERIOD_MS);
			continue;
		}

		ds_requestTemperatures();
		for (int i=0; i<MAX_DS; i++) {
			DS18 *d = &ds[i];
			if (!d->is_connected) continue;
			if (d->is_ds2482) {
				ds_getTempC(d);
			} else {
				float oldC = d->Ce;
				ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(d->handle));
				ESP_ERROR_CHECK(ds18b20_get_temperature(d->handle, &d->Ce));
				if ((oldC < 84.5 || oldC > 85.5) && d->Ce == 85) {
					// Error condition
					ESP_LOGI(TAG, "Error read from DS18B20[%d]!", d->id);
					continue;
				}
			}

			if (DS_ALARM == d->type && d->Ce > d->talert) {
				// Alarm mode
				AlarmMode = ALARM_TEMP;
				if (SavedAlarmMode != AlarmMode) {
					sendSMS("Temperature alarm! power switched off!");

				}
			} else {
				AlarmMode &= ~(ALARM_TEMP);
			}
			SavedAlarmMode = AlarmMode;
		}
		vTaskDelay(500/portTICK_PERIOD_MS);
	}
}

// –ü–æ–ª—É—á–∏—Ç—å —Ç–µ–º–ø–µ—Ä–∞—Ç—É—Ä—É –≤ –∫—É–±–µ

double getCubeTemp(void) {
	if (emulate_devices) return testCubeTemp; // –≠–º—É–ª—è—Ü–∏—è

	for (int i=0; i<MAX_DS; i++) {
		DS18 *d = &ds[i];
		if (!d->is_connected) continue;
		if (DS_CUB == d->type) return d->Ce;
	}
	return -1;
}

// –ü–æ–ª—É—á–∏—Ç—å —Ç–µ–º–ø–µ—Ä–∞—Ç—É—Ä—É –≤ –Ω–∏–∂–Ω–µ–π —á–∞—Å—Ç–∏ –∫–æ–ª–æ–Ω–Ω—ã
double getTube20Temp(void)
{
	for (int i=0; i<MAX_DS; i++) {
		DS18 *d = &ds[i];
		if (!d->is_connected) continue;
		if (DS_TUBE20 == d->type) return d->Ce;
	}
	// –ï—Å–ª–∏ –¥–∞—Ç—á–∏–∫ –Ω–µ –Ω–∞–π–¥–µ–Ω - –≤–æ–∑–≤—Ä–∞—â–∞–µ–º –∫—É–±–æ–≤—É—é —Ç–µ–º–ø–µ—Ä–∞—Ç—É—Ä—É
	return getCubeTemp();
}
