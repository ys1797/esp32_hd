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


#ifndef DS_ONEWIRE_H_
#define DS_ONEWIRE_H_

#ifdef __cplusplus
extern "C" {
#endif


// Model IDs 
#define DS18S20MODEL 0x10  // also DS1820 
#define DS18B20MODEL 0x28 
#define DS1822MODEL  0x22 
#define DS1825MODEL  0x3B 
#define DS28EA00MODEL 0x42 
 
// OneWire commands 
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad 
#define COPYSCRATCH     0x48  // Copy EEPROM 
#define READSCRATCH     0xBE  // Read EEPROM 
#define WRITESCRATCH    0x4E  // Write to EEPROM 
#define RECALLSCRATCH   0xB8  // Reload from last known 
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power 
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition 
 
// Scratchpad locations 
#define TEMP_LSB        0 
#define TEMP_MSB        1 
#define HIGH_ALARM_TEMP 2 
#define LOW_ALARM_TEMP  3 
#define CONFIGURATION   4 
#define INTERNAL_BYTE   5 
#define COUNT_REMAIN    6 
#define COUNT_PER_C     7 
#define SCRATCHPAD_CRC  8 

// Device resolution 
#define TEMP_9_BIT  0x1F //  9 bit 
#define TEMP_10_BIT 0x3F // 10 bit 
#define TEMP_11_BIT 0x5F // 11 bit 
#define TEMP_12_BIT 0x7F // 12 bit 

 
// Коды ошибок
#define DEVICE_DISCONNECTED_C -127 
#define DEVICE_DISCONNECTED_F -196.6 
#define DEVICE_DISCONNECTED_RAW -7040 

typedef uint8_t DeviceAddress[8];
typedef uint8_t ScratchPad[9];

typedef enum DsType {
	DS_CUB=0,		// Датчик температуры куба
	DS_TUBE20=1,		// Датчик в нижней части колонны
	DS_TUBE=2,		// Датчик в верней части колонны
	DS_DEFL=3,		// Датчик в дефлегматоре
	DS_WATER_IN=4,		// Датчик воды охлаждения - вход
	DS_WATER_OUT=5,		// Датчик воды охлаждения - выход
	DS_ALARM=6,		// Датчик для аварийной сигнализации
	DS_UNKNOW=7		// Неизвестный информационный датчик
} DsType;


typedef struct {
	uint8_t	id;
	bool is_connected;	// Флаг наличия датчика
	bool parasite;
	DeviceAddress deviceAddress;	// ROM адрес датчика
	char	adressStr[20];	// Строка для ROM адреса датчика в читабельном виде
	uint8_t bitResolution;	// Разрешение датчика, используется для вычисления задержки
	bool waitForConversion;	// Флаг ожидания окончания преобразования температуры
	DsType	type;		// Назначение датчика
	double corr;		// Поправка к темперетуре	
	double Ce;		// Последнее считанное значение температуры
	double talert;		// Максимальная температура в режиме аварийного датчика
	char  *description;
} DS18;

#define MAX_DS	8

// Gpio interface
extern unsigned char ow_rom_codes[MAX_DS][9];
extern uint8_t ow_devices;	// count of devices on the bus
extern uint8_t emulate_devices;	// Режим эмуляции температуры
extern double testCubeTemp;	// Тестовое значение кубовой температуры


// i2c->w1 gate interface
extern uint8_t ds18_devices;	// Количество датчиков ds18b20
extern DS18 ds[MAX_DS];


void ow_init(int GPIO);
uint8_t ow_reset(void);
void ow_write_bit(uint8_t v);
uint8_t ow_read_bit(void);
void ow_write(uint8_t v, uint8_t power);
void ow_write_bytes(const uint8_t *buf, uint16_t count, bool power);
uint8_t ow_read(void);
void ow_read_bytes(uint8_t *buf, uint16_t count);
void ow_select( int8_t rom[8]);
void ow_skip(void);
void ow_depower(void);
void ow_reset_search(void);
uint8_t ow_search(uint8_t *newAddr);
uint8_t ow_crc8(const uint8_t *addr, uint8_t len);
bool ow_check_crc16(uint8_t* input, uint16_t len, uint8_t* inverted_crc);
uint16_t ow_crc16(uint8_t* input, uint16_t len);

/* DS2482 */
void ds2482_init(void);
uint8_t ds2482_readStatus(bool setPtr);
bool ds2482_wireReset(void);
bool ds2482_configure(uint8_t config);
bool ds2482_selectChannel(uint8_t channel);
void ds2482_wireWriteByte(uint8_t b);
uint8_t ds2482_wireReadByte();
void ds2482_wireWriteBit(uint8_t bit);
void ds2482_wireSkip();
void ds2482_wireSelect(uint8_t rom[8]);
uint8_t ds2482_wireReadBit();
void ds2482_wireResetSearch();
uint8_t ds2482_wireSearch(uint8_t *newAddr);
uint8_t ds2482_devicesCount(bool printAddress);

int ds_init(int argc, char** argv);
uint8_t ds_getDeviceCount(void);
bool ds_isConnected(DS18 *ds, uint8_t* scratchPad);
bool ds_readScratchPad(DS18 *ds, uint8_t* scratchPad);
void ds_writeScratchPad(DS18 *ds, const uint8_t* scratchPad);
bool ds_readPowerSupply(const uint8_t* deviceAddress);
uint8_t ds_getResolution(DS18 *ds);
void ds_setWaitForConversion(DS18 *ds, bool flag);
bool ds_getWaitForConversion(DS18 *ds);
bool ds_isParasitePowerMode(DS18 *ds);
bool ds_isConversionComplete(void);
void ds_blockTillConversionComplete(void);
int16_t ds_calculateTemperature(DS18 *ds, uint8_t* scratchPad);
int16_t ds_getTemp(DS18 *ds);
float ds_getTempC(DS18 *ds);
void ds_requestTemperatures(void);
int16_t ds_getUserData(DS18 *ds);
void ds_setUserData(DS18 *ds, int16_t data);
const char *getDsTypeStr(DsType t);
void ds_task(void *arg);
double getCubeTemp(void);	// Получить температуру в кубе
double getTube20Temp(void);	// Получить температуру в нижней части колонны

#ifdef __cplusplus
}
#endif

#endif // DS_ONEWIRE_H_
