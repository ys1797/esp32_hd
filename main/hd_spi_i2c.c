/*
spi/i2c interface
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
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "hd_spi_i2c.h"

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16


//#define CONFIG_LCD_OVERCLOCK 1

#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    200000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0   /* I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /* I2C master do not need buffer */
#define I2C_ACK_CHECK_EN   	0x1     /* I2C master will check ack from slave*/
#define I2C_ACK_CHECK_DIS  	0x0     /* I2C master will not check ack from slave */
#define I2C_ACK_VAL		0x0	/* I2C ack value */
#define I2C_NACK_VAL		0x1	/* I2C nack value */



static i2c_port_t i2c_num;
static xSemaphoreHandle i2c_mux;
char I2C_detect[128];	/* detectecd i2c devices list */
spi_device_handle_t Spi;

/* 
 * Отправка команды в LCD. Используется spi_device_transmit,
 * которая ожидает окончания передачи команды.
 */
void spi_cmd(const uint8_t cmd) 
{
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length=8;			// Команда состоит из 8 бит
	t.tx_buffer=&cmd;		// Данные - это сама команда
	t.user=(void*)0;		// линия D/C должна быть установлена в 0
	xSemaphoreTake(i2c_mux, portMAX_DELAY);
	ret=spi_device_transmit(Spi, &t);  // Передача команды
	xSemaphoreGive(i2c_mux);
	assert(ret==ESP_OK);
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void spi_data(const uint8_t *data, int len) 
{
	esp_err_t ret;
	spi_transaction_t t;
	if (len==0) return;
	memset(&t, 0, sizeof(t));
	t.length=len*8;                 // Размер данных в битах.
	t.tx_buffer=data;               // Данные
	t.user=(void*)1;                // линия D/C должна быть установлена в 1
	xSemaphoreTake(i2c_mux, portMAX_DELAY);
	ret=spi_device_transmit(Spi, &t);  //Передача данных
	xSemaphoreGive(i2c_mux);
	assert(ret==ESP_OK);
}

void spi_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	esp_err_t ret;
	int x;
	static spi_transaction_t trans[5];

	for (x=0; x<5; x++) {
		memset(&trans[x], 0, sizeof(spi_transaction_t));
		if ((x&1)==0) {
			// Even transfers are commands
			trans[x].length=8;
			trans[x].user=(void*)0;
		} else {
			//Odd transfers are data
			trans[x].length=8*4;
			trans[x].user=(void*)1;
		}
		trans[x].flags=SPI_TRANS_USE_TXDATA;
	}

	trans[0].tx_data[0]=0x2A;	//Column Address Set
	trans[1].tx_data[0]=x0>>8;	//Start Col High
	trans[1].tx_data[1]=x0 & 0xFF;	//Start Col Low
	trans[1].tx_data[2]=x1>>8;	//End Col High
	trans[1].tx_data[3]=x1 & 0xff;	//End Col Low
	trans[2].tx_data[0]=0x2B;	//Page address set
	trans[3].tx_data[0]=y0>>8;	//Start page high
	trans[3].tx_data[1]=y0 & 0xff;	//start page low
	trans[3].tx_data[2]=y1>>8;	//end page high
	trans[3].tx_data[3]=y1&0xff;	//end page low
	trans[4].tx_data[0]=0x2C;	//memory write

	//Queue all transactions.
	for (x=0; x<5; x++) {
		ret=spi_device_transmit(Spi, &trans[x]);
		assert(ret==ESP_OK);
	}
}


void spi_send_lines(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t *data, uint32_t datasize)
{
	esp_err_t ret;
	int x;
	static spi_transaction_t trans[6];

	for (x=0; x<6; x++) {
		memset(&trans[x], 0, sizeof(spi_transaction_t));
		if ((x&1)==0) {
			// Even transfers are commands
			trans[x].length=8;
			trans[x].user=(void*)0;
		} else {
			//Odd transfers are data
			trans[x].length=8*4;
			trans[x].user=(void*)1;
		}
		trans[x].flags=SPI_TRANS_USE_TXDATA;
	}

	trans[0].tx_data[0]=0x2A;	//Column Address Set
	trans[1].tx_data[0]=x0>>8;	//Start Col High
	trans[1].tx_data[1]=x0 & 0xFF;	//Start Col Low
	trans[1].tx_data[2]=x1>>8;	//End Col High
	trans[1].tx_data[3]=x1 & 0xff;	//End Col Low
	trans[2].tx_data[0]=0x2B;	//Page address set
	trans[3].tx_data[0]=y0>>8;	//Start page high
	trans[3].tx_data[1]=y0 & 0xff;	//start page low
	trans[3].tx_data[2]=y1>>8;	//end page high
	trans[3].tx_data[3]=y1&0xff;	//end page low
	trans[4].tx_data[0]=0x2C;	//memory write
	trans[5].tx_buffer=data;	//finally send the line data
	trans[5].length=datasize*8;	//Data length, in bits
	trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag

	//Queue all transactions.
	for (x=0; x<5; x++) {
		ret=spi_device_transmit(Spi, &trans[x]);
		assert(ret==ESP_OK);
	}
}

void spi_write16(uint16_t w)
{
	esp_err_t ret;
	spi_transaction_t t;
	uint8_t d[2];
	
	d[0] = w >> 8;
	d[1] = w & 0xFF;

	memset(&t, 0, sizeof(t));
	t.length=2*8;                 // Размер данных в битах.
	t.tx_buffer=d;               // Данные
	t.user=(void*)1;                // линия D/C должна быть установлена в 1
	xSemaphoreTake(i2c_mux, portMAX_DELAY);
	ret=spi_device_transmit(Spi, &t);  //Передача данных
	xSemaphoreGive(i2c_mux);
	assert(ret==ESP_OK);

}

void spi_write32(uint32_t l)
{
	esp_err_t ret;
	spi_transaction_t t;
	uint8_t d[4];

	d[0] = (l >> 24) & 0xFF;
	d[1] = (l >> 16) & 0xFF;
	d[2] = (l >> 8) & 0xFF;
	d[3] = l & 0xFF;

	memset(&t, 0, sizeof(t));
	t.length=4*8;                 // Размер данных в битах.
	t.tx_buffer=d;               // Данные
	t.user=(void*)1;                // линия D/C должна быть установлена в 1
	xSemaphoreTake(i2c_mux, portMAX_DELAY);
	ret=spi_device_transmit(Spi, &t);  //Передача данных
	xSemaphoreGive(i2c_mux);
	assert(ret==ESP_OK);

}


/*
 * Функция вызывается (из прерывания!) перед началом передачи по шине.
 * Она должна установить уровень на линии D/C в значение указанное в поле "user".
 */
void spi_pre_transfer_callback(spi_transaction_t *t) 
{
	int dc=(int)t->user;
	gpio_set_level(SPI_PIN_DC, dc);
}

uint8_t spi_read8(uint8_t commandByte)
{
	spi_transaction_t t;

	spi_cmd(commandByte);
	memset(&t, 0, sizeof(t));
	t.length=8;
	t.flags = SPI_TRANS_USE_RXDATA;
	t.user = (void*)1;
	xSemaphoreTake(i2c_mux, portMAX_DELAY);
	esp_err_t ret = spi_device_polling_transmit(Spi, &t);
	xSemaphoreGive(i2c_mux);
	assert( ret == ESP_OK );
	return *(uint32_t*)t.rx_data;
}

/*
 * Настройка spi интерфейса
 */
void spi_setup(int t_sz)
{
	esp_err_t ret;

	// Initialize non-SPI GPIOs
	gpio_set_direction(SPI_PIN_DC, GPIO_MODE_OUTPUT);
	gpio_set_direction(SPI_PIN_CS, GPIO_MODE_OUTPUT);
	gpio_set_direction(SPI_PIN_RST, GPIO_MODE_OUTPUT);

	// Отправка команды Reset дисплею
	gpio_set_level(SPI_PIN_RST, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(SPI_PIN_RST, 1);
	vTaskDelay(100 / portTICK_RATE_MS);

	if (!i2c_mux) i2c_mux = xSemaphoreCreateMutex();

	spi_bus_config_t buscfg={
		.miso_io_num=SPI_PIN_MISO,
		.mosi_io_num=SPI_PIN_MOSI,
		.sclk_io_num=SPI_PIN_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz = t_sz,
	};
	spi_device_interface_config_t devcfg={
//#ifdef CONFIG_LCD_OVERCLOCK
	      	.clock_speed_hz=26*1000*1000,	// Частота 26 MHz
//#else
//        	.clock_speed_hz=10*1000*1000,	// Частота 10 MHz
//#endif
		.mode=0,                        //SPI mode 0
		.spics_io_num=SPI_PIN_CS,	//CS pin
		.queue_size=7,			//We want to be able to queue 7 transactions at a time
		.pre_cb = spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
	};

	// инициализация SPI шины
	ret = spi_bus_initialize(VSPI_HOST, &buscfg, 2);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "SPI initialization failed!\n");
		return;
	}
	//Attach the LCD to the SPI bus
	ret = spi_bus_add_device(VSPI_HOST, &devcfg, &Spi);
	if (ret != ESP_OK) {
		ESP_LOGW(TAG, "SPI device atach failed!\n");
		spi_bus_free(VSPI_HOST);
		return;
	}
	ESP_LOGI(TAG, "SPI initialization complete.");
}



esp_err_t I2CWrite(uint8_t i2c_address, uint8_t* data_wr, size_t size) {
	xSemaphoreTake(i2c_mux, portMAX_DELAY);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2c_address << 1 ), I2C_ACK_CHECK_EN);
	i2c_master_write(cmd, data_wr, size, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	xSemaphoreGive(i2c_mux);
	return ret;
}

esp_err_t I2CRead(uint8_t i2c_add, uint8_t* data_rd, size_t size) {
	if (size == 0) {
		return ESP_OK;
	}
	xSemaphoreTake(i2c_mux, portMAX_DELAY);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( i2c_add << 1 ) | I2C_MASTER_READ, I2C_ACK_CHECK_EN);
	if (size > 1) {
		i2c_master_read(cmd, data_rd, size-1, I2C_ACK_VAL);
	}
	i2c_master_read_byte(cmd, data_rd + size-1, I2C_NACK_VAL);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	xSemaphoreGive(i2c_mux);
	return ret;
}

esp_err_t I2CRead2(uint8_t i2c_add) 
{
	esp_err_t err;
	uint8_t data[4];
	data[0] = 0xe1;
	data[1] = 0xf0;

	xSemaphoreTake(i2c_mux, portMAX_DELAY);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2c_add << 1 ), I2C_ACK_CHECK_EN);
	i2c_master_write(cmd, data, 2, I2C_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);


	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( i2c_add << 1)|I2C_MASTER_READ, I2C_ACK_CHECK_EN);
	i2c_master_read_byte(cmd, data, I2C_NACK_VAL);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	xSemaphoreGive(i2c_mux);
	return err;
}


void task_i2cscanner(void)
{
	printf(">> i2cScanner");

	int i;
	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");

	for (i=3; i< 0x78; i++) {
                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
		i2c_master_stop(cmd);
		espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
		if (i%16 == 0) {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) {
			I2C_detect[i] = 1;	// Detected
			printf(" %.2x", i);
		} else {
			I2C_detect[i] = 0;	// Not detected
			printf(" --");
		}
		//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
	}
	printf("\n");
}



uint8_t I2C_Init(i2c_port_t _num, uint8_t _sda, uint8_t _scl)
{
	i2c_num = _num;
	if (!i2c_mux) i2c_mux = xSemaphoreCreateMutex();
	ESP_ERROR_CHECK(!i2c_mux);
printf("Install I2C _num: %d sda: %d scl: %d\n", _num, _sda, _scl);
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = _sda,
		.scl_io_num = _scl,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
	};
	i2c_param_config(i2c_num, &conf);
	i2c_driver_install(i2c_num, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	printf("I2C installed\n");
	return 0;
}

