/*
 *
 *
 *
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
#include "hd_spi_i2c.h"

#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0   /* I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /* I2C master do not need buffer */
#define I2C_ACK_CHECK_EN   	0x1     /* I2C master will check ack from slave*/
#define I2C_ACK_CHECK_DIS  	0x0     /* I2C master will not check ack from slave */
#define I2C_ACK_VAL		0x0	/* I2C ack value */
#define I2C_NACK_VAL		0x1	/* I2C nack value */



static i2c_port_t i2c_num;
static uint8_t sda_pin;
static uint8_t scl_pin;
static xSemaphoreHandle i2c_mux;
char I2C_detect[128];	/* detectecd i2c devices list */



spi_device_handle_t Spi;

/* 
 * Отправка команды в LCD. Используется spi_device_transmit,
 * которая ожидает окончания передачи команды.
 */
void lcd_cmd(const uint8_t cmd) 
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
void lcd_data(const uint8_t *data, int len) 
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

/*
 * Функция вызывается (из прерывания!) перед началом передачи по шине.
 * Она должна установить уровень на линии D/C в значение указанное в поле "user".
 */
void lcd_spi_pre_transfer_callback(spi_transaction_t *t) 
{
	int dc=(int)t->user;
	gpio_set_level(PIN_NUM_DC, dc);
}

/*
 * Настройка spi интерфейса
 */
void spi_setup(void)
{
	esp_err_t ret;
	spi_bus_config_t buscfg={
		.miso_io_num=-1,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1
	};
	spi_device_interface_config_t devcfg={
        	.clock_speed_hz=10*1000*1000,	// Частота 10 MHz
		.mode=0,                        //SPI mode 0
		.spics_io_num=-1,		//CS pin
		.queue_size=1,			//We want to be able to queue 7 transactions at a time
		.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
	};
	// Initialize non-SPI GPIOs
	gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
	gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);


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


	// Отправка команды Reset дисплею
	gpio_set_level(PIN_NUM_RST, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(PIN_NUM_RST, 1);
	vTaskDelay(100 / portTICK_RATE_MS);

	if (!i2c_mux) i2c_mux = xSemaphoreCreateMutex();
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
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
}



uint8_t I2C_Init(i2c_port_t _num, uint8_t _sda, uint8_t _scl) {

	i2c_config_t conf;
	i2c_num = _num;
	sda_pin = _sda;
	scl_pin = _scl;

	if (!i2c_mux) i2c_mux = xSemaphoreCreateMutex();
	ESP_ERROR_CHECK(!i2c_mux);
		
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = sda_pin;
	conf.scl_io_num = scl_pin;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	i2c_param_config(i2c_num, &conf);

	i2c_driver_install(i2c_num, conf.mode,
		I2C_MASTER_RX_BUF_DISABLE,
		I2C_MASTER_TX_BUF_DISABLE, 0);
	printf("I2C installed\n");
	return 0;
}

