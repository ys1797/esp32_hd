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

#ifndef HD_SPI_I2C_h
#define HD_SPI_I2C_h

#ifdef __cplusplus
extern "C" {
#endif


#include "driver/spi_master.h"
#include "driver/i2c.h"

typedef enum {
	LCD_TYPE_ILI = 1,
	LCD_TYPE_ST,
	LCD_TYPE_MAX,
} type_lcd_t;


#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
extern char I2C_detect[128];	/* detectecd i2c devices list */
extern type_lcd_t lcd_type;	/* detected lcd type on SPI bus */

uint8_t I2C_Init(i2c_port_t i2c_num, uint8_t _sda, uint8_t _scl);
esp_err_t I2CWrite(uint8_t i2c_address, uint8_t* data_wr, size_t size);
esp_err_t I2CRead(uint8_t i2c_add, uint8_t* data_rd, size_t size);
esp_err_t I2CRead2(uint8_t i2c_add);
void task_i2cscanner(void);

void spi_setup(void);
void spi_cmd(const uint8_t cmd);
void spi_data(const uint8_t *data, int len) ;
void spi_write16(const uint16_t w);
void spi_write32(const uint32_t l);
uint8_t spi_read8(uint8_t commandByte);

#ifdef __cplusplus
}
#endif

#endif // HD_SPI_I2C_h