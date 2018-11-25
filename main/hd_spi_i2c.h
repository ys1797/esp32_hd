

#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
extern char I2C_detect[128];	/* detectecd i2c devices list */

uint8_t I2C_Init(i2c_port_t i2c_num, uint8_t _sda, uint8_t _scl);
esp_err_t I2CWrite(uint8_t i2c_address, uint8_t* data_wr, size_t size);
esp_err_t I2CRead(uint8_t i2c_add, uint8_t* data_rd, size_t size);
esp_err_t I2CRead2(uint8_t i2c_add);
void task_i2cscanner(void);

void spi_setup(void);
void lcd_cmd(const uint8_t cmd);
void lcd_data(const uint8_t *data, int len) ;