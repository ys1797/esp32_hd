/*!
 * @file Adafruit_ILI9341.cpp
 *
 * @mainpage Adafruit ILI9341 TFT Displays
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's ILI9341 driver for the
 * Arduino platform.
 *
 * This library works with the Adafruit 2.8" Touch Shield V2 (SPI)
 *    http://www.adafruit.com/products/1651
 *
 * Adafruit 2.4" TFT LCD with Touchscreen Breakout w/MicroSD Socket - ILI9341
 *    https://www.adafruit.com/product/2478
 *
 * 2.8" TFT LCD with Touchscreen Breakout Board w/MicroSD Socket - ILI9341
 *    https://www.adafruit.com/product/1770
 *
 * 2.2" 18-bit color TFT LCD display with microSD card breakout - ILI9340
 *    https://www.adafruit.com/product/1770
 *
 * TFT FeatherWing - 2.4" 320x240 Touchscreen For All Feathers
 *    https://www.adafruit.com/product/3315
 *
 * These displays use SPI to communicate, 4 or 5 pins are required
 * to interface (RST is optional).
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
 * Adafruit_GFX</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Limor "ladyada" Fried for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "../hd_spi_i2c.h"

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(uint8_t *)(addr))
#endif

#define SWAPBYTES(i) ((i>>8) | (i<<8))

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

#define __min(a,b) ((a > b) ? (b):(a))


/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ILI9341 driver with hardware SPI using the
            default SPI peripheral.
    @param  cs   Chip select pin # (OK to pass -1 if CS tied to GND).
    @param  dc   Data/Command pin # (required).
    @param  rst  Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
Adafruit_ILI9341::Adafruit_ILI9341()
    : Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {}



// clang-format off
static uint8_t initcmd[] = {
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  ILI9341_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
  ILI9341_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_VMCTR1  , 2, 0x3e, 0x28,       // VCM control
  ILI9341_VMCTR2  , 1, 0x86,             // VCM control2
  ILI9341_MADCTL  , 1, 0x48,             // Memory Access Control
  ILI9341_VSCRSADD, 1, 0x00,             // Vertical scroll zero
  ILI9341_PIXFMT  , 1, 0x55,
  ILI9341_FRMCTR1 , 2, 0x00, 0x18,
  ILI9341_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control
  0xF2, 1, 0x00,                         // 3Gamma Function Disable
  ILI9341_GAMMASET , 1, 0x01,             // Gamma curve selected
  ILI9341_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLPOUT  , 0x80,                // Exit Sleep
  ILI9341_DISPON  , 0x80,                // Display on
  0x00                                   // End of list
};
// clang-format on

/**************************************************************************/
/*!
    @brief   Initialize ILI9341 chip
    Connects to the ILI9341 over SPI and sends initialization procedure commands
    @param    freq  Desired SPI clock frequency
*/
/**************************************************************************/
void Adafruit_ILI9341::begin() {


	spi_cmd(ILI9341_SWRESET); // Engage software reset
	vTaskDelay(150/portTICK_PERIOD_MS);

	uint8_t cmd, x, numArgs;
	uint8_t *addr = initcmd;
	while ((cmd = pgm_read_byte(addr++)) > 0) {
		x = pgm_read_byte(addr++);
		numArgs = x & 0x7F;
		sendCommand(cmd, addr, numArgs);
		addr += numArgs;
		if (x & 0x80) 
			vTaskDelay(150/portTICK_PERIOD_MS);
	}
	_width = ILI9341_TFTWIDTH;
	_height = ILI9341_TFTHEIGHT;
}

/**************************************************************************/
/*!
    @brief   Set origin of (0,0) and orientation of TFT display
    @param   m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void Adafruit_ILI9341::setRotation(uint8_t m) {
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
  case 0:
    m = (MADCTL_MX | MADCTL_BGR);
    _width = ILI9341_TFTWIDTH;
    _height = ILI9341_TFTHEIGHT;
    break;
  case 1:
    m = (MADCTL_MV | MADCTL_BGR);
    _width = ILI9341_TFTHEIGHT;
    _height = ILI9341_TFTWIDTH;
    break;
  case 2:
    m = (MADCTL_MY | MADCTL_BGR);
    _width = ILI9341_TFTWIDTH;
    _height = ILI9341_TFTHEIGHT;
    break;
  case 3:
    m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
    _width = ILI9341_TFTHEIGHT;
    _height = ILI9341_TFTWIDTH;
    break;
  }

  sendCommand(ILI9341_MADCTL, &m, 1);
}

/**************************************************************************/
/*!
    @brief   Enable/Disable display color inversion
    @param   invert True to invert, False to have normal color
*/
/**************************************************************************/
void Adafruit_ILI9341::invertDisplay(bool invert) {
  spi_cmd(invert ? ILI9341_INVON : ILI9341_INVOFF);
}

/**************************************************************************/
/*!
    @brief   Scroll display memory
    @param   y How many pixels to scroll display by
*/
/**************************************************************************/
void Adafruit_ILI9341::scrollTo(uint16_t y) {
  uint8_t data[2];
  data[0] = y >> 8;
  data[1] = y & 0xff;
  sendCommand(ILI9341_VSCRSADD, (uint8_t *)data, 2);
}

/**************************************************************************/
/*!
    @brief   Set the height of the Top and Bottom Scroll Margins
    @param   top The height of the Top scroll margin
    @param   bottom The height of the Bottom scroll margin
 */
/**************************************************************************/
void Adafruit_ILI9341::setScrollMargins(uint16_t top, uint16_t bottom) {
  // TFA+VSA+BFA must equal 320
  if (top + bottom <= ILI9341_TFTHEIGHT) {
    uint16_t middle = ILI9341_TFTHEIGHT - top + bottom;
    uint8_t data[6];
    data[0] = top >> 8;
    data[1] = top & 0xff;
    data[2] = middle >> 8;
    data[3] = middle & 0xff;
    data[4] = bottom >> 8;
    data[5] = bottom & 0xff;
    sendCommand(ILI9341_VSCRDEF, (uint8_t *)data, 6);
  }
}

/**************************************************************************/
/*!
    @brief   Set the "address window" - the rectangle we will write to RAM with
   the next chunk of      SPI data writes. The ILI9341 will automatically wrap
   the data as each row is filled
    @param   x1  TFT memory 'x' origin
    @param   y1  TFT memory 'y' origin
    @param   w   Width of rectangle
    @param   h   Height of rectangle
*/
/**************************************************************************/
void Adafruit_ILI9341::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
                                     uint16_t y1) {
//  uint16_t x1 = (x0 + w - 1), y1 = (y0 + h - 1);
  spi_cmd(ILI9341_CASET); // Column address set
  spi_write16(x0);
  spi_write16(x1);
  spi_cmd(ILI9341_PASET); // Row address set
  spi_write16(y0);
  spi_write16(y1);
  spi_cmd(ILI9341_RAMWR); // Write to RAM
}

/**************************************************************************/
/*!
    @brief  Read 8 bits of data from ILI9341 configuration memory. NOT from RAM!
            This is highly undocumented/supported, it's really a hack but kinda
   works?
    @param    commandByte  The command register to read data from
    @param    index  The byte index into the command to read from
    @return   Unsigned 8-bit data read from ILI9341 register
 */
/**************************************************************************/
uint8_t Adafruit_ILI9341::readcommand8(uint8_t commandByte, uint8_t index) {
	uint8_t data = 0x10 + index;
	sendCommand(0xD9, &data, 1); // Set Index Register
	return spi_read8(commandByte);
}

void Adafruit_ILI9341::sendCommand(uint8_t commandByte, uint8_t *dataBytes,
                                  uint8_t numDataBytes) {

	spi_cmd(commandByte); // Send the command byte
	spi_data(dataBytes, numDataBytes);
}

void Adafruit_ILI9341::drawPixel(int16_t x, int16_t y, uint16_t color)
{
	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
	setAddrWindow(x, y, x+1, y+1);
	spi_write16(SWAPBYTES(color));
}

	
void Adafruit_ILI9341::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{	// Rudimentary clipping
	if((x >= _width) || (y >= _height)) return;

	if((y+h-1) >= _height)
		h = _height-y;

	setAddrWindow(x, y, x, y+h-1);
	transmitData(SWAPBYTES(color), h);
}

void Adafruit_ILI9341::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) 
{	// Rudimentary clipping
	if((x >= _width) || (y >= _height)) return;
	if((x+w-1) >= _width)  w = _width-x;
	setAddrWindow(x, y, x+w-1, y+1);
	transmitData(SWAPBYTES(color), w);
}

void Adafruit_ILI9341::fillScreen(uint16_t color) 
{
	fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Adafruit_ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 
{	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= _width) || (y >= _height)) return;
	if((x + w - 1) >= _width)  w = _width  - x;
	if((y + h - 1) >= _height) h = _height - y;

	setAddrWindow(x, y, x+w-1, y+h-1);
	transmitData(SWAPBYTES(color), h*w);
}



void Adafruit_ILI9341::transmitData(uint16_t data, int32_t repeats)
{
    uint32_t i;
    uint32_t word = (data << 16) | data;
    uint32_t word_tmp[64];

        for(i = 0; i<64; i++) {
            word_tmp[i] = word;
        }


    while(repeats > 0) {
        uint16_t bytes_to_transfer = __min(repeats * 2, 64);
	spi_data((uint8_t*)word_tmp, bytes_to_transfer);
        repeats -= bytes_to_transfer / 2;
    }
}