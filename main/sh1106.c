
#include "sh1106.h"
#include "math.h"
#include "hd_spi_i2c.h"

#define abs(x)   ((x) > 0 ? (x) : -(x))
#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

#ifndef max
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#endif

/* Display data and i2c buffer */
static uint8_t Display_Buffer[DISPLAY_BUFFER_SIZE];
static uint8_t I2CBuffer[512];
static const char ANIMATION_activeSymbol[] = {
  0x00, 0x18, 0x3c, 0x7e, 0x7e, 0x3c, 0x18, 0x00
};

static const char ANIMATION_inactiveSymbol[] = {
  0x00, 0x0, 0x0, 0x18, 0x18, 0x0, 0x0, 0x00
};

/* Private Display structure */
typedef struct {
	uint8_t	i2c_address;

	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
	OLEDDISPLAY_TEXT_ALIGNMENT textAlignment;
	OLEDDISPLAY_COLOR color;
	const char	*fontData;

	// State values for logBuffer
	uint16_t   logBufferSize;
	uint16_t   logBufferFilled;
	uint16_t   logBufferLine;
	uint16_t   logBufferMaxLines;
	char      *logBuffer;

	// UI
	// Symbols for the Indicator
	IndicatorPosition   indicatorPosition;
	IndicatorDirection  indicatorDirection;

	const char*         activeSymbol;
	const char*         inactiveSymbol;

	bool                shouldDrawIndicators;

	// Values for the Frames
	AnimationDirection  frameAnimationDirection;

	int8_t              lastTransitionDirection;

	uint16_t            ticksPerFrame; // ~ 5000ms at 30 FPS
	uint16_t            ticksPerTransition;  // ~  500ms at 30 FPS

	bool                autoTransition;

	FrameCallback*      frameFunctions;
	uint8_t             frameCount;

	// Internally used to transition to a specific frame
	int8_t              nextFrameNumber;

	// Values for Overlays
	OverlayCallback*    overlayFunctions;
	uint8_t             overlayCount;

	// Will the Indicator be drawen
	// 3 Not drawn in both frames
	// 2 Drawn this frame but not next
	// 1 Not drown this frame but next
	// 0 Not known yet
	uint8_t indicatorDrawState;

	// Loading screen
	LoadingDrawFunction loadingDrawFunction;
	// UI State
	OLEDDisplayUiState      state;

	// Bookeeping for update
	uint8_t             updateInterval;
	
} Display_t;


/* Private variable */
static Display_t Oled;

static char* utf8ascii(char *str) ;


static esp_err_t sendCommand(uint8_t command)
{
	esp_err_t ret = ESP_OK;
	if (!Oled.i2c_address) {
		spi_cmd(command);
	} else {
		uint8_t buff[4];
		buff[0] = 0x80;
		buff[1] = command;
		ret = I2CWrite(Oled.i2c_address, buff, 0x2);
		if (ret == ESP_FAIL) {
	            printf("I2C Fail\n");
	        }
	}
	return ret;
}

/* Send all the init commands */
static void sendOledInitCommands(void) {

	sendCommand(DISPLAYOFF);
	sendCommand(SETDISPLAYCLOCKDIV);
	sendCommand(0xF0); // Increase speed of the display max ~96Hz
	sendCommand(SETMULTIPLEX);
	sendCommand(0x3F);
	sendCommand(SETDISPLAYOFFSET);
	sendCommand(0x00);
	sendCommand(SETSTARTLINE);
	sendCommand(CHARGEPUMP);
	sendCommand(0x14);
	sendCommand(MEMORYMODE);
	sendCommand(0x00);
	sendCommand(SEGREMAP);
	sendCommand(COMSCANINC);
	sendCommand(SETCOMPINS);
	sendCommand(0x12);
	sendCommand(SETCONTRAST);
	sendCommand(0xCF);
	sendCommand(SETPRECHARGE);
	sendCommand(0xF1);
	sendCommand(DISPLAYALLON_RESUME);
	sendCommand(NORMALDISPLAY);
	sendCommand(0x2e);            // stop scroll
	sendCommand(DISPLAYON);
/*
sendCommand(0xAE); //display off
sendCommand(0x20); //Set Memory Addressing Mode
sendCommand(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
sendCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
sendCommand(0xC8); //Set COM Output Scan DirectionsendCommand(command)
sendCommand(0x00); //---set low column address
sendCommand(0x10); //---set high column address
sendCommand(0x40); //--set start line address
sendCommand(0x81); //--set contrast control register
sendCommand(0xFF);
sendCommand(0xA1); //--set segment re-map 0 to 127
sendCommand(0xA6); //--set normal display
sendCommand(0xA8); //--set multiplex ratio(1 to 64)
sendCommand(0x3F); //
sendCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
sendCommand(0xD3); //-set display offset
sendCommand(0x00); //-not offset
sendCommand(0xD5); //--set display clock divide ratio/oscillator frequency
sendCommand(0xF0); //--set divide ratio
sendCommand(0xD9); //--set pre-charge period
sendCommand(0x22); //
sendCommand(0xDA); //--set com pins hardware configuration
sendCommand(0x12);
sendCommand(0xDB); //--set vcomh
sendCommand(0x20); //0x20,0.77xVcc
sendCommand(0x8D); //--set DC-DC enable
sendCommand(0x14); //
sendCommand(0xAF); //--turn on SSD1306 panel
*/
}

void static inline drawInternal(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const char *data, uint16_t offset, uint16_t bytesInData) {
	if (width < 0 || height < 0) return;
	if (yMove + height < 0 || yMove > DISPLAY_HEIGHT)  return;
	if (xMove + width  < 0 || xMove > DISPLAY_WIDTH)   return;

	uint8_t  rasterHeight = 1 + ((height - 1) >> 3); // fast ceil(height / 8.0)
	int8_t   yOffset      = yMove & 7;

	bytesInData = bytesInData == 0 ? width * rasterHeight : bytesInData;

	int16_t initYMove   = yMove;
	int8_t  initYOffset = yOffset;

	for (uint16_t i = 0; i < bytesInData; i++) {
		// Reset if next horizontal drawing phase is started.
		if ( i % rasterHeight == 0) {
			yMove   = initYMove;
			yOffset = initYOffset;
		}

		uint8_t currentByte = data[offset + i];
		int16_t xPos = xMove + (i / rasterHeight);
		int16_t yPos = ((yMove >> 3) + (i % rasterHeight)) * DISPLAY_WIDTH;

//		int16_t yScreenPos = yMove + yOffset;
		int16_t dataPos    = xPos  + yPos;

		if (dataPos >=  0  && dataPos < DISPLAY_BUFFER_SIZE &&
		    xPos >=  0  && xPos < DISPLAY_WIDTH ) {

			if (yOffset >= 0) {
				switch (Oled.color) {
				case WHITE:
					Display_Buffer[dataPos] |= currentByte << yOffset;
					break;
				case BLACK:
					Display_Buffer[dataPos] &= ~(currentByte << yOffset);
					break;
				case INVERSE:
					Display_Buffer[dataPos] ^= currentByte << yOffset;
					break;
				}
				if (dataPos < (DISPLAY_BUFFER_SIZE - DISPLAY_WIDTH)) {
					switch (Oled.color) {
					case WHITE:
						Display_Buffer[dataPos + DISPLAY_WIDTH] |= currentByte >> (8 - yOffset);
						break;
					case BLACK:
						Display_Buffer[dataPos + DISPLAY_WIDTH] &= ~(currentByte >> (8 - yOffset));
						break;
					case INVERSE:
						Display_Buffer[dataPos + DISPLAY_WIDTH] ^= currentByte >> (8 - yOffset);
						break;
					}
				}
			} else {
				// Make new offset position
				yOffset = -yOffset;

				switch (Oled.color) {
				case WHITE:
					Display_Buffer[dataPos] |= currentByte >> yOffset;
					break;
				case BLACK:
					Display_Buffer[dataPos] &= ~(currentByte >> yOffset);break;
				case INVERSE:
					Display_Buffer[dataPos] ^= currentByte >> yOffset;
					break;
				}

				// Prepare for next iteration by moving one block up
				yMove -= 8;

				// and setting the new yOffset
				yOffset = 8 - yOffset;
			}
		}
	}
}
	

 // Loading screen
static void startLoadingScreen(LoadingStage* stage, uint8_t progress) {
	oledSetTextAlignment(TEXT_ALIGN_CENTER);
	oledSetFont(ArialMT_Plain_10);
	oledDrawString(64, 18, (char *)stage->process);
	oledDrawProgressBar(4, 32, 120, 8, progress);
}


uint8_t SH1106_Init(uint8_t i2c_addr) {
	/* Fill privete structure */
	Oled.i2c_address = i2c_addr;

	/* Set default values */
	Oled.CurrentX = 0;
	Oled.CurrentY = 0;
	Oled.textAlignment = TEXT_ALIGN_LEFT;
	Oled.color = WHITE;
	Oled.fontData = ArialMT_Plain_10;
	Oled.logBufferSize = 0;
	Oled.logBufferFilled = 0;
	Oled.logBufferLine = 0;
	Oled.logBufferMaxLines = 0;
	Oled.logBuffer = NULL;

	// UI
	// Symbols for the Indicator
	Oled.indicatorPosition = BOTTOM;
	Oled.indicatorDirection = LEFT_RIGHT;
	Oled.activeSymbol  = ANIMATION_activeSymbol;
	Oled.inactiveSymbol = ANIMATION_inactiveSymbol;
	Oled.shouldDrawIndicators = true;
	// Values for the Frames
	Oled.frameAnimationDirection = SLIDE_RIGHT;

	Oled.lastTransitionDirection = 1;

	Oled.ticksPerFrame = 151; // ~ 5000ms at 30 FPS
	Oled.ticksPerTransition = 15;  // ~  500ms at 30 FPS

	Oled.autoTransition = true;

	Oled.frameCount = 0;

	// Internally used to transition to a specific frame
	Oled.nextFrameNumber           = -1;

	// Values for Overlays
	Oled.overlayFunctions=NULL;
	Oled.overlayCount = 0;

	// Will the Indicator be drawen
	// 3 Not drawn in both frames
	// 2 Drawn this frame but not next
	// 1 Not drown this frame but next
	// 0 Not known yet
	Oled.indicatorDrawState = 1;

	// Loading screen
	Oled.loadingDrawFunction = startLoadingScreen;

	// UI State
	Oled.state.lastUpdate  = 0;
	Oled.state.ticksSinceLastStateSwitch = 0;
	Oled.state.frameState = FIXED;
	Oled.state.currentFrame = 0;
	Oled.state.isIndicatorDrawen = true;
	// Normal = 1, Inverse = -1;
	Oled.state.frameTransitionDirection = 1;
	Oled.state.manuelControll = false;
	// Custom data that can be used by the user
	Oled.state.userData = NULL;


	// Bookeeping for update
	Oled.updateInterval            = 33;

	/* Init LCD */
	sendOledInitCommands();

	/* Clear screen */
	oledResetDisplay();

	/* Update screen */
	UpdateScreen();


	/* Initialized OK */
	Oled.Initialized = 1;
	/* Return OK */
	return 1;
}

void UpdateScreen(void) {
	uint8_t m;
        esp_err_t ret;
	for (m = 0; m < 8; m++) {
		sendCommand(0xB0 + m);
		sendCommand(0x02);
		sendCommand(0x10);

		if (!Oled.i2c_address) {
			spi_data(&Display_Buffer[DISPLAY_WIDTH * m], DISPLAY_WIDTH);
		} else {
			I2CBuffer[0] = 0x40;
			memcpy(&I2CBuffer[1], &Display_Buffer[DISPLAY_WIDTH * m], DISPLAY_WIDTH);
			ret = I2CWrite(Oled.i2c_address, I2CBuffer, DISPLAY_WIDTH+1);
			if (ret == ESP_FAIL) printf("I2C Fail\n");
		}
	}
}

void oledDisplayClear(void) {
	memset(Display_Buffer, 0, DISPLAY_BUFFER_SIZE);
}

/* Cycle through the initialization */
void oledResetDisplay(void) {
	oledDisplayClear();
	UpdateScreen();
}


/* Sets the color of all pixel operations */
void oledSetColor(OLEDDISPLAY_COLOR color) {
	Oled.color = color;
}


/* Draw a pixel at given position */
void oledSetPixel(int16_t x, int16_t y) {
	if (x >= 0 && x < 128 && y >= 0 && y < 64) {
		switch (Oled.color) {
		case WHITE:
			Display_Buffer[x + (y / 8) * DISPLAY_WIDTH] |=  (1 << (y & 7)); 
			break;
		case BLACK:
			Display_Buffer[x + (y / 8) * DISPLAY_WIDTH] &= ~(1 << (y & 7)); 
			break;
		case INVERSE:
			Display_Buffer[x + (y / 8) * DISPLAY_WIDTH] ^=  (1 << (y & 7)); 
			break;
		}
	}
}

/*
 * Draw a line from position 0 to position 1
 * Bresenham's algorithm - thx wikipedia and Adafruit_GFX
 **/
void oledDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	int16_t dx, dy;
	int16_t ystep;

	if (steep) {
		_swap_int16_t(x0, y0);
		_swap_int16_t(x1, y1);
	}

	if (x0 > x1) {
		_swap_int16_t(x0, x1);
		_swap_int16_t(y0, y1);
	}

	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;

	if (y0 < y1) ystep = 1;
	else ystep = -1;

	for (; x0<=x1; x0++) {
		if (steep) {
			oledSetPixel(y0, x0);
		} else {
			oledSetPixel(x0, y0);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

/* Draw the border of a rectangle at the given location */
void oledDrawRect(int16_t x, int16_t y, int16_t width, int16_t height) {
	oledDrawHorizontalLine(x, y, width);
	oledDrawVerticalLine(x, y, height);
	oledDrawVerticalLine(x + width - 1, y, height);
	oledDrawHorizontalLine(x, y + height - 1, width);
}

/* Fill the rectangle */
void oledFillRect(int16_t xMove, int16_t yMove, int16_t width, int16_t height) {
	for (int16_t x = xMove; x < xMove + width; x++) {
		oledDrawVerticalLine(x, yMove, height);
	}
}

/* Draw the border of a circle */
void oledDrawCircle(int16_t x0, int16_t y0, int16_t radius) {
	int16_t x = 0, y = radius;
	int16_t dp = 1 - radius;

	do {
		if (dp < 0)
			dp = dp + 2 * (++x) + 3;
		else
			dp = dp + 2 * (++x) - 2 * (--y) + 5;

		oledSetPixel(x0 + x, y0 + y);     //For the 8 octants
		oledSetPixel(x0 - x, y0 + y);
		oledSetPixel(x0 + x, y0 - y);
		oledSetPixel(x0 - x, y0 - y);
		oledSetPixel(x0 + y, y0 + x);
		oledSetPixel(x0 - y, y0 + x);
		oledSetPixel(x0 + y, y0 - x);
		oledSetPixel(x0 - y, y0 - x);

	} while (x < y);

	oledSetPixel(x0 + radius, y0);
	oledSetPixel(x0, y0 + radius);
	oledSetPixel(x0 - radius, y0);
	oledSetPixel(x0, y0 - radius);
}

/* Draw all Quadrants specified in the quads bit mask */
void oledDrawCircleQuads(int16_t x0, int16_t y0, int16_t radius, uint8_t quads) {
	int16_t x = 0, y = radius;
	int16_t dp = 1 - radius;
	while (x < y) {
		if (dp < 0)
			dp = dp + 2 * (++x) + 3;
		else
			dp = dp + 2 * (++x) - 2 * (--y) + 5;
		if (quads & 0x1) {
			oledSetPixel(x0 + x, y0 - y);
			oledSetPixel(x0 + y, y0 - x);
		}
		if (quads & 0x2) {
			oledSetPixel(x0 - y, y0 - x);
			oledSetPixel(x0 - x, y0 - y);
		}
		if (quads & 0x4) {
			oledSetPixel(x0 - y, y0 + x);
			oledSetPixel(x0 - x, y0 + y);
		}
		if (quads & 0x8) {
			oledSetPixel(x0 + x, y0 + y);
			oledSetPixel(x0 + y, y0 + x);
		}
	}
	if (quads & 0x1 && quads & 0x8) {
		oledSetPixel(x0 + radius, y0);
	}
	if (quads & 0x4 && quads & 0x8) {
		oledSetPixel(x0, y0 + radius);
	}
	if (quads & 0x2 && quads & 0x4) {
		oledSetPixel(x0 - radius, y0);
	}
	if (quads & 0x1 && quads & 0x2) {
		oledSetPixel(x0, y0 - radius);
	}
}

/* Fill circle */
void oledFillCircle(int16_t x0, int16_t y0, int16_t radius) {
	int16_t x = 0, y = radius;
	int16_t dp = 1 - radius;

	do {
		if (dp < 0)
			dp = dp + 2 * (++x) + 3;
		else
		dp = dp + 2 * (++x) - 2 * (--y) + 5;

		oledDrawHorizontalLine(x0 - x, y0 - y, 2*x);
		oledDrawHorizontalLine(x0 - x, y0 + y, 2*x);
		oledDrawHorizontalLine(x0 - y, y0 - x, 2*y);
		oledDrawHorizontalLine(x0 - y, y0 + x, 2*y);
	} while (x < y);
	oledDrawHorizontalLine(x0 - radius, y0, 2 * radius);
}

/* Draw a line horizontally */
void oledDrawHorizontalLine(int16_t x, int16_t y, int16_t length) {
	if (y < 0 || y >= DISPLAY_HEIGHT) { return; }

	if (x < 0) {
		length += x;
		x = 0;
	}

	if ( (x + length) > DISPLAY_WIDTH) {
		length = (DISPLAY_WIDTH - x);
	}

	if (length <= 0) { return; }

	uint8_t * bufferPtr = Display_Buffer;
	bufferPtr += (y >> 3) * DISPLAY_WIDTH;
	bufferPtr += x;

	uint8_t drawBit = 1 << (y & 7);

	switch (Oled.color) {
	case WHITE:
		while (length--) {
			*bufferPtr++ |= drawBit;
		};
		break;
	case BLACK:
		drawBit = ~drawBit;
		while (length--) {
			*bufferPtr++ &= drawBit;
		};
		break;
	case INVERSE:
		while (length--) {
			*bufferPtr++ ^= drawBit;
		};
		break;
	}
}

/* Draw a line vertically */
void oledDrawVerticalLine(int16_t x, int16_t y, int16_t length) {
	if (x < 0 || x >= DISPLAY_WIDTH) return;

	if (y < 0) {
		length += y;
		y = 0;
	}

	if ( (y + length) > DISPLAY_HEIGHT) {
		length = (DISPLAY_HEIGHT - y);
	}

	if (length <= 0) return;

	uint8_t yOffset = y & 7;
	uint8_t drawBit;
	uint8_t *bufferPtr = Display_Buffer;

	bufferPtr += (y >> 3) * DISPLAY_WIDTH;
	bufferPtr += x;

	if (yOffset) {
		yOffset = 8 - yOffset;
		drawBit = ~(0xFF >> (yOffset));

		if (length < yOffset) {
			drawBit &= (0xFF >> (yOffset - length));
		}

		switch (Oled.color) {
		case WHITE:
			*bufferPtr |=  drawBit;
			break;
		case BLACK:
			*bufferPtr &= ~drawBit;
			break;
		case INVERSE:
			*bufferPtr ^=  drawBit;
			break;
		}

		if (length < yOffset) return;

		length -= yOffset;
		bufferPtr += DISPLAY_WIDTH;
	}

	if (length >= 8) {
		switch (Oled.color) {
		case WHITE:
		case BLACK:
			drawBit = (Oled.color == WHITE) ? 0xFF : 0x00;
			do {
				*bufferPtr = drawBit;
				bufferPtr += DISPLAY_WIDTH;
				length -= 8;
			} while (length >= 8);
			break;
		case INVERSE:
			do {
				*bufferPtr = ~(*bufferPtr);
				bufferPtr += DISPLAY_WIDTH;
				length -= 8;
			} while (length >= 8);
			break;
		}
	}

	if (length > 0) {
		drawBit = (1 << (length & 7)) - 1;
		switch (Oled.color) {
		case WHITE:
			*bufferPtr |=  drawBit;
			break;
		case BLACK:
			*bufferPtr &= ~drawBit;
			break;
		case INVERSE:
			*bufferPtr ^=  drawBit;
			break;
		}
	}
}

/*
 *  Draws a rounded progress bar with the outer dimensions given by 
 *  width and height. Progress is
 * a unsigned byte value between 0 and 100
 */
void oledDrawProgressBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t progress) {
	uint16_t radius = height / 2;
	uint16_t xRadius = x + radius;
	uint16_t yRadius = y + radius;
	uint16_t doubleRadius = 2 * radius;
	uint16_t innerRadius = radius - 2;

	oledSetColor(WHITE);
	oledDrawCircleQuads(xRadius, yRadius, radius, 0b00000110);
	oledDrawHorizontalLine(xRadius, y, width - doubleRadius + 1);
	oledDrawHorizontalLine(xRadius, y + height, width - doubleRadius + 1);
	oledDrawCircleQuads(x + width - radius, yRadius, radius, 0b00001001);

	uint16_t maxProgressWidth = (width - doubleRadius - 1) * progress / 100;

	oledFillCircle(xRadius, yRadius, innerRadius);
	oledFillRect(xRadius + 1, y + 2, maxProgressWidth, height - 3);
	oledFillCircle(xRadius + maxProgressWidth, yRadius, innerRadius);
}

/* Draw a bitmap in the internal image format */
void oledDrawFastImage(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const char *image) {
	drawInternal(xMove, yMove, width, height, image, 0, 0);
}

/* Draw a XBM */
void oledDrawXbm(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const char *xbm) {
	int16_t widthInXbm = (width + 7) / 8;
	uint8_t data = 0;

	for(int16_t y = 0; y < height; y++) {
		for(int16_t x = 0; x < width; x++ ) {
			if (x & 7) {
				data >>= 1; // Move a bit
			} else {  // Read new data every 8 bit
				data = xbm[(x / 8) + y * widthInXbm];
			}
			// if there is a bit draw it
			if (data & 0x01) {
				oledSetPixel(xMove + x, yMove + y);
			}
		}
	}
}



static void drawStringInternal(int16_t xMove, int16_t yMove, char* text, uint16_t textLength, uint16_t textWidth) {
	uint8_t textHeight       = Oled.fontData[HEIGHT_POS];
	uint8_t firstChar        = Oled.fontData[FIRST_CHAR_POS];

	uint16_t sizeOfJumpTable = Oled.fontData[CHAR_NUM_POS]*JUMPTABLE_BYTES;
	uint8_t cursorX         = 0;
	uint8_t cursorY         = 0;

	switch (Oled.textAlignment) {
	case TEXT_ALIGN_CENTER_BOTH:
		yMove -= textHeight >> 1;
	// Fallthrough
	case TEXT_ALIGN_CENTER:
		xMove -= textWidth >> 1; // divide by 2
		break;
	case TEXT_ALIGN_RIGHT:
		xMove -= textWidth;
		break;
	case TEXT_ALIGN_LEFT:
		break;
	}

	// Don't draw anything if it is not on the screen.
	if (xMove + textWidth  < 0 || xMove > DISPLAY_WIDTH ) {return;}
	if (yMove + textHeight < 0 || yMove > DISPLAY_HEIGHT) {return;}

	for (uint16_t j = 0; j < textLength; j++) {
		int16_t xPos = xMove + cursorX;
		int16_t yPos = yMove + cursorY;

		uint8_t code = text[j];
		if (code >= firstChar) {
			uint8_t charCode = code - firstChar;

			// 4 Bytes per char code
			uint8_t msbJumpToChar    = Oled.fontData[JUMPTABLE_START + charCode * JUMPTABLE_BYTES];                  // MSB  \ JumpAddress
			uint8_t lsbJumpToChar    = Oled.fontData[JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_LSB];   // LSB /
			uint8_t charByteSize     = Oled.fontData[JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_SIZE];  // Size
			uint8_t currentCharWidth = Oled.fontData[JUMPTABLE_START + charCode * JUMPTABLE_BYTES + JUMPTABLE_WIDTH]; // Width
	
			// Test if the char is drawable
			if (!(msbJumpToChar == 255 && lsbJumpToChar == 255)) {
				// Get the position of the char data
				uint16_t charDataPosition = JUMPTABLE_START + sizeOfJumpTable + ((msbJumpToChar << 8) + lsbJumpToChar);
				drawInternal(xPos, yPos, currentCharWidth, textHeight, Oled.fontData, charDataPosition, charByteSize);
			}
			cursorX += currentCharWidth;
		}
	}
}


/* Draws a string at the given location */
void oledDrawString(int16_t xMove, int16_t yMove, char *strUser) {
	uint16_t lineHeight = Oled.fontData[HEIGHT_POS];

	// char* text must be freed!
	char* text = utf8ascii(strUser);
		
	uint16_t yOffset = 0;
	// If the string should be centered vertically too
	// we need to now how heigh the string is.
	if (Oled.textAlignment == TEXT_ALIGN_CENTER_BOTH) {
		uint16_t lb = 0;
		// Find number of linebreaks in text
		for (uint16_t i=0;text[i] != 0; i++) {
			lb += (text[i] == 10);
		}
		// Calculate center
		yOffset = (lb * lineHeight) / 2;
	}

	uint16_t line = 0;
	char* textPart = strtok(text,"\n");
	while (textPart != NULL) {
		uint16_t length = strlen(textPart);
		drawStringInternal(xMove, yMove - yOffset + (line++) * lineHeight, textPart, length, oledGetStringWidth(textPart, length));
		textPart = strtok(NULL, "\n");
	}
	free(text);
}

/*
 *Draws a String with a maximum width at the given location.
 * If the given String is wider than the specified width
 * The text will be wrapped to the next line at a space or dash
 */
void oledDrawStringMaxWidth(int16_t xMove, int16_t yMove, uint16_t maxLineWidth, char *strUser) {
	uint16_t firstChar  = Oled.fontData[FIRST_CHAR_POS];
	uint16_t lineHeight = Oled.fontData[HEIGHT_POS];
	char* text = utf8ascii(strUser);

	uint16_t length = strlen(text);
	uint16_t lastDrawnPos = 0;
	uint16_t lineNumber = 0;
	uint16_t strWidth = 0;

	uint16_t preferredBreakpoint = 0;
	uint16_t widthAtBreakpoint = 0;

	for (uint16_t i = 0; i < length; i++) {
		strWidth += Oled.fontData[JUMPTABLE_START + (text[i] - firstChar) * JUMPTABLE_BYTES + JUMPTABLE_WIDTH];

		// Always try to break on a space or dash
		if (text[i] == ' ' || text[i]== '-') {
			preferredBreakpoint = i;
			widthAtBreakpoint = strWidth;
		}

		if (strWidth >= maxLineWidth) {
			if (preferredBreakpoint == 0) {
				preferredBreakpoint = i;
				widthAtBreakpoint = strWidth;
			}
			drawStringInternal(xMove, yMove + (lineNumber++) * lineHeight , &text[lastDrawnPos], preferredBreakpoint - lastDrawnPos, widthAtBreakpoint);
			lastDrawnPos = preferredBreakpoint + 1;
			// It is possible that we did not draw all letters to i so we need
			// to account for the width of the chars from `i - preferredBreakpoint`
			// by calculating the width we did not draw yet.
			strWidth = strWidth - widthAtBreakpoint;
			preferredBreakpoint = 0;
		}
	}

	// Draw last part if needed
	if (lastDrawnPos < length) {
		drawStringInternal(xMove, yMove + lineNumber * lineHeight , &text[lastDrawnPos], length - lastDrawnPos, oledGetStringWidth(&text[lastDrawnPos], length - lastDrawnPos));
	}

	free(text);
}

/* Returns the width of the const char* with the current font settings */
uint16_t oledGetStringWidth(const char* text, uint16_t length) {
	uint16_t firstChar = Oled.fontData[FIRST_CHAR_POS];
	uint16_t stringWidth = 0;
	uint16_t maxWidth = 0;

	while (length--) {
		stringWidth += Oled.fontData[JUMPTABLE_START + (text[length] - firstChar) * JUMPTABLE_BYTES + JUMPTABLE_WIDTH];
		if (text[length] == 10) {
			maxWidth = max(maxWidth, stringWidth);
			stringWidth = 0;
		}
	}
	return max(maxWidth, stringWidth);
}

/* Specifies relative to which anchor point */
void oledSetTextAlignment(OLEDDISPLAY_TEXT_ALIGNMENT textAlignment) {
	Oled.textAlignment = textAlignment;
}

/* Sets the current font */
void oledSetFont(const char *fontData) {
	Oled.fontData = fontData;
}

/* Turn the display on */
void oledDisplayOn(void) {
	sendCommand(DISPLAYON);
}

/* Turn the display offs */
void oledDisplayOff(void) {
	sendCommand(DISPLAYOFF);
}

/* Inverted display mode */
void oledInvertDisplay(void) {
	sendCommand(INVERTDISPLAY);
}

/* Normal display mode */
void oledNormalDisplay(void) {
	sendCommand(NORMALDISPLAY);
}

/* Set display contrast */
void oledSetContrast(char contrast) {
	sendCommand(SETCONTRAST);
	sendCommand(contrast);
}

/* Turn the display upside down */
void oledFlipScreenVertically() {
	sendCommand(SEGREMAP | 0x01);
	sendCommand(COMSCANDEC);           //Rotate screen 180 Deg
}

/* Code form http://playground.arduino.cc/Main/Utf8ascii */
uint8_t utf_to_ascii(uint8_t ascii) {
	static uint8_t LASTCHAR;

	if ( ascii < 128 ) { // Standard ASCII-set 0..0x7F handling
		LASTCHAR = 0;
		return ascii;
	}

	uint8_t last = LASTCHAR;   // get last char
	LASTCHAR = ascii;

	switch (last) {    // conversion depnding on first UTF8-character
	case 0xC2: return  (ascii);  break;
	case 0xC3: return  (ascii | 0xC0);  break;
	case 0x82: if (ascii == 0xAC) return (0x80);    // special case Euro-symbol
	}

	return  0; // otherwise: return zero, if character has to be ignored
}

/* You need to free the char! */
static char* utf8ascii(char *str) {
	uint16_t k = 0;
	uint16_t length = strlen(str)+1;

	// Copy the string into a char array
	char* s = (char*) malloc(length * sizeof(char));
	if(!s) {
		printf("[OLEDDISPLAY][utf8ascii] Can't allocate another char array. Drop support for UTF-8.\n");
		return str;
	}
	memcpy(s, str, length);
	length--;

	for (uint16_t i=0; i < length; i++) {
		char c = utf_to_ascii(s[i]);
		if (c!=0) {
			s[k++]=c;
		}
	}
	s[k]=0;

	// This will leak 's' be sure to free it in the calling function.
	return s;
}

/* Draw the log buffer at position (x, y) */
void oledDrawLogBuffer(uint16_t xMove, uint16_t yMove) {
	uint16_t lineHeight = Oled.fontData[HEIGHT_POS];
	// Always align left
	oledSetTextAlignment(TEXT_ALIGN_LEFT);

	// State values
	uint16_t length   = 0;
	uint16_t line     = 0;
	uint16_t lastPos  = 0;

	for (uint16_t i=0; i<Oled.logBufferFilled;i++){
		// Everytime we have a \n print
		if (Oled.logBuffer[i] == 10) {
			length++;
			// Draw string on line `line` from lastPos to length
			// Passing 0 as the lenght because we are in TEXT_ALIGN_LEFT
			drawStringInternal(xMove, yMove + (line++) * lineHeight, &Oled.logBuffer[lastPos], length, 0);
			// Remember last pos
			lastPos = i;
			// Reset length
			length = 0;
		} else {
			// Count chars until next linebreak
			length++;
		}
	}
	// Draw the remaining string
	if (length > 0) {
		drawStringInternal(xMove, yMove + line * lineHeight, &Oled.logBuffer[lastPos], length, 0);
	}
}

/*
 * This will define the lines and characters you can
 * print to the screen. When you exeed the buffer size (lines * chars)
 * the output may be truncated due to the size constraint.
 */
bool oledSetLogBuffer(uint16_t lines, uint16_t chars) {
	if (Oled.logBuffer != NULL) free(Oled.logBuffer);
	uint16_t size = lines * chars;
	if (size > 0) {
		Oled.logBufferLine     = 0;      // Lines printed
		Oled.logBufferMaxLines = lines;  // Lines max printable
		Oled.logBufferSize     = size;   // Total number of characters the buffer can hold
		Oled.logBuffer = (char *) malloc(size * sizeof(uint8_t));
		if (!Oled.logBuffer) {
			printf("[OLEDDISPLAY][setLogBuffer] Not enough memory to create log buffer\n");
			return false;
		}
	}
	return true;
}

size_t oledPutc(uint8_t c) {
	if (Oled.logBufferSize > 0) {
		// Don't waste space on \r\n line endings, dropping \r
		if (c == 13) return 1;

		bool maxLineNotReached = Oled.logBufferLine < Oled.logBufferMaxLines;
		bool bufferNotFull = Oled.logBufferFilled < Oled.logBufferSize;

		// Can we write to the buffer?
		if (bufferNotFull && maxLineNotReached) {
			Oled.logBuffer[Oled.logBufferFilled] = utf_to_ascii(c);
			Oled.logBufferFilled++;
			// Keep track of lines written
			if (c == 10) Oled.logBufferLine++;
		} else {
			// Max line number is reached
			if (!maxLineNotReached) Oled.logBufferLine--;

			// Find the end of the first line
			uint16_t firstLineEnd = 0;
			for (uint16_t i=0; i<Oled.logBufferFilled;i++) {
				if (Oled.logBuffer[i] == 10){
					// Include last char too
					firstLineEnd = i + 1;
					break;
				}
			}
			// If there was a line ending
			if (firstLineEnd > 0) {
				// Calculate the new logBufferFilled value
				Oled.logBufferFilled = Oled.logBufferFilled - firstLineEnd;
				// Now we move the lines infront of the buffer
				memcpy(Oled.logBuffer, &Oled.logBuffer[firstLineEnd], Oled.logBufferFilled);
			} else {
				// Let's reuse the buffer if it was full
				if (!bufferNotFull) {
					Oled.logBufferFilled = 0;
				}// else {
			        //  Nothing to do here
			        //}
			}
			oledPutc(c);
		}
	}
	// We are always writing all uint8_t to the buffer
	return 1;
}

size_t oledWrite(const char* str) {
	if (str == NULL) return 0;
	size_t length = strlen(str);
	for (size_t i = 0; i < length; i++) {
		oledPutc(str[i]);
	}
	return length;
}




void setTargetFPS(uint8_t fps){
	float oldInterval = Oled.updateInterval;
	Oled.updateInterval = ((float) 1.0 / (float) fps) * 1000;

	// Calculate new ticksPerFrame
	float changeRatio = oldInterval / (float) Oled.updateInterval;
	Oled.ticksPerFrame *= changeRatio;
	Oled.ticksPerTransition *= changeRatio;
}

// -/------ Automatic controll ------\-

void enableAutoTransition(){
	Oled.autoTransition = true;
}
void disableAutoTransition(){
	Oled.autoTransition = false;
}
void setAutoTransitionForwards(){
	Oled.state.frameTransitionDirection = 1;
	Oled.lastTransitionDirection = 1;
}
void setAutoTransitionBackwards(){
	Oled.state.frameTransitionDirection = -1;
	Oled.lastTransitionDirection = -1;
}
void setTimePerFrame(uint16_t time){
	Oled.ticksPerFrame = (int) ( (float) time / (float) Oled.updateInterval);
}
void setTimePerTransition(uint16_t time){
	Oled.ticksPerTransition = (int) ( (float) time / (float) Oled.updateInterval);
}


// -/------ Customize indicator position and style -------\-
void enableIndicator(){
	Oled.state.isIndicatorDrawen = true;
}

void disableIndicator(){
	Oled.state.isIndicatorDrawen = false;
}

void enableAllIndicators(){
	Oled.shouldDrawIndicators = true;
}

void disableAllIndicators(){
	Oled.shouldDrawIndicators = false;
}

void setIndicatorPosition(IndicatorPosition pos) {
	Oled.indicatorPosition = pos;
}
void setIndicatorDirection(IndicatorDirection dir) {
	Oled.indicatorDirection = dir;
}
void setActiveSymbol(const char* symbol) {
	Oled.activeSymbol = symbol;
}
void setInactiveSymbol(const char* symbol) {
	Oled.inactiveSymbol = symbol;
}



// -/----- Frame settings -----\-
void setFrameAnimation(AnimationDirection dir) {
	Oled.frameAnimationDirection = dir;
}

void setFrames(FrameCallback* frameFunctions, uint8_t frameCount) {
	Oled.frameFunctions = frameFunctions;
	Oled.frameCount     = frameCount;
	UIresetState();
}


// -/----- Overlays ------\-
void setOverlays(OverlayCallback* overlayFunctions, uint8_t overlayCount){
	Oled.overlayFunctions = overlayFunctions;
	Oled.overlayCount     = overlayCount;
}

// -/----- Loading Process -----\-
void setLoadingDrawFunction(LoadingDrawFunction loadingDrawFunction) {
	Oled.loadingDrawFunction = loadingDrawFunction;
}

void UIrunLoadingProcess(LoadingStage* stages, uint8_t stagesCount) {
	uint8_t progress = 0;
	uint8_t increment = 100 / stagesCount;

	for (uint8_t i = 0; i < stagesCount; i++) {
		oledDisplayClear();
		Oled.loadingDrawFunction(&stages[i], progress);
		UpdateScreen();
		stages[i].callback();
		progress += increment;
		vPortYield();
	}

	oledDisplayClear();
	Oled.loadingDrawFunction(&stages[stagesCount-1], progress);
	UpdateScreen();
	vTaskDelay(150 / portTICK_PERIOD_MS);
}

// -/----- Manuel control -----\-
void nextFrame() {
	if (Oled.state.frameState != IN_TRANSITION) {
		Oled.state.manuelControll = true;
		Oled.state.frameState = IN_TRANSITION;
		Oled.state.ticksSinceLastStateSwitch = 0;
		Oled.lastTransitionDirection = Oled.state.frameTransitionDirection;
		Oled.state.frameTransitionDirection = 1;
	}
}
void previousFrame() {
	if (Oled.state.frameState != IN_TRANSITION) {
		Oled.state.manuelControll = true;
		Oled.state.frameState = IN_TRANSITION;
		Oled.state.ticksSinceLastStateSwitch = 0;
		Oled.lastTransitionDirection = Oled.state.frameTransitionDirection;
		Oled.state.frameTransitionDirection = -1;
	}
}

void switchToFrame(uint8_t frame) {
	if (frame >= Oled.frameCount) return;
	Oled.state.ticksSinceLastStateSwitch = 0;
	if (frame == Oled.state.currentFrame) return;
	Oled.state.frameState = FIXED;
	Oled.state.currentFrame = frame;
	Oled.state.isIndicatorDrawen = true;
}

void transitionToFrame(uint8_t frame) {
	if (frame >= Oled.frameCount) return;
	Oled.state.ticksSinceLastStateSwitch = 0;
	if (frame == Oled.state.currentFrame) return;
	Oled.nextFrameNumber = frame;
	Oled.lastTransitionDirection = Oled.state.frameTransitionDirection;
	Oled.state.manuelControll = true;
	Oled.state.frameState = IN_TRANSITION;
	Oled.state.frameTransitionDirection = frame < Oled.state.currentFrame ? -1 : 1;
}


// -/----- State information -----\-
OLEDDisplayUiState* getUiState(){
	return &Oled.state;
}


int8_t UIupdate(){
	long frameStart = xTaskGetTickCount() * portTICK_PERIOD_MS;
	int8_t timeBudget = Oled.updateInterval - (frameStart - Oled.state.lastUpdate);

//ESP_LOGI("test", "UI update %lu %d", frameStart, timeBudget);
	if (timeBudget <= 0) {
		// Implement frame skipping to ensure time budget is keept
		if (Oled.autoTransition && Oled.state.lastUpdate != 0) Oled.state.ticksSinceLastStateSwitch += ceil(-timeBudget / Oled.updateInterval);

		Oled.state.lastUpdate = frameStart;
		UItick();
	}
	return Oled.updateInterval - ((xTaskGetTickCount() * portTICK_PERIOD_MS) - frameStart);
}

void UItick() {
	Oled.state.ticksSinceLastStateSwitch++;

	switch (Oled.state.frameState) {
	case IN_TRANSITION:
		if (Oled.state.ticksSinceLastStateSwitch >= Oled.ticksPerTransition){
			Oled.state.frameState = FIXED;
			Oled.state.currentFrame = UIgetNextFrameNumber();
			Oled.state.ticksSinceLastStateSwitch = 0;
			Oled.nextFrameNumber = -1;
		}
		break;
	case FIXED:
		// Revert manuelControll
		if (Oled.state.manuelControll) {
			Oled.state.frameTransitionDirection = Oled.lastTransitionDirection;
			Oled.state.manuelControll = false;
		}
		if (Oled.state.ticksSinceLastStateSwitch >= Oled.ticksPerFrame){
			if (Oled.autoTransition){
				Oled.state.frameState = IN_TRANSITION;
			}
			Oled.state.ticksSinceLastStateSwitch = 0;
		}
		break;
	}

	oledDisplayClear();
	UIdrawFrame();
	if (Oled.shouldDrawIndicators) {
		UIdrawIndicator();
	}
	UIdrawOverlays();
	UpdateScreen();
}

void UIresetState() {
	Oled.state.lastUpdate = 0;
	Oled.state.ticksSinceLastStateSwitch = 0;
	Oled.state.frameState = FIXED;
	Oled.state.currentFrame = 0;
	Oled.state.isIndicatorDrawen = true;
}

void UIdrawFrame(){
	switch (Oled.state.frameState){
	case IN_TRANSITION: {
		float progress = (float) Oled.state.ticksSinceLastStateSwitch / (float) Oled.ticksPerTransition;
		int16_t x=0, y=0, x1=0, y1=0;
		switch(Oled.frameAnimationDirection){
		case SLIDE_LEFT:
			x = -128 * progress;
			y = 0;
			x1 = x + 128;
			y1 = 0;
			break;
		case SLIDE_RIGHT:
			x = 128 * progress;
			y = 0;
			x1 = x - 128;
			y1 = 0;
			break;
		case SLIDE_UP:
			x = 0;
			y = -64 * progress;
			x1 = 0;
			y1 = y + 64;
			break;
		case SLIDE_DOWN:
			x = 0;
			y = 64 * progress;
			x1 = 0;
			y1 = y - 64;
			break;
		}

		// Invert animation if direction is reversed.
		int8_t dir = Oled.state.frameTransitionDirection >= 0 ? 1 : -1;
		x *= dir; y *= dir; x1 *= dir; y1 *= dir;
		bool drawenCurrentFrame;


		// Prope each frameFunction for the indicator Drawen state
		enableIndicator();
		(Oled.frameFunctions[Oled.state.currentFrame])(&Oled.state, x, y);
		drawenCurrentFrame = Oled.state.isIndicatorDrawen;

		enableIndicator();
		(Oled.frameFunctions[UIgetNextFrameNumber()])(&Oled.state, x1, y1);

		// Build up the indicatorDrawState
		if (drawenCurrentFrame && !Oled.state.isIndicatorDrawen) {
			// Drawen now but not next
			Oled.indicatorDrawState = 2;
		} else if (!drawenCurrentFrame && Oled.state.isIndicatorDrawen) {
			// Not drawen now but next
			Oled.indicatorDrawState = 1;
		} else if (!drawenCurrentFrame && !Oled.state.isIndicatorDrawen) {
			// Not drawen in both frames
			Oled.indicatorDrawState = 3;
		}

		// If the indicator isn't draw in the current frame
		// reflect it in state.isIndicatorDrawen
		if (!drawenCurrentFrame) Oled.state.isIndicatorDrawen = false;
		break;
		}
	case FIXED:
		// Always assume that the indicator is drawn!
		// And set indicatorDrawState to "not known yet"
		Oled.indicatorDrawState = 0;
		enableIndicator();
		(Oled.frameFunctions[Oled.state.currentFrame])(&Oled.state, 0, 0);
		break;
	}
}

void UIdrawIndicator() {

	// Only draw if the indicator is invisible
	// for both frames or
	// the indiactor is shown and we are IN_TRANSITION
	if (Oled.indicatorDrawState == 3 || (!Oled.state.isIndicatorDrawen && Oled.state.frameState != IN_TRANSITION)) {
		return;
	}

	uint8_t posOfHighlightFrame = 0;
	float indicatorFadeProgress = 0;

	// if the indicator needs to be slided in we want to
	// highlight the next frame in the transition
	uint8_t frameToHighlight = Oled.indicatorDrawState == 1 ? UIgetNextFrameNumber() : Oled.state.currentFrame;

	// Calculate the frame that needs to be highlighted
	// based on the Direction the indiactor is drawn
	switch (Oled.indicatorDirection){
	case LEFT_RIGHT:
	        posOfHighlightFrame = frameToHighlight;
	        break;
	case RIGHT_LEFT:
		posOfHighlightFrame = Oled.frameCount - frameToHighlight;
	        break;
	}

	switch (Oled.indicatorDrawState) {
	case 1: // Indicator was not drawn in this frame but will be in next
		// Slide IN
		indicatorFadeProgress = 1 - ((float) Oled.state.ticksSinceLastStateSwitch / (float) Oled.ticksPerTransition);
		break;
	case 2: // Indicator was drawn in this frame but not in next
		// Slide OUT
		indicatorFadeProgress = ((float) Oled.state.ticksSinceLastStateSwitch / (float) Oled.ticksPerTransition);
		break;
	}

	uint16_t frameStartPos = (12 * Oled.frameCount / 2);
	const char *image;
	uint16_t x=0,y=0;
	for (uint8_t i = 0; i < Oled.frameCount; i++) {
		switch (Oled.indicatorPosition){
		case TOP:
			y = 0 - (8 * indicatorFadeProgress);
			x = 64 - frameStartPos + 12 * i;
			break;
		case BOTTOM:
			y = 56 + (8 * indicatorFadeProgress);
			x = 64 - frameStartPos + 12 * i;
			break;
		case RIGHT:
			x = 120 + (8 * indicatorFadeProgress);
			y = 32 - frameStartPos + 2 + 12 * i;
			break;
		case LEFT:
			x = 0 - (8 * indicatorFadeProgress);
			y = 32 - frameStartPos + 2 + 12 * i;
			break;
		}

		if (posOfHighlightFrame == i) {
			image = Oled.activeSymbol;
		} else {
			image = Oled.inactiveSymbol;
		}
		oledDrawFastImage(x, y, 8, 8, image);
	}
}

void UIdrawOverlays() {
	for (uint8_t i=0; i<Oled.overlayCount;i++){
		(Oled.overlayFunctions[i])(&Oled.state);
	}
}

uint8_t UIgetNextFrameNumber(){
	if (Oled.nextFrameNumber != -1) return Oled.nextFrameNumber;
	return (Oled.state.currentFrame + Oled.frameCount + Oled.state.frameTransitionDirection) % Oled.frameCount;
}



