#ifndef SH1106_H
#define SH1106_H
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "stdlib.h"
#include "string.h"
#include "OLEDDisplayFonts.h"



/* Display setting */
#define DISPLAY_WIDTH	128
#define DISPLAY_HEIGHT	64
#define DISPLAY_BUFFER_SIZE DISPLAY_WIDTH * DISPLAY_HEIGHT / 8

#define WIDTH_POS 0
#define HEIGHT_POS 1
#define FIRST_CHAR_POS 2
#define CHAR_NUM_POS 3

// Header Values
#define JUMPTABLE_BYTES 4
#define JUMPTABLE_LSB   1
#define JUMPTABLE_SIZE  2
#define JUMPTABLE_WIDTH 3
#define JUMPTABLE_START 4



/* Display commands */
#define CHARGEPUMP 0x8D
#define COLUMNADDR 0x21
#define COMSCANDEC 0xC8
#define COMSCANINC 0xC0
#define DISPLAYALLON 0xA5
#define DISPLAYALLON_RESUME 0xA4
#define DISPLAYOFF 0xAE
#define DISPLAYON 0xAF
#define EXTERNALVCC 0x1
#define INVERTDISPLAY 0xA7
#define MEMORYMODE 0x20
#define NORMALDISPLAY 0xA6
#define PAGEADDR 0x22
#define SEGREMAP 0xA0
#define SETCOMPINS 0xDA
#define SETCONTRAST 0x81
#define SETDISPLAYCLOCKDIV 0xD5
#define SETDISPLAYOFFSET 0xD3
#define SETHIGHCOLUMN 0x10
#define SETLOWCOLUMN 0x00
#define SETMULTIPLEX 0xA8
#define SETPRECHARGE 0xD9
#define SETSEGMENTREMAP 0xA1
#define SETSTARTLINE 0x40
#define SETVCOMDETECT 0xDB
#define SWITCHCAPVCC 0x2



/**
 * @brief  Oled display color enumeration
 */
typedef enum {
	BLACK = 0,
	WHITE = 1,
	INVERSE = 2
} OLEDDISPLAY_COLOR;

typedef enum {
	SH1106_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
	SH1106_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
} SH1106_COLOR_t;


/**
 * @brief  Oled display text aligment enumeration
 */
typedef enum {
	TEXT_ALIGN_LEFT = 0,
	TEXT_ALIGN_RIGHT = 1,
	TEXT_ALIGN_CENTER = 2,
	TEXT_ALIGN_CENTER_BOTH = 3
} OLEDDISPLAY_TEXT_ALIGNMENT;

#ifndef DEBUG_OLEDDISPLAYUI
#define DEBUG_OLEDDISPLAYUI(...)
#endif

typedef enum {
	SLIDE_UP,
	SLIDE_DOWN,
	SLIDE_LEFT,
	SLIDE_RIGHT
} AnimationDirection;

typedef enum {
	TOP,
	RIGHT,
	BOTTOM,
	LEFT
} IndicatorPosition;

typedef enum {
	LEFT_RIGHT,
	RIGHT_LEFT
} IndicatorDirection;

typedef enum {
	IN_TRANSITION,
	FIXED
} FrameState;

// Structure of the UiState
typedef struct {
	uint64_t     lastUpdate;
	uint16_t      ticksSinceLastStateSwitch;
	FrameState    frameState;
	uint8_t       currentFrame;
	bool          isIndicatorDrawen;
	// Normal = 1, Inverse = -1;
	int8_t        frameTransitionDirection;
	bool          manuelControll;
	// Custom data that can be used by the user
	void*         userData;
} OLEDDisplayUiState;

typedef struct {
	const char* process;
	void (*callback)();
} LoadingStage;

typedef void (*FrameCallback)(OLEDDisplayUiState* state, int16_t x, int16_t y);
typedef void (*OverlayCallback)(OLEDDisplayUiState* state);
typedef void (*LoadingDrawFunction)(LoadingStage* stage, uint8_t progress);




/**
 * @brief  Initializes Oled Display 
 * @param  None
 * @retval Initialization status:
 *           - 0: SH1106 was not detected on I2C port
 *           - >0: SH1106 initialized OK and ready to use
 */
uint8_t Display_Init(uint8_t i2c_addr);

/**
 * @brief  Updates buffer from internal RAM to LCD
 * @note   This function must be called each time you do some changes to LCD, to update buffer from RAM to LCD
 * @param  None
 * @retval None
 */
void UpdateScreen(void);

/* Sets the color of all pixel operations */
void oledSetColor(OLEDDISPLAY_COLOR color);

void oledDisplayClear(void);

/* Cycle through the initialization */
void oledResetDisplay(void);

/* Draw a pixel at given position */
void oledSetPixel(int16_t x, int16_t y);

/* Draw a line from position 0 to position 1 */
void oledDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);

/* Draw the border of a rectangle at the given location */
void oledDrawRect(int16_t x, int16_t y, int16_t width, int16_t height);

/* Fill the rectangle */
void oledFillRect(int16_t xMove, int16_t yMove, int16_t width, int16_t height);

/* Draw the border of a circle */
void oledDrawCircle(int16_t x0, int16_t y0, int16_t radius);

/* Draw all Quadrants specified in the quads bit mask */
void oledDrawCircleQuads(int16_t x0, int16_t y0, int16_t radius, uint8_t quads);

/* Fill circle */
void oledFillCircle(int16_t x0, int16_t y0, int16_t radius);

/* Draw a line horizontally */
void oledDrawHorizontalLine(int16_t x, int16_t y, int16_t length);

/* Draw a line vertically */
void oledDrawVerticalLine(int16_t x, int16_t y, int16_t length);

/*
 *  Draws a rounded progress bar with the outer dimensions given by
 *  width and height. Progress is
 * a unsigned byte value between 0 and 100
 */
void oledDrawProgressBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t progress);

/* Draw a bitmap in the internal image format */
void oledDrawFastImage(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const char *image);

/* Draw a XBM */
void oledDrawXbm(int16_t xMove, int16_t yMove, int16_t width, int16_t height, const char *xbm);

/* Draws a string at the given location */
void oledDrawString(int16_t xMove, int16_t yMove, char *strUser);


/*
 *Draws a String with a maximum width at the given location.
 * If the given String is wider than the specified width
 * The text will be wrapped to the next line at a space or dash
 */
void oledDrawStringMaxWidth(int16_t x, int16_t y, uint16_t maxLineWidth, char *text);

/*
 * Returns the width of the const char* with the current
 * font settings
 */
uint16_t oledGetStringWidth(const char* text, uint16_t length);


/*
 * Specifies relative to which anchor point
 * the text is rendered. Available constants:
 * TEXT_ALIGN_LEFT, TEXT_ALIGN_CENTER, TEXT_ALIGN_RIGHT, TEXT_ALIGN_CENTER_BOTH
 */
void oledSetTextAlignment(OLEDDISPLAY_TEXT_ALIGNMENT textAlignment);

/*
 * Sets the current font. Available default fonts
 * ArialMT_Plain_10, ArialMT_Plain_16, ArialMT_Plain_24
 */
void oledSetFont(const char *font);

/* Turn the display on */
void oledDisplayOn(void);

/* Turn the display offs */
void oledDisplayOff(void);

/* Inverted display mode */
void oledInvertDisplay(void);

/* Normal display mode */
void oledNormalDisplay(void);

/* Set display contrast */
void oledSetContrast(char contrast);

/* Turn the display upside down */
void oledFlipScreenVertically();


/* Draw the log buffer at position (x, y) */
void oledDrawLogBuffer(uint16_t xMove, uint16_t yMove);

/*
 * This will define the lines and characters you can
 * print to the screen. When you exeed the buffer size (lines * chars)
 * the output may be truncated due to the size constraint.
 */
bool oledSetLogBuffer(uint16_t lines, uint16_t chars);

size_t oledPutc(uint8_t c);
size_t oledWrite(const char* str);



/*	UI	*/
void setTargetFPS(uint8_t fps);

// Automatic controll

void enableAutoTransition();
void disableAutoTransition();
void setAutoTransitionForwards();
void setAutoTransitionBackwards();
void setTimePerFrame(uint16_t time);
void setTimePerTransition(uint16_t time);

// Customize indicator position and style
void enableIndicator();
void disableIndicator();
void enableAllIndicators();
void disableAllIndicators();
void setIndicatorPosition(IndicatorPosition pos);
void setIndicatorDirection(IndicatorDirection dir);
void setActiveSymbol(const char* symbol);
void setInactiveSymbol(const char* symbol);

// Frame settings
void setFrameAnimation(AnimationDirection dir);
void setFrames(FrameCallback* frameFunctions, uint8_t frameCount);

// Overlays
void setOverlays(OverlayCallback* overlayFunctions, uint8_t overlayCount);

// Loading Process
void setLoadingDrawFunction(LoadingDrawFunction loadingDrawFunction);
void UIrunLoadingProcess(LoadingStage* stages, uint8_t stagesCount);

// Manuel control
void nextFrame();
void previousFrame();
void switchToFrame(uint8_t frame);
void transitionToFrame(uint8_t frame);

// State information 
OLEDDisplayUiState* getUiState();


int8_t UIupdate();
void UItick();
void UIresetState();
void UIdrawFrame();
void UIdrawIndicator();
void UIdrawOverlays();
uint8_t UIgetNextFrameNumber();



#endif
