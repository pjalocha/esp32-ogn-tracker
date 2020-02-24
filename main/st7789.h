#ifndef __ST7789_H__
#define __ST7789_H__

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

// some predefine colors                           Red Green Blue
const uint16_t RGB565_BLACK       = 0x0000;      //   0,   0,   0
const uint16_t RGB565_LIGHTGREY   = 0x18C6;      // 192, 192, 192
const uint16_t RGB565_DARKGREY    = 0xEF7B;      // 128, 128, 128
const uint16_t RGB565_WHITE       = 0xFFFF;      // 255, 255, 255

const uint16_t RGB565_NAVY        = 0x0F00;      //   0,   0, 128
const uint16_t RGB565_BLUE        = 0x1F00;      //   0,   0, 255
const uint16_t RGB565_LIGHTBLUE   = 0xFF7B;      // 127, 127, 255

// const uint16_t RGB565_DARKGREEN   = 0x0004;      //   0, 128,   0
const uint16_t RGB565_DARKGREEN   = 0x0006;      //   0, 192,   0
const uint16_t RGB565_GREEN       = 0xE007;      //   0, 255,   0

const uint16_t RGB565_DARKCYAN    = 0xEF03;      //   0, 128, 128
const uint16_t RGB565_CYAN        = 0xFF07;      //   0, 255, 255

const uint16_t RGB565_MAROON      = 0x0078;      // 128,   0,   0
const uint16_t RGB565_RED         = 0x00F8;      // 255,   0,   0
const uint16_t RGB565_LIGHTRED    = 0xEFFB;      // 255, 127, 127

const uint16_t RGB565_PURPLE      = 0x0F78;      // 128,   0, 128
const uint16_t RGB565_MAGENTA     = 0x1FF8;      // 255,   0, 255

const uint16_t RGB565_OLIVE       = 0xE07B;      // 128, 128,   0 ?

// const uint16_t RGB565_DARKYELLOW  = 0x0084; // 128, 128, 0
const uint16_t RGB565_DARKYELLOW  = 0x00C6; // 192, 192, 0
const uint16_t RGB565_YELLOW      = 0xE0FF; // 255, 255, 0
const uint16_t RGB565_ORANGE      = 0xA0FD; // 255, 180, 0
const uint16_t RGB565_DARKORANGE  = 0xC082; // 128, 90, 0
const uint16_t RGB565_GREENYELLOW = 0xE0B7; // 180, 255, 0
const uint16_t RGB565_DARKGREENYELLOW = 0x005C; //  90, 128, 0

const uint16_t RGB565_PINK        = 0x9FFC;

// python formula: "%04X" % ( ((Red>>3)<<11) | ((Green>>2)<<5) | (Blue>>3) )
// and then swap the MSB and LSB bytes

uint16_t RGB565(uint8_t Red, uint8_t Green, uint8_t Blue); // create RGB565 from RGB888
inline uint16_t RGB565(const uint8_t *RGB) { return RGB565(RGB[0], RGB[1], RGB[2]); }

// Embedded fonts
extern uint8_t tft_SmallFont[];
extern uint8_t tft_DefaultFont[];
extern uint8_t tft_Dejavu18[];
extern uint8_t tft_Dejavu24[];
extern uint8_t tft_Ubuntu16[];
extern uint8_t tft_Comic24[];
extern uint8_t tft_minya24[];
extern uint8_t tft_tooney32[];
extern uint8_t tft_def_small[];

// pins specific for this LCD
extern gpio_num_t LCD_PIN_CS;      // connected to ground: constantly active
extern gpio_num_t LCD_PIN_DC;      // D/C: Command = 0, Data = 1
extern gpio_num_t LCD_PIN_RST;     // actually connected to system RESET
extern gpio_num_t LCD_PIN_BCKL;    // back-light: HIGH active

extern int LCD_TYPE;        // 0 = ST7789
extern int LCD_WIDTH;       // [pixels]
extern int LCD_HEIGHT;      // [pixels]

const int LCD_BUFF_SIZE = 12*320;

void LCD_Init(spi_host_device_t LCD_SPI_HOST, uint8_t LCD_SPI_MODE, int LCD_SPI_SPEED=10000000 /*, int LCD_TYPE=0 */ );
void LCD_Start(void);
void LCD_ClearDisplay(uint16_t RGB565);

void LCD_SetBacklightLevel(uint8_t Level);
inline void LCD_SetBacklightON(void) { LCD_SetBacklightLevel(8); }
inline void LCD_SetBacklightOFF(void) { LCD_SetBacklightLevel(0); }

void LCD_DrawBox(int xpos, int ypos, int xsize, int ysize, uint16_t RGB565);

void LCD_DrawLine(int x0, int y0, int x1, int y1, uint16_t RGB565);
inline void LCD_DrawPixel(int xpos, int ypos, uint16_t RGB565) { LCD_DrawBox(xpos, ypos, 1, 1, RGB565); }
inline void LCD_DrawHorLine(int xpos, int ypos, int xsize, uint16_t RGB565) { LCD_DrawBox(xpos, ypos, xsize, 1, RGB565); }
inline void LCD_DrawVerLine(int xpos, int ypos, int ysize, uint16_t RGB565) { LCD_DrawBox(xpos, ypos, 1, ysize, RGB565); }

void LCD_DrawCircle(int x, int y, int radius, uint16_t RGB565);

void LCD_DrawJPEG(const uint8_t *JPEG, int JPEGsize, int xpos=0, int ypos=0, int Scale=0);

int LCD_DrawTranspChar(char Char, int xpos, int ypos, uint16_t RGB565, const uint8_t *propFont = tft_Dejavu24);
int LCD_DrawTranspString(const char *String, int xpos, int ypos, uint16_t RGB565, const uint8_t *propFont = tft_Dejavu24);
int LCD_DrawChar(char Char, int xpos, int ypos, uint16_t Fore, uint16_t Back, const uint8_t *propFont = tft_Dejavu24);
int LCD_DrawString(const char *String, int xpos, int ypos, uint16_t Fore, uint16_t Back, const uint8_t *propFont = tft_Dejavu24);
int LCD_UpdateString(const char *String, const char *RefString, int xpos, int ypos, uint16_t Fore, uint16_t Back, const uint8_t *propFont = tft_Dejavu24);
int LCD_CharWidth(char Char, const uint8_t *propFont = tft_Dejavu24);
int LCD_StringWidth(const char *String, const uint8_t *propFont = tft_Dejavu24);
int LCD_FontHeight(const uint8_t *propFont = tft_Dejavu24);

#endif // __ST7789_H__
