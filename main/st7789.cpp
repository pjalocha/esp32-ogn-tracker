#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"     // for SPI
#include "esp32/rom/tjpgd.h"       // for JPEG
#include "driver/ledc.h"           // for PWM backlight control

#include "st7789.h"

#include "hal.h"
#include "format.h"

// #define LCD_FLIP                   // flip the LCD: rotate by 180deg, works only for 240x240 displays

// =============================================================================

// pins specific for this LCD
gpio_num_t LCD_PIN_CS   = GPIO_NUM_NC;   // connected to ground: constantly active
gpio_num_t LCD_PIN_DC   = GPIO_NUM_0;    // D/C: Command = 0, Data = 1
gpio_num_t LCD_PIN_RST  = GPIO_NUM_NC;   // actually connected to system RESET
gpio_num_t LCD_PIN_BCKL = GPIO_NUM_4;    // back-light: HIGH active

#ifdef LCD_FLIP
const int LCD_XOFS   =  80;      // [pixels]
#endif

int LCD_TYPE   =   0;      // 0 = ST7789, 1 = ILI9341
int LCD_WIDTH  = 240;      // [pixels]
int LCD_HEIGHT = 240;      // [pixels]

static spi_device_handle_t LCD_SPI;

// =============================================================================

/*
typedef struct
{ uint8_t cmd;       // command
  uint8_t databytes; // Number of parameter bytes; bit 7 = delay after set; 0xFF = end of cmds.
  uint8_t data[14];  // up to 16 parameter bytes per command
} lcd_init_cmd_t;

static const lcd_init_cmd_t ST7789_init_cmds[] = {
    { 0x01, 0xC0, {0} },       // LCD software reset, wait 120ms
    { 0x36, 1, {0b01100000} }, // Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0
    { 0x3A, 1, {0x55} },       // Interface Pixel Format, 16bits/pixel for RGB/MCU interface
    { 0xB2, 5, {0x0c, 0x0c, 0x00, 0x33, 0x33} },   // Porch Setting
    { 0xB7, 1, {0x45} },       // Gate Control, Vgh=13.65V, Vgl=-10.43V
    { 0xBB, 1, {0x2B} },       // VCOM Setting, VCOM=1.175V
    { 0xC0, 1, {0x2C} },       // LCM Control, XOR: BGR, MX, MH
    { 0xC2, 2, {0x01, 0xff} }, // VDV and VRH Command Enable, enable=1
    { 0xC3, 1, {0x11} },       // VRH Set, Vap=4.4+...
    { 0xC4, 1, {0x20} },       // VDV Set, VDV=0
    { 0xC6, 1, {0x0f} },       // Frame Rate Control, 60Hz, inversion=0
    { 0xD0, 2, {0xA4, 0xA1} }, // Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V
                              // Positive Voltage Gamma Control
    { 0xE0, 14, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19} },
                              // Negative Voltage Gamma Control
    { 0xE1, 14, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19} },
    { 0x11, 0xC0, {0} },       // Sleep Out, wait 120ms
    { 0x21, 0, {0} },          // Invert ON
    { 0x29, 0x10, {0} },       // Display ON, wait 10ms ?
    { 0x00, 0xFF, {0} }
};
*/

static const uint8_t ST7789_init[] = {
//    0x00, 0x10,
    0x01, 0xE0,          // LCD software reset, wait 140ms
    0x01, 0xE0,          // 2nd LCD software reset, wait 140ms
#ifdef LCD_FLIP
    0x36, 0x01, 0b10100000, // flipped by 180deg but needs horizontal offset
#else
    0x36, 0x01, 0b01100000, // Memory Data Access Control, MX MY MV ML RGB MH 1 0
#endif
    0x3A, 0x01, 0x55,       // Interface Pixel Format, 16bits/pixel for RGB/MCU interface
    0xB2, 0x05, 0x0c, 0x0c, 0x00, 0x33, 0x33,   // Porch Setting
    0xB7, 0x01, 0x45,       // Gate Control, Vgh=13.65V, Vgl=-10.43V
    0xBB, 0x01, 0x2B,       // VCOM Setting, VCOM=1.175V
    0xC0, 0x01, 0x2C,       // LCM Control, XOR: BGR, MX, MH
    0xC2, 0x02, 0x01,0xff,  // VDV and VRH Command Enable, enable=1
    0xC3, 0x01, 0x11,       // VRH Set, Vap=4.4+...
    0xC4, 0x01, 0x20,       // VDV Set, VDV=0
    0xC6, 0x01, 0x0f,       // Frame Rate Control, 60Hz, inversion=0
    0xD0, 0x02, 0xA4, 0xA1, // Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V
    0xE0, 0x0E, 0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19, // Positive Voltage Gamma Control
    0xE1, 0x0E, 0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19, // Negative Voltage Gamma Control
    0x11, 0xE0,             // Sleep Out, wait 140ms
    0x21, 0x00,             // Invert ON
    0x29, 0x20,             // Display ON, wait 20ms ?
    0x00, 0xFF              // terminator
};

static const uint8_t ILI9341_init[] = {
    0x01, 0xD0,                       // Software RESET
    0xCF, 3, 0x00, 0x83, 0x30 ,       // Power contorl B, power control = 0, DC_ENA = 1
    0xED, 4, 0x64, 0x03, 0x12, 0x81 , // Power on sequence control, cp1 keeps 1 frame, 1st frame enable, vcl = 0, ddvdh=3, vgh=1, vgl=2, DDVDH_ENH=1
    0xE8, 3, 0x85, 0x01, 0x79 ,       // Driver timing control A, non-overlap=default +1, EQ=default - 1, CR=default, pre-charge=default - 1
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02 , // Power control A, Vcore=1.6V, DDVDH=5.6V
    0xF7, 1, 0x20 ,                   // Pump ratio control, DDVDH=2xVCl
    0xEA, 2, 0x00, 0x00 ,             // Driver timing control, all=0 unit
    // 0xC0, 1, 0x23,
    // 0xC1, 1, 0x10,
    // 0xC5, 2, 0x3E, 0x28,
    // 0xC7, 1, 0x86,
    0xC0, 1, 0x26,                    // Power control 1, GVDD=4.75V
    0xC1, 1, 0x11,                    // Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3
    0xC5, 2, 0x35, 0x3E ,             // VCOM control 1, VCOMH=4.025V, VCOML=-0.950V
    0xC7, 1, 0xBE,                    // VCOM control 2, VCOMH=VMH-2, VCOML=VML-2
    0x36, 1, 0b00001000 ,             // Memory Data Access Control, MX=MV=0, MY=ML=MH=0, RGB=1
    0x3A, 1, 0x55 ,                   // Pixel format, 16bits/pixel for RGB/MCU interface
    0xB1, 2, 0x00, 0x1B ,             // Frame rate control, f=fosc, 70Hz fps
    // 0xF2, 1, 0x08 ,                   // Enable 3G, disabled
    0xF2, 1, 0x00 ,                   // 3Gamma function disable
    0x26, 1, 0x01 ,                   // Gamma set, curve 1
    // 0x26, 1, 0x02 ,                   // Gamma set, curve 1
    // 0xE0, 15, 0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00 , // Positive gamma correction
    // 0xE1, 15, 0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F , // Negative gamma correction
    0xE0, 15, 0x0f, 0x31, 0x2b, 0x0c, 0x0e, 0x08, 0x4e, 0xf1, 0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00,
    0xE1, 15, 0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xc1, 0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f,
    // 0xE0, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
    // 0xE1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
    0x2A, 4, 0x00, 0x00, 0x00, 0xEF ,    // Column address set, SC=0, EC=0xEF
    0x2B, 4, 0x00, 0x00, 0x01, 0x3f ,    // Page address set, SP=0, EP=0x013F
    0x2C, 0,                             // Memory write
    0xB7, 1, 0x07 ,                      // Entry mode set, Low vol detect disabled, normal display
    0xB6, 4, 0x0A, 0x82, 0x27, 0x00 ,    // Display function control
    0x11, 0xC0,                          // Sleep out
    0x29, 0x20,                          // Display on
    0x00, 0xFF
};

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
static esp_err_t lcd_cmd(const uint8_t cmd)
{ spi_transaction_t t;
  memset(&t, 0, sizeof(t));       // Zero out the transaction
  t.length=8;                     // Command is 8 bits
  t.tx_buffer=&cmd;               // The data is the cmd itself
  t.user=(void*)0;                // D/C needs to be set to 0
  return spi_device_polling_transmit(LCD_SPI, &t); } // Transmit!

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */

static DRAM_ATTR uint8_t ram_buff[16];

static esp_err_t lcd_data(const uint8_t *data, int len)
{ // uint8_t ram_buff[len];
  memcpy(ram_buff, data, len);    // copy to RAM buffer, otherwise DMA would not work
  spi_transaction_t t;
  if (len==0) return ESP_OK;      // no need to send anything
  memset(&t, 0, sizeof(t));       // Zero out the transaction
  t.length=len*8;                 // Len is in bytes, transaction length is in bits.
  t.tx_buffer=ram_buff;           // Data
  t.user=(void*)1;                // D/C needs to be set to 1
  return spi_device_polling_transmit(LCD_SPI, &t); } // Transmit!

// This function is called (in irq context!) just before a transmission starts. It will
// set the D/C line to the value indicated in the user field.
static void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{ int DC=(int)t->user;
  gpio_set_level(LCD_PIN_DC, DC);
  // if(LCD_PIN_CS>GPIO_NUM_NC) gpio_set_level(LCD_PIN_CS, 0);
}

// static void lcd_spi_post_transfer_callback(spi_transaction_t *t)
// { if(LCD_PIN_CS>GPIO_NUM_NC) gpio_set_level(LCD_PIN_CS, 1); }

static ledc_timer_config_t LEDC_Timer =
{ speed_mode      : LEDC_HIGH_SPEED_MODE,   // timer mode
  duty_resolution : LEDC_TIMER_8_BIT,       // resolution of PWM duty: 0..255
  timer_num       : LEDC_TIMER_0,           // timer index
  freq_hz         : 1000                    // frequency of PWM signal
} ;

static ledc_channel_config_t LEDC_Channel =
  { gpio_num   : LCD_PIN_BCKL,
    speed_mode : LEDC_HIGH_SPEED_MODE,
    channel    : LEDC_CHANNEL_1,
    intr_type  : LEDC_INTR_DISABLE,
    timer_sel  : LEDC_TIMER_0,
    duty       : 0,
    hpoint     : 0
  } ;

void LCD_SetBacklightLevel(uint8_t Level)
{ if(Level>8) Level=0;
  uint8_t Duty=1; Duty<<=Level; Duty--;
  if(LCD_PIN_BCKL>GPIO_NUM_NC)
  { ledc_set_duty(LEDC_Channel.speed_mode, LEDC_Channel.channel, Duty);
    ledc_update_duty(LEDC_Channel.speed_mode, LEDC_Channel.channel); }
}

// Initialize the display
static void lcd_gpio_init(void)
{
  if(LCD_PIN_BCKL>GPIO_NUM_NC)
  { LEDC_Channel.gpio_num = LCD_PIN_BCKL;
    ledc_timer_config(&LEDC_Timer);            // Set configuration of timer for high speed channels
    ledc_channel_config(&LEDC_Channel);
    LCD_SetBacklightOFF(); }

  gpio_set_direction(LCD_PIN_DC, GPIO_MODE_OUTPUT);

  if(LCD_PIN_RST>GPIO_NUM_NC)
  { gpio_set_direction(LCD_PIN_RST, GPIO_MODE_OUTPUT); }

  // if(LCD_PIN_CS>GPIO_NUM_NC)
  // { gpio_set_direction(LCD_PIN_CS, GPIO_MODE_OUTPUT);
  //   gpio_set_level(LCD_PIN_CS, 1); }
}

static void lcd_start(const uint8_t *cmd) // reset controller and send initial config commands
{
  if(LCD_PIN_RST>GPIO_NUM_NC)
  { gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(100);
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(140);
    cmd+=2; }                         // skip Software Reset
  else
  { }                                 // include Software Reset

  for( ; ; )
  { uint8_t Cmd = *cmd++;
    uint8_t Len = *cmd++; if(Len==0xFF) break;
    uint8_t Wait = Len>>4; Len&=0x0F;
    lcd_cmd(Cmd);
    lcd_data(cmd, Len);
    if(Wait) vTaskDelay(10*Wait);
    //     else vTaskDelay(1);
    cmd+=Len; }

}

// =============================================================================

// const int LCD_BUFF_SIZE = 12*320;
DRAM_ATTR static uint16_t lcd_buffer[LCD_BUFF_SIZE];
static int      lcd_buffer_filled = 0;                 // buffer is prefilled up to this size with a fixed RGB565

static void lcd_buffer_fill(int size, uint16_t RGB565) // fill the buffer with given RGB565 up to the desired size
{ if(lcd_buffer[0]!=RGB565) lcd_buffer_filled=0;       // if filled with a different RGB565 then assume not filled
  if(lcd_buffer_filled>=size) return;                  // if filled up to the desired size then we are done
  for(int x=lcd_buffer_filled; x<size; x++)            // fill up to the desired size
  { lcd_buffer[x]=RGB565; }
  lcd_buffer_filled=size; }                            // mark as filled till the requested size

static spi_transaction_t lcd_trans[6];                 // six SPI transactions to transfer pixel data
static bool lcd_transaction_active = 0;                // initially not active

static void lcd_trans_init(void)                       // initial setup of the transactions
{ for (int x=0; x<6; x++)
  { memset(&lcd_trans[x], 0, sizeof(spi_transaction_t));
    if ((x&1)==0)                                      // Even transfers are commands
    { lcd_trans[x].length = 8;                         // 8-bit = single byte command
      lcd_trans[x].user=(void*)0; }                    // D/C = LOW for command
    else                                               // Odd transfers are data
    { lcd_trans[x].length = 8*4;                       // 4 byte = arguments (except for the last transaction)
      lcd_trans[x].user=(void*)1; }                    // D/C = HIGH for data
    lcd_trans[x].flags=SPI_TRANS_USE_TXDATA; }
  lcd_transaction_active=0; }

static void lcd_trans_wait(void)                       // wait for the six SPI transaction to complete
{ if(!lcd_transaction_active) return;
  spi_transaction_t *rtrans;
  for (int x=0; x<6; x++)
  { esp_err_t ret=spi_device_get_trans_result(LCD_SPI, &rtrans, portMAX_DELAY); }
  lcd_transaction_active=0; }

static void lcd_trans_start(void)                      // start the six SPI transactions to write data to given X/Y
{ if(lcd_transaction_active) lcd_trans_wait();         // if previous transaction set not finished, then wait for them
  for (int x=0; x<6; x++)                              // start the six SPI transactions
  { esp_err_t ret=spi_device_queue_trans(LCD_SPI, &lcd_trans[x], portMAX_DELAY); }
  lcd_transaction_active=1; }                          // mark the transactions as beig active

// positions here must fit into the screen, there is no check and no correction if they don't fit
static void lcd_trans_setup(int xpos, int ypos, int xsize, int ysize, uint16_t *data)
{
#ifdef LCD_FLIP
  xpos+=LCD_XOFS;
#endif
  if(lcd_transaction_active) lcd_trans_wait();         // if previous transaction set not finished, then wait for them
  lcd_trans[0].tx_data[0] = 0x2A;                      // Column Address Set
  lcd_trans[1].tx_data[0] = xpos>>8;                   // Start Col MSB
  lcd_trans[1].tx_data[1] = xpos&0xFF;                 // Start Col LSB
  xpos += xsize-1;
  lcd_trans[1].tx_data[2] = xpos>>8;                   // End Col MSB
  lcd_trans[1].tx_data[3] = xpos&0xFF;                 // End Col LSB
  lcd_trans[2].tx_data[0] = 0x2B;                      // Page address set
  lcd_trans[3].tx_data[0] = ypos>>8;                   // Start page MSB
  lcd_trans[3].tx_data[1] = ypos&0xFF;                 // start page LSB
  ypos += ysize-1;
  lcd_trans[3].tx_data[2] = ypos>>8;                   // end page MSB
  lcd_trans[3].tx_data[3] = ypos&0xFF;                 // end page LSB
  lcd_trans[4].tx_data[0] = 0x2C;                      // memory write
  lcd_trans[5].tx_buffer = data;                       // finally send the line data
  lcd_trans[5].length = xsize*ysize*2*8;               // Data length, in bits
  lcd_trans[5].rxlength = 0;                           // need to set this, otherwise a previous value is taken and an error occurs
  lcd_trans[5].flags=0; }                              // undo SPI_TRANS_USE_TXDATA flag

// ================================================================================

void LCD_DrawBox(int xpos, int ypos, int xsize, int ysize, uint16_t RGB565)
{ if(xsize==0) return;
  if(ysize==0) return;
  if(xpos>=LCD_WIDTH) return;
  if(ypos>=LCD_HEIGHT) return;
  if((xpos+xsize)<=0) return;
  if((ypos+ysize)<=0) return;
  if(xpos<0) { xsize+=xpos; xpos=0; }
  if(ypos<0) { ysize+=ypos; ypos=0; }
  if((xpos+xsize)>LCD_WIDTH)  { xsize=LCD_WIDTH-xpos; }
  if((ypos+ysize)>LCD_HEIGHT) { ysize=LCD_HEIGHT-ypos; }

  if(lcd_transaction_active) lcd_trans_wait();

  int lines_per_batch = LCD_BUFF_SIZE/xsize;             // number of lines we can do per batch given the buffer size
  if(lines_per_batch>ysize) lines_per_batch=ysize;       // if bigger than we need to do then cut it down
  int pixels_per_batch = lines_per_batch*xsize;          // number of pixels per batch
  lcd_buffer_fill(pixels_per_batch, RGB565);             // fill the buffer with uniform color, if not filled already
  for( ; ysize; )                                        // send given number of lines: do it in batches for speed
  { if(lines_per_batch>ysize) lines_per_batch=ysize;
    lcd_trans_setup(xpos, ypos, xsize, lines_per_batch, lcd_buffer);
    lcd_trans_start();
    ypos+=lines_per_batch; ysize-=lines_per_batch; }
}

// void LCD_DrawHorLine(int xpos, int ypos, int xsize, uint16_t RGB565)
// { LCD_DrawBox(xpos, ypos, xsize, 1, RGB565); }

// void LCD_DrawVerLine(int xpos, int ypos, int ysize, uint16_t RGB565)
// { LCD_DrawBox(xpos, ypos, 1, ysize, RGB565); }

// void LCD_DrawPixel(int xpos, int ypos, uint16_t RGB565)
// { LCD_DrawBox(xpos, ypos, 1, 1, RGB565); }

// ================================================================================

static void swap(int &a, int &b) { int t = a; a = b; b = t; }

void LCD_DrawLine(int x0, int y0, int x1, int y1, uint16_t RGB565)
{
  if (x0 == x1)
  { if (y0 <= y1) LCD_DrawVerLine(x0, y0, y1-y0+1, RGB565);
             else LCD_DrawVerLine(x0, y1, y0-y1+1, RGB565);
    return; }
  if (y0 == y1)
  { if (x0 <= x1) LCD_DrawHorLine(x0, y0, x1-x0+1, RGB565);
             else LCD_DrawHorLine(x1, y0, x0-x1+1, RGB565);
    return; }

  int steep = 0;
  if (abs(y1-y0) > abs(x1-x0)) steep = 1;
  if (steep)   { swap(x0, y0); swap(x1, y1); }
  if (x0>x1)   { swap(x0, x1); swap(y0, y1); }

  int dx = x1 - x0;
  int dy = abs(y1 - y0);
  int err = dx >> 1;
  int ystep = -1;
  int xs = x0;
  int dlen = 0;

  if (y0<y1) ystep = 1;

  if (steep)
  { for ( ; x0<=x1; x0++)
    { dlen++;
      err-=dy;
      if(err<0)
      { err+=dx;
        if (dlen==1) LCD_DrawPixel(y0, xs, RGB565);
                else LCD_DrawVerLine(y0, xs, dlen, RGB565);
        dlen=0; y0+=ystep; xs=x0+1;
      }
    }
    if (dlen) LCD_DrawVerLine(y0, xs, dlen, RGB565);
  }
  else
  { for ( ; x0<=x1; x0++)
    { dlen++;
      err-=dy;
      if(err<0)
      { err+=dx;
        if (dlen==1) LCD_DrawPixel(xs, y0, RGB565);
                else LCD_DrawHorLine(xs, y0, dlen, RGB565);
        dlen=0; y0+=ystep; xs=x0+1;
      }
    }
    if (dlen) LCD_DrawHorLine(xs, y0, dlen, RGB565);
  }

}

void LCD_DrawCircle(int x, int y, int radius, uint16_t RGB565)
{ int f = 1 - radius;
  int ddF_x = 1;
  int ddF_y = -2 * radius;
  int x1 = 0;
  int y1 = radius;

  LCD_DrawPixel(x, y + radius, RGB565);
  LCD_DrawPixel(x, y - radius, RGB565);
  LCD_DrawPixel(x + radius, y, RGB565);
  LCD_DrawPixel(x - radius, y, RGB565);

  while(x1<y1)
  { if (f >= 0)
    { y1--;
      ddF_y += 2;
      f += ddF_y; }
    x1++;
    ddF_x += 2;
    f += ddF_x;
    LCD_DrawPixel(x + x1, y + y1, RGB565);
    LCD_DrawPixel(x - x1, y + y1, RGB565);
    LCD_DrawPixel(x + x1, y - y1, RGB565);
    LCD_DrawPixel(x - x1, y - y1, RGB565);
    LCD_DrawPixel(x + y1, y + x1, RGB565);
    LCD_DrawPixel(x - y1, y + x1, RGB565);
    LCD_DrawPixel(x + y1, y - x1, RGB565);
    LCD_DrawPixel(x - y1, y - x1, RGB565);
  }
}

// ================================================================================
// Proportional fonts for Arduino

class propChar
{ public:
   uint8_t charCode;     // for example 32 for ' '
   uint8_t yOffset;      // empty horizontal space on top
   uint8_t width;        // core width
   uint8_t height;       // core height
   uint8_t xOffset;      // empty vertical space on the left
   uint8_t xDelta;       // shift X-pos for the next character
   uint8_t Data[];       //
  public:
   uint16_t CoreBits(void) const { return (uint16_t)width*height; }
   uint8_t DataBytes(void) const { return (CoreBits()+7)/8; }
} ;

int LCD_FontHeight(const uint8_t *propFont) { return propFont[1]; } // height of the given font is in the 2nd byte of the font header

static const propChar *FindChar(char Char, const uint8_t *propFont) // find given character
{ propFont+=4;                                                // skip the font header: 4 bytes
  for( ; ; )                                                  // loop over the characters
  { const propChar *Geom = (const propChar *)propFont;        // pointer to the next character geometry
    if(Geom->charCode==0xFF) break;                           // 0xFF is terminator: no more characters
    if(Geom->charCode==(uint8_t)Char) return Geom;            // if code matches: return the pointer
    propFont += 6 + Geom->DataBytes(); }                      // if not: skip this character and go to the next
  return 0; }                                                 // if went through all and not found: return NULL

// slow, because goes pixel-by-pixel, but simple to code, no big deal with out-of-screen positions
int LCD_DrawTranspChar(char Char, int xpos, int ypos, uint16_t RGB565, const uint8_t *propFont)
{ const propChar *Geom = FindChar(Char, propFont); if(Geom==0) return 0;
  xpos += Geom->xOffset;
  ypos += Geom->yOffset;
  const uint8_t *Data = Geom->Data;
  uint8_t Byte = 0x00;
  uint8_t Mask = 0x00;
  for(int dy=0; dy < Geom->height; dy++)
  { for(int dx=0; dx < Geom->width; dx++)
    { if(Mask==0) { Byte = *Data++; Mask=0x80; }
      if(Byte&Mask) LCD_DrawPixel(xpos+dx, ypos+dy, RGB565);
      Mask>>=1; }
  }
  return Geom->xDelta; }                               // return by how much move the cursor to draw the next character

int LCD_DrawTranspString(const char *String, int xpos, int ypos, uint16_t RGB565, const uint8_t *propFont)
{ int Len = 0;
  for( ; ; )
  { char Char = *String++; if(Char==0) break;
    Len += LCD_DrawTranspChar(Char, xpos+Len, ypos, RGB565, propFont); }
  return Len; }

int LCD_DrawChar(char Char, int xpos, int ypos, uint16_t Fore, uint16_t Back, const uint8_t *propFont)
{ const propChar *Geom = FindChar(Char, propFont); if(Geom==0) return 0;
  int Width = Geom->xDelta;                 // the box for the character to be printed
  int Height = LCD_FontHeight(propFont);
  int CropLeft = 0; if(xpos<0) { CropLeft=(-xpos); xpos=0; Width -=CropLeft; }
  int CropTop  = 0; if(ypos<0) { CropTop =(-ypos); ypos=0; Height-=CropTop ; }
  int CropRight = 0; if((xpos+Width)>LCD_WIDTH) { CropRight=xpos-LCD_WIDTH; Width-=CropRight; }
  int CropBottom = 0; if((ypos+Height)>LCD_HEIGHT) { CropBottom=ypos-LCD_HEIGHT; Height-=CropBottom; }
  if(Width<=0 || Height<=0) return Geom->xDelta;
  int Pixels = Width*Height;
  lcd_buffer_filled=0;
  for(int Pix=0; Pix<Pixels; Pix++) lcd_buffer[Pix]=Back;
  const uint8_t *Data = Geom->Data;
  uint8_t Byte = 0x00;
  uint8_t Mask = 0x00;
  for(int dy=0; dy < Geom->height; dy++)
  { for(int dx=0; dx < Geom->width; dx++)
    { if(Mask==0) { Byte = *Data++; Mask=0x80; }
      if(Byte&Mask)
      { int X = Geom->xOffset+dx-CropLeft;
        int Y = Geom->yOffset+dy-CropTop;
        if( X>=0 && X<Width && Y>=0 && Y<Height)
        { lcd_buffer[Y*Width+X] = Fore; }
      }
      Mask>>=1; }
  }
  lcd_trans_setup(xpos, ypos, Width, Height, lcd_buffer);
  lcd_trans_start();
  lcd_trans_wait();
  return Width; }

int LCD_CharWidth(char Char, const uint8_t *propFont)
{ const propChar *Geom = FindChar(Char, propFont); if(Geom==0) return 0;
  return Geom->xDelta; }

int LCD_StringWidth(const char *String, const uint8_t *propFont)
{ int Len = 0;
  for( ; ; )
  { char Char = *String++; if(Char==0) break;
    Len += LCD_CharWidth(Char, propFont); }
  return Len; }

// draw opaque characters of a string
int LCD_DrawString(const char *String, int xpos, int ypos, uint16_t Fore, uint16_t Back, const uint8_t *propFont)
{ int Len = 0;
  for( ; ; )
  { char Char = *String++; if(Char==0) break;
    Len += LCD_DrawChar(Char, xpos+Len, ypos, Fore, Back, propFont); }
  return Len; }

// draw only those characters of a string which have changed to minimize the LCD transfer thus maximize the speed
int LCD_UpdateString(const char *String, const char *RefString, int xpos, int ypos, uint16_t Fore, uint16_t Back, const uint8_t *propFont)
{ int Len = 0;
  for( ; ; )
  { char Char = *String; if(Char) String++;
    char RefChar = *RefString; if(RefChar) RefString++;
    if(Char==0 && RefChar==0) break;
    if(Char==RefChar)
    { Len += LCD_CharWidth(Char, propFont); }
    else
    { if(Char) { Len += LCD_DrawChar(Char, xpos+Len, ypos, Fore, Back, propFont); }
          else { int Width = LCD_CharWidth(RefChar, propFont);
                 LCD_DrawBox(xpos+Len, ypos, Width, LCD_FontHeight(propFont), Back);
                 Len+=LCD_CharWidth(RefChar, propFont); }
    }
  }
  return Len; }

// =============================================================================

typedef struct
{ int16_t         xpos;              // top left point X position on the LCD
  int16_t         ypos;              // top left point Y position on the LCD
  const uint8_t * Input;             // memory buffer containing the image
  uint32_t        InpSize;           // size of the input image
  uint32_t        InpPtr;            // input image current position
} JPGIODEV;

static UINT tjd_buf_input (
        JDEC* jd,               // Decompression object
        BYTE* buff,             // Pointer to the read buffer (NULL:skip)
        UINT nd )               // Number of bytes to read/skip from input stream
{ JPGIODEV *dev = (JPGIODEV*)jd->device;
  // CONS_UART_Write('i');
  // Format_Hex(CONS_UART_Write, (uint32_t)(dev->Input));
  // CONS_UART_Write(':');
  // Format_Hex(CONS_UART_Write, (uint32_t)(buff));
  // CONS_UART_Write(' ');
  // return 0;
  if(!dev->Input) return 0;                       // if Input pointer is NULL (can it be ?)
  if(dev->InpPtr >= dev->InpSize) return 0;       // if past the InpSize: end of stream
  if( (dev->InpPtr+nd) > dev->InpSize) nd = dev->InpSize-dev->InpPtr;
  if(buff)                                        // if not NULL pointer then copy, otherwise just skip
  { memcpy(buff, dev->Input + dev->InpPtr, nd); } // copy the nd bytes of the input stream
  dev->InpPtr += nd;                              // incremeant the input pointer
  return nd; }                                    // return the number of bytes copied or skipped

static UINT tjd_output (
        JDEC* jd,       // Decompression object of current session
        void* bitmap,   // Bitmap data to be output
        JRECT* rect     // Rectangular region to output
)
{ // Device identifier for the session (5th argument of jd_prepare function)
  JPGIODEV *dev = (JPGIODEV*)jd->device;
  // CONS_UART_Write('o');

  int Lx0 = rect->left     + dev->xpos;  // coordinates on the screen
  int Ly0 = rect->top      + dev->ypos;
  int Lx1 = rect->right+1  + dev->xpos;  //
  int Ly1 = rect->bottom+1 + dev->ypos;
  int Jxs = Lx1-Lx0;                     // X-size of the JPEG rectagle
  int Jys = Ly1-Ly0;                     // Y-size of the JPEG rectagle

  int CropLeft   = 0; if(Lx0<0) { CropLeft = (-Lx0); Lx0=0; }             // how much crop the JPEG rectangle on the left
  int CropTop    = 0; if(Ly0<0) { CropTop  = (-Ly0); Ly0=0; }             // how much crop the JPEG rectangle on the right
  int CropRight  = 0; if(Lx1>LCD_WIDTH)  { CropRight = LCD_WIDTH-Lx1; Lx1=LCD_WIDTH; }    //
  int CropBottom = 0; if(Ly1>LCD_HEIGHT) { CropBottom = LCD_HEIGHT-Ly1; Ly1=LCD_HEIGHT; } //
  int Llines = Ly1-Ly0;                                                   // number of lines going to LCD
  int Lrows  = Lx1-Lx0;                                                   // number of pixels per line going to LCD
  if(Llines<=0) return 1;
  if(Lrows<=0) return 1;

  uint16_t *Dst = lcd_buffer;                // buffer to form RGB565 for transfer
  uint8_t *Src = (uint8_t *)bitmap;          // RGB from JPEG decoder
  if(CropTop) Src += CropTop*3*Jxs;          // Advance by the nuber of lines to skip
  for(int Line=0; Line<Llines; Line++)       // Loop over line to display
  { uint8_t *Ptr = Src + 3*CropLeft;         // advance by the pixels to skip on the left
    for(int Row=0; Row<Lrows; Row++)         // loop over pixel in this line
    { Dst[Row] = RGB565(Ptr); Ptr+=3; }      // convert to RGB565 and store in the buffer
    Src+=3*Jxs; Dst+=Lrows; }                // advance by the number of pixels
  lcd_buffer_filled=0;

  lcd_trans_setup(Lx0, Ly0, Llines, Lrows, lcd_buffer);
  lcd_trans_start();

  return 1; } // continue with decompression

void LCD_DrawJPEG(const uint8_t *JPEG, int JPEGsize, int xpos, int ypos, int Scale)
{ if(Scale<0) Scale=0;
  else if(Scale>3) Scale=3;
  JPGIODEV dev;                          // input/output device
  JDEC decoder;                          // Decompression object (70 bytes)
  // CONS_UART_Write('J');
  const UINT WorkSize = 3800;
  char *Work = (char *)malloc(WorkSize); // JPEG decode work space
  if(Work==0) return;
  // CONS_UART_Write('P');

  dev.Input = JPEG;
  dev.InpSize = JPEGsize;
  dev.InpPtr = 0;
  dev.xpos = xpos;
  dev.ypos = ypos;

  if(jd_prepare(&decoder, tjd_buf_input, (void *)Work, WorkSize, &dev)!=JDR_OK) { free(Work); return ; }
  // CONS_UART_Write('E');
  JRESULT rc = jd_decomp(&decoder, tjd_output, Scale);
  // CONS_UART_Write('G');
  free(Work); }

// ================================================================================

void LCD_Start(void)
{ // if(LCD_TYPE==1) lcd_start(ILI9341_init);                          // reset, send initial commands
  //            else lcd_start(ST7789_init);                           // reset, send initial commands
  if(LCD_TYPE==1) lcd_start(ILI9341_init);                          // reset, send initial commands
             else lcd_start(ST7789_init);                           // reset, send initial commands
  lcd_trans_init();                                                 // initialize SPI transactions
  LCD_DrawBox(0,  0, LCD_WIDTH, LCD_HEIGHT, RGB565_WHITE);          // screen all-white
}

void LCD_Init(spi_host_device_t LCD_SPI_HOST, uint8_t LCD_SPI_MODE, int LCD_SPI_SPEED) // Initialize SPI and LCD
{
  spi_device_interface_config_t DevConfig =  // specific device on the SPI bus
  {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = LCD_SPI_MODE,                     // SPI mode 3, but cna be 0 for other displays
    .duty_cycle_pos = 0,
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = LCD_SPI_SPEED,
    .input_delay_ns = 0,                      // seems to help with the reliability
    .spics_io_num = LCD_PIN_CS,               // CS pin
    .flags = 0,
    .queue_size = 8,                          // We want to be able to queue 7 transactions at a time
    .pre_cb = lcd_spi_pre_transfer_callback,  // Specify pre-transfer callback to handle D/C line
    .post_cb = 0 // lcd_spi_post_transfer_callback
  };

  esp_err_t ret = spi_bus_add_device(LCD_SPI_HOST, &DevConfig, &LCD_SPI);     // Attach the LCD to the SPI bus

  lcd_gpio_init();                                                  // setup GPIO

  // LCD_Start();

  LCD_SetBacklightLevel(8);                                         // max. backlight to signal power-on

//  LCD_DrawJPEG(OGN_logo_jpg, OGN_logo_size, 0, 0);                  // draw logo
//  // LCD_DrawJPEG(Club_logo_jpg, Club_logo_size, 0, 0);
/*
  LCD_DrawBox(LCD_SPI, 0,  0, LCD_WIDTH, LCD_HEIGHT, RGB565_RED);
  LCD_DrawBox(LCD_SPI, 32, 32, LCD_WIDTH-64, LCD_HEIGHT-64, RGB565_BLUE);
  LCD_DrawBox(LCD_SPI, 64, 64, LCD_WIDTH-128, LCD_HEIGHT-128, RGB565_GREEN);
*/
/*
  LCD_DrawLine(   0, 240, 240,   0, RGB565_MAGENTA);
  LCD_DrawLine(   0, 180, 240,  60, RGB565_MAGENTA);
  LCD_DrawLine(   0, 120, 240, 120, RGB565_MAGENTA);
  LCD_DrawLine(   0,  60, 240, 180, RGB565_MAGENTA);
  LCD_DrawLine(   0,   0, 240, 240, RGB565_MAGENTA);
  LCD_DrawLine( 180,   0,  60, 240, RGB565_MAGENTA);
  LCD_DrawLine( 120,   0, 120, 240, RGB565_MAGENTA);
  LCD_DrawLine(  60,   0, 180, 240, RGB565_MAGENTA);

  LCD_DrawCircle(120, 120,  25, RGB565_MAGENTA);
  LCD_DrawCircle(120, 120,  50, RGB565_MAGENTA);
  LCD_DrawCircle(120, 120,  75, RGB565_MAGENTA);
  LCD_DrawCircle(120, 120, 100, RGB565_MAGENTA);
*/
  // LCD_DrawString("OGN-Tracker",  40,  210, RGB565_BLUE, RGB565_PINK, tft_Dejavu24);
  // LCD_DrawTranspString("OGN-Tracker",  40,  210, RGB565_BLUE, tft_Dejavu24);
  // LCD_DrawTranspString("with T-Beam",  4, 20, RGB565_BLACK, tft_minya24);
  // LCD_DrawString("Test Test Test Test Test Test Test Test Test",  -10, -5, RGB565_BLUE, RGB565_PINK, tft_Dejavu24);

  // LCD_clearDisplay(RGB565_GREEN);
}

void LCD_ClearDisplay(uint16_t RGB565)
{ LCD_DrawBox(0,  0, LCD_WIDTH, LCD_HEIGHT, RGB565); }

uint16_t RGB565(uint8_t Red, uint8_t Green, uint8_t Blue) // convert 8/8/8-bit RGB to 5/6/5-bit RGB
{ uint16_t RGB = (Red>>3);
  RGB = (RGB<<5) | (Green>>2);
  RGB = (RGB<<6) | (Blue>>3);
  return (RGB>>8) | (RGB<<8); }

// uint16_t RGB565(uint8_t *RGB)
// { return RGB565(RGB[0], RGB[1], RGB[2]); }
