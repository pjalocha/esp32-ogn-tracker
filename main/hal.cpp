#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "hal.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "esp_system.h"
#include "esp_freertos_hooks.h"

#ifdef WITH_SLEEP
#include "esp_sleep.h"
#endif

#include "nvs.h"
#include "nvs_flash.h"

#ifdef WITH_SPIFFS
#include "esp_spiffs.h"
#endif

#ifdef WITH_SD
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#endif

#ifdef WITH_BT_SPP
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "fifo.h"
#endif

#ifdef WITH_BEEPER
#include "driver/ledc.h"
#include "fifo.h"
#endif

#ifdef WITH_SOUND
#include "intmath.h"
#include "driver/i2s.h"
#endif

#ifdef WITH_OLED
#include "ssd1306.h"
#include "font8x8_basic.h"
#endif

#ifdef WITH_U8G2_OLED
#include "u8g2.h"
#endif

#ifdef WITH_TFT_LCD
extern "C" {
#include "tftspi.h"
#include "tft.h"
           }
#endif

#if defined(WITH_ST7789) || defined(WITH_ILI9341)
#include "st7789.h"
#endif

#ifdef WITH_AXP
#include "axp192.h"
#endif

#ifdef WITH_BQ
#include "bq24295.h"
#endif

uint8_t PowerMode = 2;                    // 0=sleep/minimal power, 1=comprimize, 2=full power

// ======================================================================================================
/*
The HELTEC AUtomation board WiFi LoRa 32 with sx1278 (RFM95)

Referenced:   http://esp32.net/
Pinout:       http://esp32.net/images/Heltec/WIFI-LoRa-32/Heltec_WIFI-LoRa-32_DiagramPinoutFromTop.jpg
              http://esp32.net/images/Heltec/WIFI-LoRa-32/Heltec_WIFI-LoRa-32_DiagramPinoutFromBottom.jpg
Arduino code: https://robotzero.one/heltec-wifi-lora-32/
ESP32 API:    https://esp-idf.readthedocs.io/en/latest/api-reference/index.html
UART example: https://github.com/espressif/esp-idf/blob/f4009b94dca9d17b909e1094d6e3d7dbb75d52c0/examples/peripherals/uart_echo
SPI example:  https://github.com/espressif/esp-idf/tree/f4009b94dca9d17b909e1094d6e3d7dbb75d52c0/examples/peripherals/spi_master
I2C example:  https://github.com/espressif/esp-idf/tree/f4009b94dca9d17b909e1094d6e3d7dbb75d52c0/examples/peripherals/i2c
OLED driver:  https://github.com/olikraus/u8g2/tree/master/csrc
OLED datasheet: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
OLED example: https://github.com/yanbe/ssd1306-esp-idf-i2c
OLED article: http://robotcantalk.blogspot.co.uk/2015/03/interfacing-arduino-with-ssd1306-driven.html

SX1276 pins:
14 = GPIO14 = RST
 5 = GPIO5  = SCK
18 = GPIO18 = CS  = SS
19 = GPIO19 = MISO
27 = GPIO27 = MOSI
26 = GPIO26 = IRQ = DIO0

OLED type: U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8 (by Arduino)
OLED pins:
16 = GPIO16 = RST
 4 = GPIO04 = SDA
15 = GPIO15 = SCL

LED pin:
25 = GPIO25

Button pin:
 0 = GPIO0

UART0 pins: taken by console ?
 1 = GPIO1 = TxD  CPU->GPS
 3 = GPIO3 = RxD  GPS->CPU

GPS pins:
22 = GPIO22 = PPS
23 = GPIO23 = ENA

UART2 pins:
16 = GPIO16 = RxD -> taken by OLED ?
17 = GPIO17 = TxD

T-Beam board pinout and docs: http://tinymicros.com/wiki/TTGO_T-Beam

HPD13A = RF chip ?
23 = RST (?)
 5 = SCK
18 = NSS = CS
19 = MISO
27 = MOSI
26 = IO0 = IRQ

24AA32A = 32K I2C EEPROM => GPS I2C

GPS
12 = GPIO12 = RXD1
15 = GPIO15 = TXD1

PSRM32 = SDIO ?
16 = CS

21 = GPIO21 = green LED

*/
/*
https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

GPIO   HELTEC      TTGO       JACEK     M5_JACEK    T-Beam     T-Beamv10    FollowMe   Restrictions

 0    Button      Button                .           LCD/DC     LCD/DC                  Can lock the boot, better not pull it down
 1    CONS/TxD    CONS/TxD   CONS/TxD   CONS/TxD    CONS/TxD   CONS/TxD                Console/Program
 2                           SD/MISO    .           LCD/MOSI   LCD/MOSI     LED/DBG    Bootstrap: LOW to enter UART download mode
 3    CONS/RxD    CONS/RxD   CONS/RxD   CONS/RxD    CONS/RxD   CONS/RxD                Console/Program
 4    OLED/SDA    OLED/SDA   ADC/CS                 LCD/BCKL   LCD/BCKL     PERF/RST
 5    RF/SCK      RF/SCK     RF/SCK     GPS/ANT     RF/SCK     RF/SCK       RF/CS                          PWM at boot
 6                                      SD/CLK                              SD/CLK     SD/CLK              SPI flash
 7                                      SD/DATA0                                       SD/DATA0            SPI flash
 8                                      SD/DATA1                                       SD/DATA1            SPI flash
 9                                      SD/DATA2                                       SD/DATA2            SPI flash
10                                      SD/DATA3                                       SD/DATA3            SPI flash
11                                      SD/CMD                              SD/CMD     SD/CMD              SPI flash
12    GPS/RxD     GPS/RxD    SD/CS      GPS/RxD                GPS/TxD      SD/MISO    HSPI/MISO JTAG/TDI  Bootstrap: select output voltage to power the flash chip
13    GPS/Ena     GPS/Ena    SD/SCK     LCD/SCL     LCD/CLK    LCD/CLK      SD/MOSI    HSPI/MOSI JTAG/TCK
14    RF/RST      RF/RST     Beeper     ?           LED/PCB    .            SD/CLK     HSPI/CLK  JTAG/TMS  PWM at boot
15    OLED/SCL    OLED/SCL   SD/MOSI    GPS/TxD                .            HSPI/CS0   JTAG/TDO            PWM at boot
16    OLED/RST    OLED/RST   RF/IRQ                 GPS/Tx     RAM/CS       GPS/TX     U2_RXD
17    Beeper      Beeper     RF/RST                 GPS/Rx     Beeper       GPS/RX     U2_TXD
18    RF/CS       RF/CS      RF/MISO    RF/CS       RF/SCK     RF/CS                   VSPI/CLK
19    RF/MISO     RF/MISO    RF/MOSI    RF/MISO     RF/MISO    RF/MISO      RF/MISO    VSPI/MISO
20                                                                                     not listed
21                LED?       RF/CS      I2C/SCL     I2C/SDA    I2C/SDA      I2C/SDA    VSPI/QUADHP
22                .          PWR/ON     I2C/SDA     I2C/CLK    I2C/SCL                 VSPI/QUADWP
23                .          PWR/LDO    RF/RST      RF/MOSI    RF/RST       RF/MOSI    VSPI/MOSI
24                                                                                     not listed
25    LED         Speaker    .          Speaker     Speaker    Speaker      TT/RX
26    RF/IRQ      RF/IRQ     SCL        RF/IRQ      RF/IRQ     RF/IRQ       PWR/Good
27    RF/MOSI     RF/MOSI    SDA                    RF/MOSI    RF/MOSI      TT/TX
28                                                                                     not listed
29                                                                                     not listed
30                                                                                     not listed
31                                                                                     not listed
32    .           .          GPS/TxD    ?                      .            TT/RST     XTAL
33    .           .          OLED/RST   ?                      .            GPS/EN     XTAL
34    GPS/PPS     GPS/PPS    GPS/RxD    KNOB/Sense             GPS/TxD      GPS/PPS
35    GPS/TxD     GPS/TxD    GPS/PPS    BAT/Sense              AXP/IRQ      RF/IRQ
36    .           .          BAT/Sense  BAT/Ext.               .            Vbat/Sense
37    .           .                                            GPS/PPS
38    .           .          Button                                         Button
39    .           .                     Button      Button     Button

*/


#ifdef WITH_TTGO
#define PIN_LED_PCB  GPIO_NUM_2   // status LED on the PCB
#endif

#if defined(WITH_HELTEC) || defined(WITH_HELTEC_V2)
#define PIN_LED_PCB  GPIO_NUM_25  // status LED on the PCB: 25, GPIO25 is DAC2
#endif

#if defined(WITH_TBEAM) // || defined(WITH_TBEAM_V10)
#define PIN_LED_PCB  GPIO_NUM_14  // blue LED on the PCB: 14
// #define PIN_LED_PCB_INV
#endif

#ifdef WITH_FollowMe
#define PIN_LED_PCB  GPIO_NUM_2   // debug LED
#define PIN_LED_TX   GPIO_NUM_15
#ifdef WITH_BQ
#define PIN_POWER_GOOD GPIO_NUM_26
#endif
#endif

// #define PIN_LED_TX   GPIO_NUM_??
// #define PIN_LED_RX   GPIO_NUM_??

#ifdef WITH_JACEK
#define PIN_RFM_RST  GPIO_NUM_17  // Reset
#define PIN_RFM_IRQ  GPIO_NUM_16  // packet done on receive or transmit
#define PIN_RFM_SS   GPIO_NUM_21  // SPI chip-select
#define PIN_RFM_SCK  GPIO_NUM_5   // SPI clock
#define PIN_RFM_MISO GPIO_NUM_18  // SPI MISO
#define PIN_RFM_MOSI GPIO_NUM_19  // SPI MOSI
#endif // JACEK

#ifdef WITH_M5_JACEK
#define PIN_RFM_RST  GPIO_NUM_13  // Reset
#define PIN_RFM_IRQ  GPIO_NUM_35  // Packet done on receive or transmit
#define PIN_RFM_SS   GPIO_NUM_12  // SPI chip-select
#define PIN_RFM_SCK  GPIO_NUM_18  // SPI clock (same as LCD)
#define PIN_RFM_MISO GPIO_NUM_19  // SPI MISO  (save as LCD)
#define PIN_RFM_MOSI GPIO_NUM_23  // SPI MOSI  (save as LCD)
#endif // M5_JACEK

#if defined(WITH_HELTEC) || defined(WITH_HELTEC_V2) || defined(WITH_TTGO)
#define PIN_RFM_RST  GPIO_NUM_14  // Reset
#define PIN_RFM_IRQ  GPIO_NUM_26  // packet done on receive or transmit
#define PIN_RFM_SS   GPIO_NUM_18  // SPI chip-select
#define PIN_RFM_SCK  GPIO_NUM_5   // SPI clock
#define PIN_RFM_MISO GPIO_NUM_19  // SPI MISO
#define PIN_RFM_MOSI GPIO_NUM_27  // SPI MOSI
#endif // HELTEC TTGO

#ifdef WITH_TBEAM
#define PIN_RFM_RST  GPIO_NUM_23  // Reset - not clear if T-Beam is using it, or maybe only the older version
#define PIN_RFM_IRQ  GPIO_NUM_26  // packet done on receive or transmit
#define PIN_RFM_SS   GPIO_NUM_18  // SPI chip-select
#define PIN_RFM_SCK  GPIO_NUM_5   // SPI clock
#define PIN_RFM_MISO GPIO_NUM_19  // SPI MISO
#define PIN_RFM_MOSI GPIO_NUM_27  // SPI MOSI
#endif // TBEAM

#ifdef WITH_TBEAM_V10
#define PIN_RFM_RST  GPIO_NUM_23  // Reset 23
#define PIN_RFM_IRQ  GPIO_NUM_26  // packet done on receive or transmit
#define PIN_RFM_SS   GPIO_NUM_18  // SPI chip-select
#define PIN_RFM_SCK  GPIO_NUM_5   // SPI clock
#define PIN_RFM_MISO GPIO_NUM_19  // SPI MISO
#define PIN_RFM_MOSI GPIO_NUM_27  // SPI MOSI
#endif // TBEAM

#ifdef WITH_FollowMe
// #define PIN_RFM_RST  GPIO_NUM_32  // Reset
#define PIN_RFM_IRQ  GPIO_NUM_35  // 39 // packet done on receive or transmit
#define PIN_RFM_SS   GPIO_NUM_5   // SPI chip-select
#define PIN_RFM_SCK  GPIO_NUM_18  // SPI clock
#define PIN_RFM_MISO GPIO_NUM_19  // SPI MISO
#define PIN_RFM_MOSI GPIO_NUM_23  // SPI MOSI
#endif // FollowMe

#define RFM_SPI_HOST  VSPI_HOST   // or H or VSPI_HOST ?
#define RFM_SPI_DMA   1           // DMA channel
#define RFM_SPI_SPEED 4000000     // [Hz] 4MHz SPI clock rate for RF chip

#ifdef WITH_ST7789
#ifdef WITH_TBEAM                  // old T-Beam
#define LCD_PIN_MOSI GPIO_NUM_2    // SDA
#define LCD_PIN_MISO GPIO_NUM_NC   // MISO not connected
#define LCD_PIN_CLK  GPIO_NUM_13   // SCL
#endif // TBEAM
#ifdef WITH_TBEAM_V10              // new T-Beam
#define LCD_PIN_MOSI GPIO_NUM_14 // 13   // SDA
#define LCD_PIN_MISO GPIO_NUM_NC   // MISO not connected
#define LCD_PIN_CLK  GPIO_NUM_13	 // 2   // SCL
// #define LCD_PIN_MOSI GPIO_NUM_2 // 13   // SDA
// #define LCD_PIN_MISO GPIO_NUM_NC   // MISO not connected
// #define LCD_PIN_CLK  GPIO_NUM_13 // 14   // SCL
// #define LCD_PIN_RST  GPIO_NUM_15 // NC  // RST
#endif // TBEAM v1.0
#define LCD_SPI_SPEED 10000000     // [Hz]
#define LCD_SPI_HOST HSPI_HOST     // VSPI_HOST or HSPI_HOST
#define LCD_SPI_DMA          2     // DMA channel
#define LCD_SPI_MODE         3     // for ST7789 with CS tied to LOW
#endif

#ifdef WITH_JACEK
#define PIN_GPS_TXD  GPIO_NUM_32
#define PIN_GPS_RXD  GPIO_NUM_34
#define PIN_GPS_PPS  GPIO_NUM_35
#endif // JACEK

#ifdef WITH_M5_JACEK
#define PIN_GPS_TXD  GPIO_NUM_17
#define PIN_GPS_RXD  GPIO_NUM_16
#define PIN_GPS_PPS  GPIO_NUM_26
#endif // M5_JACEK

#ifdef WITH_TBEAM_V10
#define PIN_AXP_IRQ GPIO_NUM_35
#endif

#if defined(WITH_HELTEC) || defined(WITH_TTGO)
                                  // VK2828U   GN-801   MAVlink
#define PIN_GPS_TXD  GPIO_NUM_13  // green     green    green
#define PIN_GPS_RXD  GPIO_NUM_35 // blue      yellow   yellow
#define PIN_GPS_PPS  GPIO_NUM_34 // white     blue
// #define PIN_GPS_RXD  GPIO_NUM_38  // for a new HELTEC
// #define PIN_GPS_PPS  GPIO_NUM_37  // for a new HELTEC
// #define PIN_GPS_ENA  GPIO_NUM_13  // yellow    white
#endif // HELTEC || TTGO

#if defined(WITH_HELTEC_V2)
                                  // VK2828U   GN-801   MAVlink
#define PIN_GPS_TXD  GPIO_NUM_13  // green     green    green
#define PIN_GPS_RXD  GPIO_NUM_39 // blue      yellow   yellow
#define PIN_GPS_PPS  GPIO_NUM_38 // white     blue
#endif

// Note: I had a problem with GPS ENABLE on GPIO13, thus I tied the enable wire to 3.3V for the time being.

#ifdef WITH_TBEAM
#define PIN_GPS_TXD  GPIO_NUM_15
#define PIN_GPS_RXD  GPIO_NUM_12
#endif // TBEAM

#ifdef WITH_TBEAM_V10
#define PIN_GPS_TXD  GPIO_NUM_12
#define PIN_GPS_RXD  GPIO_NUM_34
#define PIN_GPS_PPS  GPIO_NUM_37
#endif // TBEAM v1.0

#ifdef WITH_FollowMe              // L80 GPS with PPS, Enable and Reset
#define PIN_GPS_TXD  GPIO_NUM_17
#define PIN_GPS_RXD  GPIO_NUM_16
#define PIN_GPS_PPS  GPIO_NUM_34  // high active
#define PIN_GPS_ENA  GPIO_NUM_33  // Enable: high-active

#define PIN_PERIPH_RST GPIO_NUM_4   // Reset: high-active
#endif

#define CONS_UART UART_NUM_0      // UART0 for the console (the system does this for us)
#define GPS_UART  UART_NUM_1      // UART1 for GPS data read and dialog

#define I2C_BUS     I2C_NUM_1     // use bus #1 to talk to OLED and Baro sensor

#ifdef WITH_FollowMe
#define AERO_UART     UART_NUM_2  // UART2
#define PIN_AERO_TXD  GPIO_NUM_25
#define PIN_AERO_RXD  GPIO_NUM_27
#endif

uint8_t BARO_I2C = (uint8_t)I2C_BUS;

#ifdef WITH_JACEK
#define PIN_I2C_SCL GPIO_NUM_26   // SCL pin
#define PIN_I2C_SDA GPIO_NUM_27   // SDA pin
#endif // JACEK

#ifdef WITH_M5_JACEK
#define PIN_I2C_SCL GPIO_NUM_22   // SCL pin
#define PIN_I2C_SDA GPIO_NUM_21   // SDA pin
#endif // M5_JACEK

#if defined(WITH_HELTEC) || defined(WITH_HELTEC_V2) || defined(WITH_TTGO)
#define PIN_I2C_SCL GPIO_NUM_15   // SCL pin
#define PIN_I2C_SDA GPIO_NUM_4    // SDA pin
#define OLED_I2C_ADDR 0x3C        // I2C address of the OLED display
#define PIN_OLED_RST GPIO_NUM_16  // OLED RESET: low-active
#endif // HELTEC || TTGO

#if defined(WITH_TBEAM) || defined(WITH_TBEAM_V10) // T-Beam
#define PIN_I2C_SCL GPIO_NUM_22   // SCL pin => this way the pin pattern fits the BMP280 module
#define PIN_I2C_SDA GPIO_NUM_21   // SDA pin
#define OLED_I2C_ADDR 0x3C        // I2C address of the OLED display
#endif // TBEAM

#ifdef WITH_FollowMe              //
#define PIN_I2C_SCL GPIO_NUM_22   // SCL pin
#define PIN_I2C_SDA GPIO_NUM_21   // SDA pin
#define OLED_I2C_ADDR 0x3C        // I2C address of the OLED display
// #define PIN_OLED_RST GPIO_NUM_15  // OLED RESET: low-active
#endif

// #if defined(WITH_HELTEC) || defined(WITH_TTGO)
// #define PIN_OLED_RST GPIO_NUM_16  // OLED RESET: low-active
// #endif

#ifdef WITH_JACEK

#define PIN_OLED_RST  GPIO_NUM_33 // OLED RESET: low-active
#define OLED_I2C_ADDR 0x3C        // I2C address of the OLED display

#define PIN_POWER     GPIO_NUM_22 // power to GPS, RF, ...
#define PIN_POWER_LDO GPIO_NUM_23 // LOW=low power LDO, HIGH=higher power LDO

#define PIN_SD_CS     GPIO_NUM_12 // SD card interface in SPI mode
#define PIN_SD_SCK    GPIO_NUM_13
#define PIN_SD_MISO   GPIO_NUM_2
#define PIN_SD_MOSI   GPIO_NUM_15
#define SD_SPI_DMA              2

#define PIN_BEEPER    GPIO_NUM_14

#endif // JACEK

#if defined(WITH_TBEAM) || defined(WITH_TBEAM_V10)
#define PIN_BEEPER    GPIO_NUM_25 // same as DAC
#endif // TBEAM

#if defined(WITH_HELTEC) || defined(WITH_HELTEC_V2) || defined(WITH_TTGO)
#define PIN_BEEPER    GPIO_NUM_17
#endif // HELTEC || TTGO

#ifdef WITH_M5_JACEK

#define PIN_GPS_ANT   GPIO_NUM_5  // internal(H) or external(L) GPS antenna

#define PIN_SD_CS     GPIO_NUM_11 // SD card interface in SPI mode
#define PIN_SD_SCK    GPIO_NUM_6
#define PIN_SD_MISO   GPIO_NUM_7
#define PIN_SD_MOSI   GPIO_NUM_8
#define SD_SPI_DMA             2

#define PIN_BEEPER    GPIO_NUM_25 // DAC

#endif // M5_JACEK

#if !defined(WITH_AXP) && !defined(WITH_BQ) && !defined(WITH_OLED) && !defined(WITH_U8G2_OLED) && !defined(WITH_BMP180) && !defined(WITH_BMP280) && !defined(WITH_BME280)
#undef PIN_I2C_SCL
#undef PIN_I2C_SDA
#endif

#ifdef WITH_FollowMe
#define PIN_SD_MISO   GPIO_NUM_12 // SD card in simple SPI mode, using HSPI IOMUX pins
#define PIN_SD_MOSI   GPIO_NUM_13
#define PIN_SD_SCK    GPIO_NUM_14
#define PIN_SD_CS     GPIO_NUM_15
#define SD_SPI_DMA              2
#endif // FollowMe

#ifdef WITH_M5_JACEK              // the three buttons below the LCD

#define PIN_BUTTON_L  GPIO_NUM_37 // Left
#define PIN_BUTTON    GPIO_NUM_38 // Center
#define PIN_BUTTON_R  GPIO_NUM_39 // Right

#endif

#if defined(WITH_FollowMe) // || defined(WITH_TBEAM) || defined(WITH_TBEAM_V10)
#define PIN_BUTTON    GPIO_NUM_39 // or 38 ?
#endif

#if defined(WITH_TBEAM_V10)
#define PIN_BUTTON    GPIO_NUM_38
#endif

#ifdef WITH_JACEK
#define PIN_BUTTON    GPIO_NUM_38
#endif

#if defined(WITH_TTGO) || defined(WITH_HELTEC) || defined(WITH_HELTEC_V2)
#define PIN_BUTTON    GPIO_NUM_0
#endif

#ifndef PIN_BUTTON
#define PIN_BUTTON    GPIO_NUM_39
#endif

// ======================================================================================================
// 48-bit unique ID of the chip

uint64_t getUniqueID(void)
{ uint64_t ID=0; esp_err_t ret=esp_efuse_mac_get_default((uint8_t *)&ID); return ID; }

uint32_t getUniqueAddress(void)
{ uint32_t ID = getUniqueID()>>24;
  ID &= 0x00FFFFFF;
  ID = (ID>>16) | (ID&0x00FF00) | (ID<<16);
  ID &= 0x00FFFFFF;
  return ID; }

// ======================================================================================================

#ifdef WITH_MAVLINK
uint8_t  MAV_Seq=0;                   // sequence number for MAVlink message sent out
#endif

// ======================================================================================================

// system_get_time()  - return s 32-bit time in microseconds since the system start
// gettimeofday()
// xthal_get_ccount() - gets Xtal or master clock counts ?

// ======================================================================================================

FlashParameters Parameters;

//--------------------------------------------------------------------------------------------------------
// Power control

#ifdef WITH_JACEK
void POWER_Dir    (void) { gpio_set_direction(PIN_POWER, GPIO_MODE_OUTPUT); }
void POWER_On     (void) { gpio_set_level(PIN_POWER, 0); }      //
void POWER_Off    (void) { gpio_set_level(PIN_POWER, 1); }
void POWER_LDO_Dir(void) { gpio_set_direction(PIN_POWER_LDO, GPIO_MODE_OUTPUT); }
void POWER_LDO_On (void) { gpio_set_level(PIN_POWER_LDO, 1); }  //
void POWER_LDO_Off(void) { gpio_set_level(PIN_POWER_LDO, 0); }
#endif

#ifdef WITH_M5_JACEK
void GPS_ANT_Dir (void)              { gpio_set_direction(PIN_GPS_ANT, GPIO_MODE_OUTPUT); }
void GPS_ANT_Sel (uint8_t Antenna=1) { gpio_set_level(PIN_GPS_ANT, Antenna); } // 1=internal, 0=external
#endif

#ifdef PIN_POWER_GOOD
bool Power_isGood(void) { return gpio_get_level(PIN_POWER_GOOD); }
#endif

//--------------------------------------------------------------------------------------------------------
// Status LED on the PCB

#ifdef PIN_LED_PCB
void LED_PCB_Dir  (void) { gpio_set_direction(PIN_LED_PCB, GPIO_MODE_OUTPUT); }
#ifdef PIN_LED_PCB_INV
void LED_PCB_On   (void) { gpio_set_level(PIN_LED_PCB, 0); }
void LED_PCB_Off  (void) { gpio_set_level(PIN_LED_PCB, 1); }
#else
void LED_PCB_On   (void) { gpio_set_level(PIN_LED_PCB, 1); }
void LED_PCB_Off  (void) { gpio_set_level(PIN_LED_PCB, 0); }
#endif
#else                                            // if LED on the PCB is absent, just make dummy calls
void LED_PCB_Dir  (void) { }
void LED_PCB_On   (void) { }
void LED_PCB_Off  (void) { }
#endif

#ifdef WITH_LED_TX
void LED_TX_Dir  (void) { gpio_set_direction(PIN_LED_TX, GPIO_MODE_OUTPUT); }
void LED_TX_On   (void) { gpio_set_level(PIN_LED_TX, 0); }
void LED_TX_Off  (void) { gpio_set_level(PIN_LED_TX, 1); }
#endif

#ifdef PIN_BUTTON
void Button_Dir      (void) { gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT); }
bool Button_isPressed(void) { return !gpio_get_level(PIN_BUTTON); }
#endif

// ========================================================================================================

// ~/esp-idf/components/bt/bluedroid/api/include/esp_spp_api.h
// esp_err_t esp_spp_write(uint32_t handle, int len, uint8_t *p_data);

#ifdef WITH_BT_SPP

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t  sec_mask     = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave   = ESP_SPP_ROLE_SLAVE;

static FIFO<char, 512> BT_SPP_TxFIFO;         //
static uint32_t        BT_SPP_Conn = 0;       // BT incoming connection handle
static uint32_t        BT_SPP_TxCong = 0;     // congestion control
// static TickType_t      BT_SPP_LastTxPush=0;   // [ms]

// static esp_bd_addr_t BT_SPP_MAC;  // BT incoming connection MAC - could be used for pilot id in the flight log
// static uint32_t BT_SPP_Wait = 0;  // bytes waiting to be written into BT_SPP

// static const char *BT_SPP_Welcome = "ESP32 OGN-Tracker\n";

static void setPilotID(esp_bd_addr_t MAC, size_t Len=6)           // set PilotID in the parameters from the BT SPP client MAC (thus Pilot's smartphone)
{ char *ID = Parameters.PilotID;
  ID[0]='B'; ID[1]='T'; ID[2]='_'; ID+=3;
  for(int Idx=0; Idx<Len; Idx++)
  { Format_Hex(ID, MAC[Idx]); ID+=2; }
  ID[0]=0; }

static void clrPilotID(void)                                      // clear the Pilot_ID when BT SPP gets disconnected
{ Parameters.PilotID[0]=0; }

static size_t BT_SPP_TxPush(size_t MaxLen=128)                    // transmit part of the TxFIFO to the BT link
{ // BT_SPP_LastTxPush = xTaskGetTickCount();                        // [ms] remember last time the TxPush was done
  char *Data; size_t Len=BT_SPP_TxFIFO.getReadBlock(Data);        // see how much data is there in the queue for transmission
  if(Len==0) return 0;                                            // if block is empty then give up
  if(Len>MaxLen) Len=MaxLen;                                      // limit the block size
  esp_err_t Ret=esp_spp_write(BT_SPP_Conn, Len, (uint8_t *)Data); // write the block to the BT
  if(Ret!=ESP_OK) return 0;                                       // if an error then give up
  BT_SPP_TxFIFO.flushReadBlock(Len);                              // remove the transmitted block from the FIFO
  return Len; }                                                   // return number of transmitted bytes

static void esp_spp_cb(esp_spp_cb_event_t Event, esp_spp_cb_param_t *Param)
{ switch (Event)
  { case ESP_SPP_INIT_EVT:                                        // [0]
      esp_bt_dev_set_device_name(Parameters.BTname);
      // esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE); // for older ESP-IDF
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
      esp_spp_start_srv(sec_mask, role_slave, 0, "SPP_SERVER");
      break;
    case ESP_SPP_DISCOVERY_COMP_EVT:                              // [8]
      break;
    case ESP_SPP_START_EVT:                                       // [28] SPP server started succesfully
      break;
    case ESP_SPP_SRV_OPEN_EVT:                                    // [34] server connection opens: new handle comes
      BT_SPP_TxFIFO.Clear();                                      // clear the TxFIFO
      BT_SPP_Conn = Param->srv_open.handle;                       // store handle for esp_spp_write()
      BT_SPP_TxCong = 0;                                          // assume no congestion
      setPilotID(Param->srv_open.rem_bda, sizeof(esp_bd_addr_t)); // PilotID is not taken from the connected BT client
      // memcpy(BT_SPP_MAC, Param->srv_open.rem_bda, sizeof(esp_bd_addr_t));
      // esp_spp_write(Param->srv_open.handle, BT_SPP_Wait, (uint8_t *)BT_SPP_Welcome); // write Welcome message to the BT_SPP
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "BT_SPP: ");
      Format_MAC(CONS_UART_Write, Param->srv_open.rem_bda, sizeof(esp_bd_addr_t));
      Format_String(CONS_UART_Write, " connected\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      break;
    case ESP_SPP_OPEN_EVT:                                        // [26] connection opens: what's the difference to ESP_SPP_SRV_OPEN_EVT ?
      // Param->open.handle, Param->open.rem_bda
      break;
    case ESP_SPP_CLOSE_EVT:                                       // [27] connection closes for given handle
      BT_SPP_Conn=0;                                              // clear the handle: signal the BT connection is off
      clrPilotID();
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "BT_SPP: \n");
      // Format_MAC(CONS_UART_Write, BT_SPP_MAC);
      Format_String(CONS_UART_Write, " disconnected\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      break;
    case ESP_SPP_DATA_IND_EVT:                                    // [30] data is sent by the client
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Param->data_ind.handle, Param->data_ind.data, Param->data_ind.len
      Format_String(CONS_UART_Write, "BT_SPP: [");
      Format_UnsDec(CONS_UART_Write, Param->data_ind.len);
      Format_String(CONS_UART_Write, "]\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      break;
    case ESP_SPP_CONG_EVT:                                        // [31] congestion on the outgoing data
      BT_SPP_TxCong = Param->cong.cong;
      break;
    case ESP_SPP_WRITE_EVT:                                       // [33] (queued) data has been sent to the client
      BT_SPP_TxCong = Param->write.cong;
      break;
    default:
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "BT_SPP: Event ");
      Format_UnsDec(CONS_UART_Write, (uint32_t)Event);
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      break;
  }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t Event, esp_bt_gap_cb_param_t *Param)
{
  switch (Event)     // event numbers are in esp-idf/components/bt/bluedroid/api/include/api/esp_gap_bt_api.h
  {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      if (Param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
      { Format_String(CONS_UART_Write, "BT_GAP: ");
        Format_String(CONS_UART_Write, (const char *)Param->auth_cmpl.device_name);
        Format_String(CONS_UART_Write, " authenticated\n"); }
      else
      { Format_String(CONS_UART_Write, "BT_GAP: Authentication failure (");
        Format_SignDec(CONS_UART_Write, Param->auth_cmpl.stat);
        Format_String(CONS_UART_Write, ")\n"); }
      // ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
      // esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
      // ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
      xSemaphoreGive(CONS_Mutex);
      break;
    case ESP_BT_GAP_PIN_REQ_EVT:
     /*
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
      */
      break;
    default:
      break;
    }

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "BT_GAP: Event ");
  Format_UnsDec(CONS_UART_Write, (uint32_t)Event);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
}

void static BT_SPP_Write (char Byte)  // send a character to the BT serial port
{ if(BT_SPP_Conn)                                                                           // if BT connection is active
  { BT_SPP_TxFIFO.Write(Byte);                                                              // write the byte into the TxFIFO
    // TickType_t Behind = xTaskGetTickCount() - BT_SPP_LastTxPush;                         // [ms]
    // if(Behind>=20) BT_SPP_TxPush();
    if( (BT_SPP_TxCong==0) && ( (Byte=='\n') || (BT_SPP_TxFIFO.Full()>=64) ) )              // if no congestion and EOL or 64B waiting already
    { BT_SPP_TxPush(); }                                                                    // read a block from TxFIFO ad push it into the BT_SPP
  }
}

int BT_SPP_Init(void)
{ esp_err_t Err=ESP_OK;
  if(Parameters.BTname[0]==0) return Err;

  esp_bt_controller_config_t BTconf = BT_CONTROLLER_INIT_CONFIG_DEFAULT();                  // the default mode is defined by the menuconfig settings
  Err = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  Err = esp_bt_controller_init(&BTconf); if(Err!=ESP_OK) return Err;
  Err = esp_bt_controller_enable((esp_bt_mode_t)BTconf.mode); if(Err!=ESP_OK) return Err;   // mode must be same as in BTconf
  Err = esp_bluedroid_init(); if(Err!=ESP_OK) return Err;                                   // init the BT stack
  Err = esp_bluedroid_enable(); if(Err!=ESP_OK) return Err;                                 // enable the BT stack
  Err = esp_bt_gap_register_callback(esp_bt_gap_cb); if(Err!=ESP_OK) return Err;
  Err = esp_spp_register_callback(esp_spp_cb); if(Err!=ESP_OK) return Err;
  Err = esp_spp_init(esp_spp_mode); if(Err!=ESP_OK) return Err;

  // Set default parameters for Secure Simple Pairing */
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE; // _IO;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

  // Set default parameters for Legacy Pairing: fixed PIN
  esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
  esp_bt_pin_code_t pin_code = { '0', '1', '2', '3' };
  esp_bt_gap_set_pin(pin_type, 4, pin_code);

  // set the UUID so this BT device is recognized as a serial port: thanks to Linar for this code
  esp_bt_cod_t cod;
  cod.minor   =      0b000101;
  cod.major   =       0b00001;
  cod.service = 0b00000001101; // 0b00000001101 = serial port, 0b00000010000 = generic
  esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

  return Err; }

#endif // WITH_BT_SPP

// ========================================================================================================

// Console UART

SemaphoreHandle_t CONS_Mutex;

/*
bool CONS_InpReady(void)
{ struct timeval tv = { tv_sec:0, tv_usec:0} ;
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return (FD_ISSET(0, &fds)); }
*/
// int  CONS_UART_Read       (uint8_t &Byte) { return uart_read_bytes  (CONS_UART, &Byte, 1, 0); }  // non-blocking
// void CONS_UART_Write      (char     Byte) {        uart_write_bytes (CONS_UART, &Byte, 1);    }  // blocking ?
void CONS_UART_Write (char     Byte)
{ putchar(Byte);
#ifdef WITH_BT_SPP
  BT_SPP_Write(Byte);
#endif
}                                            // it appears the NL is translated into CR+NL
int  CONS_UART_Read  (uint8_t &Byte)  { int Ret=getchar(); if(Ret>=0) { Byte=Ret; return 1; } else return Ret; }
// int  CONS_UART_Free  (void)           { return UART2_Free(); }
// int  CONS_UART_Full  (void)           { return UART2_Full(); }

void  CONS_UART_SetBaudrate(int BaudRate)  {        uart_set_baudrate(CONS_UART, BaudRate);    }

//--------------------------------------------------------------------------------------------------------
// ADS-B UART

#ifdef AERO_UART
int   AERO_UART_Read       (uint8_t &Byte) { return uart_read_bytes  (AERO_UART, &Byte, 1, 0); }  // should be buffered and non-blocking
void  AERO_UART_Write      (char     Byte) {        uart_write_bytes (AERO_UART, &Byte, 1);    }  // should be buffered and blocking
void  AERO_UART_SetBaudrate(int BaudRate)  {        uart_set_baudrate(AERO_UART, BaudRate);    }
#endif

//--------------------------------------------------------------------------------------------------------
// GPS UART

#ifdef GPS_UART
// int   GPS_UART_Full       (void)          { size_t Full=0; uart_get_buffered_data_len(GPS_UART, &Full); return Full; }
int   GPS_UART_Read       (uint8_t &Byte) { return uart_read_bytes  (GPS_UART, &Byte, 1, 0); }  // should be buffered and non-blocking
void  GPS_UART_Write      (char     Byte) {        uart_write_bytes (GPS_UART, &Byte, 1);    }  // should be buffered and blocking
void  GPS_UART_SetBaudrate(int BaudRate)  {        uart_set_baudrate(GPS_UART, BaudRate);    }
#endif

#ifdef WITH_GPS_ENABLE
void GPS_DISABLE(void) { gpio_set_level(PIN_GPS_ENA, 0); }
void GPS_ENABLE (void) { gpio_set_level(PIN_GPS_ENA, 1); }
#endif

#ifdef PIN_GPS_PPS
bool GPS_PPS_isOn(void) { return gpio_get_level(PIN_GPS_PPS); }
#endif

//--------------------------------------------------------------------------------------------------------
// RF chip

#ifdef PIN_RFM_RST      // if reset pin declared for the RF chip
void RFM_RESET_SetInput  (void)         { gpio_set_direction(PIN_RFM_RST, GPIO_MODE_INPUT); }
void RFM_RESET_SetOutput (void)         { gpio_set_direction(PIN_RFM_RST, GPIO_MODE_OUTPUT); }
void RFM_RESET_SetLevel  (uint8_t High) { gpio_set_level(PIN_RFM_RST, High&1); }

#ifdef WITH_RFM95       // for RFM95 reset is low-active
void RFM_RESET(uint8_t On) { if(On&1) { RFM_RESET_SetOutput(); RFM_RESET_SetLevel(0); } else RFM_RESET_SetInput(); }
#endif

#ifdef WITH_RFM69       // for RFM69 reset is high-active
void RFM_RESET(uint8_t On) { RFM_RESET_SetLevel(On); }
#endif

#else                   // if no reset pin declared for the RF chip, then make an empty call
inline void RFM_RESET_SetOutput (void) { }
void RFM_RESET(uint8_t On) { }
#endif // PIN_RFM_RST

void RFM_IRQ_SetInput(void) { gpio_set_direction(PIN_RFM_IRQ, GPIO_MODE_INPUT); }
bool RFM_IRQ_isOn(void)      { return gpio_get_level(PIN_RFM_IRQ); }

static spi_device_handle_t RFM_SPI;

void RFM_TransferBlock(uint8_t *Data, uint8_t Len)
{ spi_transaction_t Trans;
  memset(&Trans, 0, sizeof(Trans));
  Trans.tx_buffer = Data;
  Trans.rx_buffer = Data;
  Trans.length = 8*Len;
  // esp_err_t ret = spi_device_polling_transmit(RFM_SPI, &Trans); }
  esp_err_t ret = spi_device_transmit(RFM_SPI, &Trans); }

//--------------------------------------------------------------------------------------------------------

#ifdef WITH_TFT_LCD

static void TFT_LCD_Init(void)
{ TFT_PinsInit();

  spi_lobo_device_handle_t spi;

  spi_lobo_bus_config_t buscfg =
  { .mosi_io_num=PIN_NUM_MOSI,              // set SPI MOSI pin
#ifdef PIN_NUM_MISO
    .miso_io_num = PIN_NUM_MISO,            // set SPI MISO pin
#else
    .miso_io_num = -1,
#endif
    .sclk_io_num = PIN_NUM_CLK,             // set SPI CLK pin
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 6*1024 };

  spi_lobo_device_interface_config_t devcfg =
  { .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
#ifdef PIN_NUM_CS
    .mode = 0,
#else
    .mode = 3,
#endif
    .duty_cycle_pos = 0,
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = 8000000,              // Initial clock out at 8 MHz
    .spics_io_num = -1,                     // we will use external CS pin
#ifdef PIN_NUM_CS
    .spics_ext_io_num = PIN_NUM_CS,         // external CS pin
#else
    .spics_ext_io_num = -1,
#endif
    .flags=LB_SPI_DEVICE_HALFDUPLEX,        // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
    .pre_cb = 0,
    .post_cb = 0,
    .selected = 0,
  };

  max_rdclock = 4000000;
  vTaskDelay(100);

#ifdef WITH_M5_JACEK
  esp_err_t ret=spi_lobo_bus_add_device(TFT_VSPI_HOST, &buscfg, &devcfg, &spi);
#endif
#ifdef WITH_TBEAM
  esp_err_t ret=spi_lobo_bus_add_device(TFT_VSPI_HOST, &buscfg, &devcfg, &spi);
#endif
  // assert(ret==ESP_OK);
  // printf("SPI: display device added to spi bus (%d)\r\n", SPI_BUS);
  disp_spi = spi;

  // ret = spi_lobo_device_select(spi, 1);
  // assert(ret==ESP_OK);
  // ret = spi_lobo_device_deselect(spi);
  // assert(ret==ESP_OK);

  TFT_display_init();
  // max_rdclock = find_rd_speed();

  font_rotate = 0;
  text_wrap = 0;
  font_transparent = 0;
  font_forceFixed = 0;
  gray_scale = 0;
  // disp_select();
  // spi_lobo_device_select(spi, 1);
  TFT_invertDisplay(INVERT_ON);
  TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
  // spi_lobo_device_deselect(spi);
  // disp_deselect();
  // TFT_setRotation(LANDSCAPE);
  TFT_setRotation(PORTRAIT);
  TFT_setFont(DEFAULT_FONT, NULL);
  TFT_resetclipwin();

  TFT_fillScreen(TFT_GREEN);
  // TFT_fillScreen(TFT_WHITE);
  // TFT_fillScreen(TFT_BLACK);
  TFT_resetclipwin();

  TFT_drawRect(20, 20, 200, 200, TFT_CYAN);

  TFT_print("OGN-Tracker", CENTER, CENTER);

}

#endif // WITH_TFT_LCD

//--------------------------------------------------------------------------------------------------------
// BEEPER

#ifdef WITH_BEEPER

static ledc_timer_config_t LEDC_Timer =
  {
    speed_mode      : LEDC_HIGH_SPEED_MODE,   // timer mode
    duty_resolution : LEDC_TIMER_8_BIT,       // resolution of PWM duty: 0..255
//   { duty_resolution : LEDC_TIMER_8_BIT, }     // resolution of PWM duty: 0..255
    timer_num       : LEDC_TIMER_0,           // timer index
    freq_hz         : 880                     // frequency of PWM signal
  } ;

static ledc_channel_config_t LEDC_Channel =
  {
    gpio_num   : PIN_BEEPER,
    speed_mode : LEDC_HIGH_SPEED_MODE,
    channel    : LEDC_CHANNEL_0,
    intr_type  : LEDC_INTR_DISABLE,
    timer_sel  : LEDC_TIMER_0,
    duty       : 0,
    hpoint     : 0
  } ;

esp_err_t Beep_Init(void)
{ ledc_timer_config(&LEDC_Timer);            // Set configuration of timer0 for high speed channels
  ledc_channel_config(&LEDC_Channel);
  return ESP_OK; }

void Beep(uint16_t Freq, uint8_t Duty, uint8_t DoubleAmpl) // [Hz, 1/256] play sound with given frequency and duty (=volume)
{ ledc_set_freq(LEDC_Timer.speed_mode, LEDC_Timer.timer_num, Freq);
  ledc_set_duty(LEDC_Channel.speed_mode, LEDC_Channel.channel, Duty);
  ledc_update_duty(LEDC_Channel.speed_mode, LEDC_Channel.channel); }

// Frequencies for notes of the highest octave: C,     C#,    D,     D#,    E,     F,     F#,    G,     G#,    A,     A#,    B
// Freq[i] = 32*523.25*2**(i/12)            i = 0,     1,     2,     3,     4,     5,     6,     7,     8,     9,     A,     B
static const uint16_t NoteFreq[12] =      { 16744, 17740, 18795, 19912, 21096, 22351, 23680, 25088, 26579, 28160, 29834, 31608 } ;

void Beep_Note(uint8_t Note) // Note = VVOONNNN: VV = Volume, OO=Octave, NNNN=Note
{ uint8_t Volume =  Note>>6;                             // [0..3]
  uint8_t Octave = (Note>>4)&0x03;                       // [0..3]
  Note &= 0x0F; if(Note>=12) { Note-=12; Octave+=1; }    // [0..11] [0..4]
  uint8_t Duty = 0; uint8_t DoubleAmpl=0;
  if(Volume) { Duty=0x10; Duty<<=Volume; }               // Duty = 0x00, 0x20, 0x40, 0x80
  if(Volume>2) { DoubleAmpl=1; }                         // DoubleAmpl = 0, 0, 1, 1
  uint16_t Freq = NoteFreq[Note];
  if(Octave) { Freq += 1<<(Octave-1); Freq >>= (4-Octave); }
  Beep(Freq, Duty, DoubleAmpl); }

uint8_t  Vario_Note=0x00; // 0x40;
uint16_t Vario_Period=800;
uint16_t Vario_Fill=50;

static volatile uint16_t Vario_Time=0;

static volatile uint8_t Play_Note=0;             // Note being played
static volatile uint8_t Play_Counter=0;          // [ms] time counter

static FIFO<uint16_t, 8> Play_FIFO;              // queue of notes to play

void Play(uint8_t Note, uint8_t Len)             // [Note] [ms] put a new not to play in the queue
{ uint16_t Word = Note; Word<<=8; Word|=Len; Play_FIFO.Write(Word); }

uint8_t Play_Busy(void) { return Play_Counter; } // is a note being played right now ?

void Play_TimerCheck(void)                       // every ms serve the note playing
{ uint8_t Counter=Play_Counter;
  if(Counter)                                    // if counter non-zero
  { Counter--;                                   // decrement it
    if(!Counter) Beep_Note(Play_Note=0x00);      // if reached zero, stop playing the note
  }
  if(!Counter)                                   // if counter reached zero
  { if(!Play_FIFO.isEmpty())                     // check for notes in the queue
    { uint16_t Word=0; Play_FIFO.Read(Word);     // get the next note
      Beep_Note(Play_Note=Word>>8); Counter=Word&0xFF; }   // start playing it, load counter with the note duration
  }
  Play_Counter=Counter;

  uint16_t Time=Vario_Time;
  Time++; if(Time>=Vario_Period) Time=0;
  Vario_Time = Time;

  if(Counter==0)                            // when no notes are being played, make the vario sound
  { if(Time<=Vario_Fill)
    { if(Play_Note!=Vario_Note) Beep_Note(Play_Note=Vario_Note); }
    else
    { if(Play_Note!=0) Beep_Note(Play_Note=0x00); }
  }
}

#endif

//--------------------------------------------------------------------------------------------------------
// SOUND

#ifdef WITH_SOUND

// extern const uint8_t Sound_737_Traffic_u8[]   asm("_binary_737_Traffic_u8_start");
// extern const uint8_t Sound_737_Traffic_end[]  asm("_binary_737_Traffic_u8_end");
// const int Sound_737_Traffic_size = Sound_737_Traffic_end-Sound_737_Traffic_u8;

// extern const uint8_t Sound_737_Sink_Rate_u8[]   asm("_binary_737_Sink_Rate_u8_start");
// extern const uint8_t Sound_737_Sink_Rate_end[]  asm("_binary_737_Sink_Rate_u8_end");
// const int Sound_737_Sink_Rate_size = Sound_737_Sink_Rate_end-Sound_737_Sink_Rate_u8;

// extern const uint8_t Sound_737_Whoop_Whoop_u8[]   asm("_binary_737_Whoop_Whoop_u8_start");
// extern const uint8_t Sound_737_Whoop_Whoop_end[]  asm("_binary_737_Whoop_Whoop_u8_end");
// const int Sound_737_Whoop_Whoop_size = Sound_737_Whoop_Whoop_end-Sound_737_Whoop_Whoop_u8;

// const uint32_t Sound_SampleRate = 16000;
const  int     Sound_BuffSize   =   256;

esp_err_t Sound_Init(void)
{
  i2s_config_t i2s_config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate          = Sound_SampleRate,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT, // I2S_CHANNEL_FMT_ALL_RIGHT or I2S_CHANNEL_FMT_ONLY_RIGHT ?
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags     = 0,
        .dma_buf_count        = 4,
        .dma_buf_len          = Sound_BuffSize,
        .use_apll             = 1,
        .tx_desc_auto_clear   = 1,
        .fixed_mclk           = 0 };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);         // install and start i2s driver
  // i2s_set_pin(I2S_NUM_0, 0);
  i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);                  // init DAC pad
  // i2s_set_sample_rates(I2S_NUM_0, Sound_SampleRate);
  // i2s_set_clk(I2S_NUM_0, Sound_SampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  return ESP_OK; }

static uint16_t Sound_Buffer[Sound_BuffSize];

int Sound_Play(const uint16_t *Data, int Len)
{ size_t Written=0;
  i2s_write(I2S_NUM_0, Data, Len<<1, &Written, portMAX_DELAY);
  return Written>>1; }

void Sound_PlayU8(const uint8_t *Data, uint16_t Len)
{ int BuffIdx=0;
  for( ; Len--; )
  { if(BuffIdx>=Sound_BuffSize) { Sound_Play(Sound_Buffer, Sound_BuffSize); BuffIdx=0; }
    uint16_t DAC = (*Data++); DAC<<=8;
    Sound_Buffer[BuffIdx++] = DAC; }
  if(BuffIdx) { Sound_Play(Sound_Buffer, BuffIdx); BuffIdx=0; }
}

void Sound_PlayS8(const int8_t *Data, uint16_t Len, uint8_t Vol)
{ if(Vol>8) Vol=8;
  int BuffIdx=0;
  for( ; Len--; )
  { if(BuffIdx>=Sound_BuffSize) { Sound_Play(Sound_Buffer, Sound_BuffSize); BuffIdx=0; }
    int16_t DAC = (*Data++); if(DAC&0x80) DAC|=0xFF00; DAC<<=Vol; DAC+=0x8000;
    Sound_Buffer[BuffIdx++] = DAC; }
  if(BuffIdx) { Sound_Play(Sound_Buffer, BuffIdx); BuffIdx=0; }
}

void Sound_PlaySilence(uint16_t Len)
{ int BuffIdx=0;
  for( ; Len--; )
  { if(BuffIdx>=Sound_BuffSize) { Sound_Play(Sound_Buffer, Sound_BuffSize); BuffIdx=0; }
    Sound_Buffer[BuffIdx++] = 0x8000; }
  if(BuffIdx) { Sound_Play(Sound_Buffer, BuffIdx); BuffIdx=0; }
}

static uint16_t Sound_Phase=0;

void Sound_Beep(int16_t Freq, uint16_t Len, int16_t Ampl)
{ int BuffIdx=0;
  for( ; Len--; )
  { if(BuffIdx>=Sound_BuffSize) { Sound_Play(Sound_Buffer, Sound_BuffSize); BuffIdx=0; } // if buffer full then play it
    int16_t Sig = IntSine(Sound_Phase)>>16;           // 16-bit sine
    // int16_t Sig = Isin(Sound_Phase);               // 13-bit, +/-4096 range sine
    Sound_Phase += Freq;                           // increment the Phase
    // Sig = ((int32_t)Sig*Ampl+0x1000)>>13;          // multiply by requested Amplitude
    Sig>>=1;
    uint16_t DAC = 0x8000+Sig; // DAC = (DAC&0xFF00) | (DAC>>8);
    Sound_Buffer[BuffIdx++] = DAC; }
  if(BuffIdx) { Sound_Play(Sound_Buffer, BuffIdx); BuffIdx=0; }
}

#endif

//--------------------------------------------------------------------------------------------------------
// OLED display

#ifdef WITH_OLED

#ifdef PIN_OLED_RST
void OLED_RESET(bool Level) { gpio_set_level(PIN_OLED_RST, Level); }
#endif

esp_err_t OLED_Init(uint8_t DispIdx)
{ i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ((OLED_I2C_ADDR+DispIdx)<<1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
  i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
  i2c_master_write_byte(cmd, 0x14, true);
  i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
  i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping
  i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);        // Turn ON the display
  i2c_master_stop(cmd);
  esp_err_t espRc = i2c_master_cmd_begin(I2C_BUS, cmd, 10);
  i2c_cmd_link_delete(cmd);
  return espRc; }

esp_err_t OLED_DisplayON(uint8_t ON, uint8_t DispIdx)
{ i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ((OLED_I2C_ADDR+DispIdx)<<1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
  i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF+ON, true);
  i2c_master_stop(cmd);
  esp_err_t espRc = i2c_master_cmd_begin(I2C_BUS, cmd, 10);
  i2c_cmd_link_delete(cmd);
  return espRc; }

esp_err_t OLED_DisplayINV(uint8_t INV, uint8_t DispIdx)
{ i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ((OLED_I2C_ADDR+DispIdx)<<1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
  i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL+INV, true);
  i2c_master_stop(cmd);
  esp_err_t espRc = i2c_master_cmd_begin(I2C_BUS, cmd, 10);
  i2c_cmd_link_delete(cmd);
  return espRc; }

esp_err_t OLED_SetContrast(uint8_t Contrast, uint8_t DispIdx)
{ i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ((OLED_I2C_ADDR+DispIdx)<<1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
  i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);
  i2c_master_write_byte(cmd, Contrast, true);
  i2c_master_stop(cmd);
  esp_err_t espRc = i2c_master_cmd_begin(I2C_BUS, cmd, 10);
  i2c_cmd_link_delete(cmd);
  return espRc; }

esp_err_t OLED_PutLine(uint8_t Line, const char *Text, uint8_t DispIdx)
{ if(Line>=8) return ESP_OK;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ((OLED_I2C_ADDR+DispIdx)<<1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
  i2c_master_write_byte(cmd, 0x00, true);           // 0x0L => column address: Lower nibble
  i2c_master_write_byte(cmd, 0x10, true);           // 0x1H => column address: Higher nibble
  i2c_master_write_byte(cmd, 0xB0 | Line, true);    // 0xBP => Page address
  i2c_master_stop(cmd);
  esp_err_t espRc = i2c_master_cmd_begin(I2C_BUS, cmd, 10);
  i2c_cmd_link_delete(cmd);
  if(espRc!=ESP_OK) return espRc;

  for(uint8_t Idx=0; Idx<16; Idx++)
  { char Char=0;
    if(Text)
    { Char=Text[Idx];
      if(Char==0) Text=0;
             else Char&=0x7F; }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((OLED_I2C_ADDR+DispIdx)<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
    i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)Char], 8, true);
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_BUS, cmd, 10);
    i2c_cmd_link_delete(cmd);
    if(espRc!=ESP_OK) break; }
  return espRc; }

esp_err_t OLED_Clear(uint8_t DispIdx)
{ esp_err_t espRc;
  for(uint8_t Line=0; Line<8; Line++)
  { espRc=OLED_PutLine(Line, 0, DispIdx); if(espRc!=ESP_OK) break; }
  return espRc; }
#endif

#ifdef WITH_U8G2_OLED

static i2c_cmd_handle_t U8G2_Cmd;

static uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "u8g2_byte_cb(");
  CONS_UART_Write(',');
  Format_UnsDec(CONS_UART_Write, (uint16_t)msg);
  CONS_UART_Write(',');
  Format_UnsDec(CONS_UART_Write, (uint16_t)arg_int);
  Format_String(CONS_UART_Write, ") ");
#endif
  switch(msg)
  { case U8X8_MSG_BYTE_SET_DC:
      break;
    case U8X8_MSG_BYTE_INIT:
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      { U8G2_Cmd=i2c_cmd_link_create();
        uint8_t Addr = u8x8_GetI2CAddress(u8x8);
#ifdef DEBUG_PRINT
        Format_Hex(CONS_UART_Write, Addr);
#endif
        i2c_master_start(U8G2_Cmd);
        i2c_master_write_byte(U8G2_Cmd, (Addr<<1) | I2C_MASTER_WRITE, true);
        break; }
    case U8X8_MSG_BYTE_SEND:
      // { i2c_master_write(U8G2_Cmd, (uint8_t *)arg_ptr, arg_int, true); break; }
      { uint8_t* Data = (uint8_t *)arg_ptr;
        for( int Idx=0; Idx<arg_int; Idx++)
        {
#ifdef DEBUG_PRINT
          Format_Hex(CONS_UART_Write, Data[Idx]);
#endif
          i2c_master_write_byte(U8G2_Cmd, Data[Idx], true); }
        break; }
    case U8X8_MSG_BYTE_END_TRANSFER:
      { i2c_master_stop(U8G2_Cmd);
        xSemaphoreTake(I2C_Mutex, portMAX_DELAY);
        i2c_master_cmd_begin(I2C_BUS, U8G2_Cmd, 10);
        xSemaphoreGive(I2C_Mutex);
        i2c_cmd_link_delete(U8G2_Cmd);
        break; }
  }
#ifdef DEBUG_PRINT
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  return 0; }

static uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "u8g2_gpio_cb(");
  CONS_UART_Write(',');
  Format_UnsDec(CONS_UART_Write, (uint16_t)msg);
  CONS_UART_Write(',');
  Format_UnsDec(CONS_UART_Write, (uint16_t)arg_int);
  Format_String(CONS_UART_Write, ")\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  switch(msg)
  { case U8X8_MSG_GPIO_AND_DELAY_INIT: break;
    case U8X8_MSG_GPIO_RESET:
    {
#ifdef PIN_OLED_RST
      gpio_set_level(PIN_OLED_RST, arg_int);
#endif
      break; }
    case U8X8_MSG_GPIO_CS:             break;
    case U8X8_MSG_GPIO_I2C_CLOCK:      break;
    case U8X8_MSG_GPIO_I2C_DATA:       break;
    case U8X8_MSG_DELAY_MILLI: vTaskDelay(arg_int); break;
  }
  return 0; }

u8g2_t U8G2_OLED;

void U8G2_Init(void)
{
#ifdef WITH_U8G2_SH1106
  u8g2_Setup_sh1106_i2c_128x64_noname_f(&U8G2_OLED, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
#else
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&U8G2_OLED, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
#endif
  u8x8_SetI2CAddress(&U8G2_OLED.u8x8, OLED_I2C_ADDR);
  u8g2_InitDisplay(&U8G2_OLED);
#ifdef WITH_U8G2_FLIP
  u8g2_SetDisplayRotation(&U8G2_OLED, U8G2_R2);       // flip the display
#endif
  u8g2_SetPowerSave(&U8G2_OLED, 0);

  // u8g2_ClearBuffer(&U8G2_OLED);
  // U8G2_DrawLogo(&U8G2_OLED);
  // u8g2_SendBuffer(&U8G2_OLED);
}

#endif

//--------------------------------------------------------------------------------------------------------
// SD card in SPI mode

#ifdef WITH_SD

static sdmmc_host_t        SD_Host = SDSPI_HOST_DEFAULT();
// static spi_bus_config_t    SD_BusConfig = {
//         .mosi_io_num = PIN_SD_MOSI,
//         .miso_io_num = PIN_SD_MISO,
//         .sclk_io_num = PIN_SD_SCK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 4000,
//     } ;
// static sdspi_device_config_t SD_SlotConfig = SDSPI_DEVICE_CONFIG_DEFAULT();
static sdspi_slot_config_t SD_SlotConfig;

static esp_vfs_fat_sdmmc_mount_config_t SD_MountConfig =
  { .format_if_mount_failed = false,
    .max_files = 5,
    /* .allocation_unit_size = 16 * 1024 */ };

sdmmc_card_t *SD_Card = 0;

bool      SD_isMounted(void) { return SD_Card; }
int       SD_getSectors(void) { return SD_Card->csd.capacity; }
int       SD_getSectorSize(void) { return SD_Card->csd.sector_size; }

void SD_Unmount(void)
{ esp_vfs_fat_sdmmc_unmount(); SD_Card=0; }

esp_err_t SD_Mount(void)
{ esp_err_t Ret = esp_vfs_fat_sdmmc_mount("/sdcard", &SD_Host, &SD_SlotConfig, &SD_MountConfig, &SD_Card); // ESP_OK => good, ESP_FAIL => failed to mound the file system, other => HW not working
  if(Ret!=ESP_OK) SD_Unmount();
  return Ret; } // ESP_OK => all good, ESP_FAIL => failed to mount file system, other => failed to init. the SD card

static esp_err_t SD_Init(void)
{
  // Host = SDSPI_HOST_DEFAULT();
  // Host.max_freq_khz = SDMMC_FREQ_PROBING;
  // esp_err_t Ret = spi_bus_initialize((spi_host_device_t)SD_Host.slot, &SD_BusConfig, SD_SPI_DMA);
  SD_SlotConfig = SDSPI_SLOT_CONFIG_DEFAULT();
  SD_SlotConfig.gpio_miso = PIN_SD_MISO;
  SD_SlotConfig.gpio_mosi = PIN_SD_MOSI;
  SD_SlotConfig.gpio_sck  = PIN_SD_SCK;
  SD_SlotConfig.gpio_cs = PIN_SD_CS;
  // SD_SlotConfig.host_id = (spi_host_device_t)SD_Host.slot;
  SD_SlotConfig.dma_channel = SD_SPI_DMA; // otherwise it conflicts with RFM SPI or LCD SPI
  return SD_Mount(); } // ESP_OK => all good, ESP_FAIL => failed to mount file system, other => failed to init. the SD card

#endif // WITH_SD

//--------------------------------------------------------------------------------------------------------

volatile uint8_t LED_PCB_Counter = 0;
void LED_PCB_Flash(uint8_t Time) { if(Time>LED_PCB_Counter) LED_PCB_Counter=Time; } // [ms]

#ifdef WITH_LED_TX
volatile uint8_t LED_TX_Counter = 0;
void LED_TX_Flash(uint8_t Time) { if(Time>LED_TX_Counter) LED_TX_Counter=Time; } // [ms]
#endif

#ifdef WITH_LED_RX
volatile uint8_t LED_RX_Counter = 0;
void LED_RX_Flash(uint8_t Time) { if(Time>LED_RX_Counter) LED_RX_Counter=Time; } // [ms]
#endif

void LED_TimerCheck(uint8_t Ticks)
{ uint8_t Counter=LED_PCB_Counter;
  if(Counter)
  { if(Ticks<Counter) Counter-=Ticks;
                 else Counter =0;
    if(Counter) LED_PCB_On();
           else LED_PCB_Off();
    LED_PCB_Counter=Counter; }
#ifdef WITH_LED_TX
  Counter=LED_TX_Counter;
  if(Counter)
  { if(Ticks<Counter) Counter-=Ticks;
                 else Counter =0;
    if(Counter) LED_TX_On();
           else LED_TX_Off();
    LED_TX_Counter=Counter; }
#endif
#ifdef WITH_LED_RX
  Counter=LED_RX_Counter;
  if(Counter)
  { if(Ticks<Counter) Counter-=Ticks;
                 else Counter =0;
    if(Counter) LED_RX_On();
           else LED_RX_Off();
    LED_RX_Counter=Counter; }
#endif
}

// bool Button_SleepRequest = 0;

#ifdef WITH_SLEEP
static bool SleepPending = 0;
extern void SleepIn(void);
extern void SleepOut(void);

void Sleep(void)
{ Format_String(CONS_UART_Write, "Sleep...\n");
#ifdef WITH_LONGPRESS_SLEEP
  gpio_wakeup_enable(PIN_BUTTON, GPIO_INTR_LOW_LEVEL); // _NEGEDGE ?
#endif
  esp_sleep_enable_gpio_wakeup();
  vTaskDelay(100);
  esp_light_sleep_start();
  gpio_wakeup_enable(PIN_BUTTON, GPIO_INTR_DISABLE);
  Format_String(CONS_UART_Write, "Wake up !\n"); }

#endif

static uint32_t Button_PressTime=0;                              // [ms] counts for how long the button is kept pressed
static uint32_t Button_ReleaseTime=0;

const  int8_t Button_FilterTime  = 20;                    // [ms] anti-glitch filter width
const  int8_t Button_FilterThres = Button_FilterTime/2;
static int8_t Button_Filter=(-Button_FilterTime);

// void Sleep(void)
// {
// #ifdef PIN_PERIPH_RST
//   gpio_set_level(PIN_PERIPH_RST, 0);
// #endif
// #ifdef PIN_GPS_ENA
//   gpio_set_level(PIN_GPS_ENA, 0);
// #endif
//   esp_light_sleep_start();
//   Button_SleepRequest = 0;
//   Button_PressTime=0;
//  #ifdef PIN_PERIPH_RST
//    gpio_set_level(PIN_PERIPH_RST, 0);
//
//    gpio_set_level(PIN_PERIPH_RST, 1);
//  #endif
// }


static uint32_t Button_keptPressed(uint8_t Ticks)
{ uint32_t ReleaseTime=0;
  Button_PressTime+=Ticks;                                  // count for how long the button is kept pressed
  // Button_SleepRequest = Button_PressTime>=30000;           // [ms] setup SleepRequest if button pressed for >= 4sec
#ifdef WITH_LONGPRESS_SLEEP
   if(!SleepPending && Button_PressTime>=4000)
   { SleepIn(); SleepPending=1; }
#endif
  if(Button_ReleaseTime)                                    // if release-time counter non-zero
  { // Format_String(CONS_UART_Write, "Button pressed: released for ");
    // Format_UnsDec(CONS_UART_Write, Button_ReleaseTime, 4, 3);
    // Format_String(CONS_UART_Write, "sec\n");
    ReleaseTime=Button_ReleaseTime;                         // then return the release time
    Button_ReleaseTime=0; }
  return ReleaseTime; }  // [ms] when button was pressed, return the release time

static uint32_t Button_keptReleased(uint8_t Ticks)
{ uint32_t PressTime=0;
  Button_ReleaseTime+=Ticks;                                // count release time
  if(Button_PressTime)                                      // if pressed-time non-zero
  { // Format_String(CONS_UART_Write, "Button released: pressed for ");
    // Format_UnsDec(CONS_UART_Write, Button_PressTime, 4, 3);
    // Format_String(CONS_UART_Write, "sec\n");
    // if(Button_SleepRequest)
    // { Format_String(CONS_UART_Write, "Sleep in 2 sec\n");
    //   vTaskDelay(2000);
    //   Sleep(); }
    PressTime=Button_PressTime;                             // return the pressed-time
#ifdef WITH_SLEEP
    if(SleepPending)
    { Sleep();
      SleepOut();
      SleepPending=0; }
#endif
    Button_PressTime=0;
  }
  return PressTime; }  // [ms] when button is released, return the press time

int32_t Button_TimerCheck(uint8_t Ticks)
{ int32_t PressReleaseTime=0;
#ifdef PIN_BUTTON
 // CONS_UART_Write(Button_isPressed()?'^':'_');
 if(Button_isPressed())
 { Button_Filter+=Ticks; if(Button_Filter>Button_FilterTime) Button_Filter=Button_FilterTime; // increment and saturate the counter
   if(Button_Filter>=Button_FilterThres)                             // if above the threshold
   { uint32_t ReleaseTime=Button_keptPressed(Ticks);                 // count for how long it is being kept pressed
     if(ReleaseTime) PressReleaseTime=(-(int32_t)ReleaseTime);; }
 }
 else
 { Button_Filter-=Ticks; if(Button_Filter<(-Button_FilterTime)) Button_Filter=(-Button_FilterTime); // decrement and saturate
   if(Button_Filter<=(-Button_FilterThres))                          // if below the threshold
   { uint32_t PressTime=Button_keptReleased(Ticks);                  // count for how long it is being kept released
     if(PressTime) PressReleaseTime=PressTime; }
 }
#endif
  return PressReleaseTime; } // [ms] return press (positive) or release (negative) button times

/*
extern "C"
void vApplicationIdleHook(void) // when RTOS is idle: should call "sleep until an interrupt"
{ // __WFI();                      // wait-for-interrupt
}

extern "C"
void vApplicationTickHook(void) // RTOS timer tick hook
{ LED_TimerCheck();
}
*/

//--------------------------------------------------------------------------------------------------------
// AXP192

#ifdef WITH_AXP
AXP192 AXP;
#endif

#ifdef WITH_BQ
BQ24295 BQ;
#endif

//--------------------------------------------------------------------------------------------------------
// ADC

static esp_adc_cal_characteristics_t *ADC_characs =
        (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
#ifdef WITH_TBEAM
static adc1_channel_t ADC_Chan_Batt = ADC1_GPIO35_CHANNEL;
static adc1_channel_t ADC_Chan_Knob = ADC1_GPIO34_CHANNEL;
#else
static adc1_channel_t ADC_Chan_Batt = ADC1_GPIO36_CHANNEL;
#endif
static const adc_atten_t ADC_atten = ADC_ATTEN_DB_11;
static const adc_unit_t ADC_unit = ADC_UNIT_1;
#define ADC_Vref 1100

static int ADC_Init(void)
{ // if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) // Check TP is burned into eFuse
  // if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) // Check Vref is burned into eFuse
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_Chan_Batt, ADC_atten);
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_unit, ADC_atten, ADC_WIDTH_BIT_12, ADC_Vref, ADC_characs); // calibrate ADC1
  return 0; }

#ifdef WITH_AXP
uint16_t BatterySense(int Samples)
{ return AXP.readBatteryVoltage(); } // [mV]
#else
uint16_t BatterySense(int Samples)
{ uint32_t RawVoltage=0;
  for( int Idx=0; Idx<Samples; Idx++)
  { RawVoltage += adc1_get_raw(ADC_Chan_Batt); }
  RawVoltage = (RawVoltage+Samples/2)/Samples;
  uint16_t Volt = (uint16_t)esp_adc_cal_raw_to_voltage(RawVoltage, ADC_characs)*2;
  // const uint16_t Bias = 80;  // apparently, there is 80mV bias in the battery voltage measurement
  // if(Volt>=Bias) Volt-=Bias;
  return Volt; } // [mV]
#endif

#ifdef WITH_TBEAM
uint16_t KnobSense(int Samples)
{ uint16_t RawVoltage=0;
  for( int Idx=0; Idx<Samples; Idx++)
  { RawVoltage += adc1_get_raw(ADC_Chan_Knob); }
  RawVoltage = (RawVoltage+Samples/2)/Samples;
  return RawVoltage; }
#endif

//--------------------------------------------------------------------------------------------------------

void IO_Configuration(void)
{

#ifdef PIN_LED_PCB
  LED_PCB_Dir();                    // PCB LED
  LED_PCB_Off();
#endif
#ifdef WITH_LED_TX
  LED_TX_Dir();
  LED_TX_Off();
#endif

#ifdef PIN_BUTTON
  Button_Dir();                     // Push-button
#endif

#ifdef WITH_JACEK
  POWER_LDO_Dir();                  // speific power control
  POWER_Dir();
  POWER_LDO_On();
  POWER_On();
#endif

#ifdef WITH_M5_JACEK
  GPS_ANT_Dir();
  GPS_ANT_Sel(1);                   // 0 = external entenna
#endif

#ifdef PIN_PERIPH_RST
  gpio_set_direction(PIN_PERIPH_RST, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_PERIPH_RST, 1);
#endif

#ifdef PIN_GPS_PPS
  gpio_set_direction(PIN_GPS_PPS, GPIO_MODE_INPUT);
#endif
#ifdef PIN_GPS_RST
  gpio_set_direction(PIN_GPS_RST, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_GPS_RST, 0);
#endif
#ifdef PIN_GPS_ENA
  gpio_set_direction(PIN_GPS_ENA, GPIO_MODE_OUTPUT);
  GPS_ENABLE();
// #ifdef WITH_GPS_MTK
//   gpio_set_level(PIN_GPS_ENA, 0);                       //
// #else
//   gpio_set_level(PIN_GPS_ENA, 1);                       //
// #endif
#endif

#ifdef GPS_UART
  uart_config_t GPS_UART_Config =                       // GPS UART
  { baud_rate : 4800,
    data_bits : UART_DATA_8_BITS,
    parity    : UART_PARITY_DISABLE,
    stop_bits : UART_STOP_BITS_1,
    flow_ctrl : UART_HW_FLOWCTRL_DISABLE,
    rx_flow_ctrl_thresh: 0,
    use_ref_tick: 0
  };
  uart_param_config  (GPS_UART, &GPS_UART_Config);
  uart_set_pin       (GPS_UART, PIN_GPS_TXD, PIN_GPS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(GPS_UART, 256, 256, 0, 0, 0);
#endif

#ifdef AERO_UART
  uart_config_t AERO_UART_Config =                      // AERO UART
  { baud_rate : 115200,
    data_bits : UART_DATA_8_BITS,
    parity    : UART_PARITY_DISABLE,
    stop_bits : UART_STOP_BITS_1,
    flow_ctrl : UART_HW_FLOWCTRL_DISABLE,
    rx_flow_ctrl_thresh: 0,
    use_ref_tick: 0
  };
  uart_param_config  (AERO_UART, &AERO_UART_Config);
  uart_set_pin       (AERO_UART, PIN_AERO_TXD, PIN_AERO_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(AERO_UART, 256, 256, 0, 0, 0);
#endif

#if defined(WITH_OLED) || defined(WITH_U8G2_OLED)
#ifdef PIN_OLED_RST
  gpio_set_direction(PIN_OLED_RST, GPIO_MODE_OUTPUT);
#endif
#endif

#if defined(PIN_I2C_SCL) && defined(PIN_I2C_SDA)
  i2c_config_t I2C_Config =                            // I2C for OLED and pressue sensor
  { mode          : I2C_MODE_MASTER,
    sda_io_num    : PIN_I2C_SDA,
    sda_pullup_en : GPIO_PULLUP_ENABLE,
    scl_io_num    : PIN_I2C_SCL,
    scl_pullup_en : GPIO_PULLUP_ENABLE
  } ;
  I2C_Config.master.clk_speed =  I2C_SPEED;
  i2c_param_config  (I2C_BUS, &I2C_Config);
  i2c_driver_install(I2C_BUS, I2C_Config.mode, 0, 0, 0);
#endif

#ifdef WITH_BQ
  gpio_set_direction(PIN_POWER_GOOD, GPIO_MODE_INPUT);
  BQ.Bus=(uint8_t)I2C_BUS;
  BQ.checkID();
  // BQ.writeSource(0x05);     // Reg #00
  // BQ.writePowerON(0x2F);    // Reg #01, disable charging
  BQ.writeChargeCurr(0x20); // Reg #02 00 = 512mA, 0x20 = 512+512mA
  BQ.writePreCharge(0x00);  // Reg #03
  BQ.writeChargeVolt(0x9A); // Reg #04
  // BQ.writePowerON(0x3F);    // Reg #01, enable charging
  BQ.writeTimerCtrl(0x8C);  // Reg #05, disable the watchdog, so it always stays in host-mode, not default-mode
  // BQ.writeTimerCtrl(0x9C);  // REG #05, enable the watchdog
#endif

#ifdef WITH_AXP
  gpio_set_direction(PIN_AXP_IRQ, GPIO_MODE_INPUT);
  AXP.Bus=(uint8_t)I2C_BUS;
  AXP.checkID();
  AXP.setPOK(0xDC);                      // power-on = 11 = 1sec, long-press = 01 = 1.5sec, power-off = enable, PWROK delay = 64ms, power-off = 00 = 4sec
  uint8_t PwrStatus = AXP.readStatus();
  AXP.setLED(1);
  AXP.setPowerOutput(AXP.OUT_DCDC1, 1);  // 3.3V on the pin header for LCD and BME280
  AXP.setDCDC1(3300);
  // vTaskDelay(100);
  AXP.setPowerOutput(AXP.OUT_LDO2,  1);  // RFM power
  // vTaskDelay(100);
  AXP.setPowerOutput(AXP.OUT_LDO3,  1);  // GPS power
  // vTaskDelay(100);
  AXP.enableBatMon();                    // enable battery charge/discharge current counters
  AXP.enableADC(AXP.ADC_BAT_VOLT | AXP.ADC_BAT_CURR | AXP.ADC_VBUS_VOLT | AXP.ADC_VBUS_CURR);
  // AXP.SetPowerOutput(AXP.OUT_DCDC2); // not used on T-Beam
  // AXP.SetPowerOutput(AXP.OUT_DCDC3); // DCDC3 is a feedback from VDD3V3 ?
  // AXP.SetPowerOutput(AXP.OUT_EXTEN); // seems only wired to a test pin ?
  AXP.enableIRQ(AXP.AXP202_PEK_LONGPRESS_IRQ | AXP.AXP202_PEK_SHORTPRESS_IRQ, 1);
  AXP.clearIRQ();
  // vTaskDelay(500);
#endif

#ifdef WITH_OLED
#ifdef PIN_OLED_RST
  OLED_RESET(0);
  vTaskDelay(10);
  OLED_RESET(1);
#endif
  vTaskDelay(10);
  OLED_Init(0);
  OLED_Clear(0);
  OLED_SetContrast(128, 0);
#ifdef WITH_OLED2
  OLED_Init(1);
  OLED_Clear(1);
  OLED_SetContrast(128, 1);
#endif
#endif

#ifdef WITH_U8G2_OLED
  U8G2_Init();
#endif

  RFM_IRQ_SetInput();
  RFM_RESET_SetOutput();
  RFM_RESET(0);

  spi_bus_config_t BusCfg =                               // RF/LCD chip SPI
  { mosi_io_num     : PIN_RFM_MOSI,
    miso_io_num     : PIN_RFM_MISO,
    sclk_io_num     : PIN_RFM_SCK,
    quadwp_io_num   : -1,
    quadhd_io_num   : -1,
#ifdef WITH_ILI9341             // M5stack configuration
    max_transfer_sz : LCD_BUFF_SIZE*2+8,
#else
    max_transfer_sz : 64,
#endif
    flags           : SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI,
    intr_flags      : 0 // ESP_INTR_FLAG_SHARED  ESP_INTR_FLAG_INTRDISABLED
  };

  spi_device_interface_config_t DevCfg =
  { command_bits     : 0,
    address_bits     : 0,
    dummy_bits       : 0,
    mode             : 0,
    duty_cycle_pos   : 0,
    cs_ena_pretrans  : 0,
    cs_ena_posttrans : 0,
    clock_speed_hz   : RFM_SPI_SPEED,
    input_delay_ns   : 0,
    spics_io_num     : PIN_RFM_SS,
    flags            : 0,
    queue_size       : 3,
    pre_cb           : 0,
    post_cb          : 0
  };
  esp_err_t ret=spi_bus_initialize(RFM_SPI_HOST, &BusCfg,  RFM_SPI_DMA); // RFM/LCD SPI bus
            ret=spi_bus_add_device(RFM_SPI_HOST, &DevCfg, &RFM_SPI);     // RFM SPI device

#ifdef WITH_TFT_LCD
  TFT_LCD_Init();
#endif

#ifdef WITH_ST7789

  spi_bus_config_t BusConfig =    // SPI bus setup
  {
    .mosi_io_num = LCD_PIN_MOSI,
    .miso_io_num = LCD_PIN_MISO,
    .sclk_io_num = LCD_PIN_CLK,
    .quadwp_io_num = GPIO_NUM_NC,
    .quadhd_io_num = GPIO_NUM_NC,
    .max_transfer_sz = LCD_BUFF_SIZE*2+8,
    .flags = 0,
    .intr_flags = 0
  };

  ret = spi_bus_initialize(LCD_SPI_HOST, &BusConfig, LCD_SPI_DMA);  // Initialize the SPI bus

  LCD_WIDTH=240; LCD_HEIGHT=240;
  LCD_TYPE=0;
#ifdef WITH_TBEAM_V10
  LCD_PIN_RST = GPIO_NUM_33;
  LCD_PIN_DC  = GPIO_NUM_2;
  LCD_PIN_BCKL = GPIO_NUM_32; // on one baord it is 15
#endif
  LCD_Init(LCD_SPI_HOST, LCD_SPI_MODE, LCD_SPI_SPEED);
  LCD_Start();
#endif

#ifdef WITH_ILI9341             // M5stack configuration
#define LCD_SPI_SPEED 26000000     // [Hz]
#define LCD_SPI_MODE         0     //
  LCD_PIN_CS   = GPIO_NUM_14;   //
  LCD_PIN_DC   = GPIO_NUM_27;   // D/C: Command = 0, Data = 1
  LCD_PIN_RST  = GPIO_NUM_33;   //
  LCD_PIN_BCKL = GPIO_NUM_32;   // back-light: HIGH active
  LCD_WIDTH=320; LCD_HEIGHT=240;
  LCD_TYPE = 1;
  LCD_Init(RFM_SPI_HOST, LCD_SPI_MODE, LCD_SPI_SPEED);
  LCD_Start();
#endif

#ifdef WITH_SD
  SD_Init();
#endif

#ifdef WITH_BEEPER
  Beep_Init();
#endif

#ifdef WITH_SOUND
  Sound_Init();
  // Sound_PlayU8(Sound_737_Traffic_u8, Sound_737_Traffic_size);
  // Sound_PlayU8(Sound_737_Whoop_Whoop_u8, Sound_737_Whoop_Whoop_size);
  // Sound_PlayU8(Sound_737_Sink_Rate_u8, Sound_737_Sink_Rate_size);
  // for( uint16_t Freq=0x400; Freq<=0x1000; Freq<<=1)
  // { Sound_Beep(Freq, 4000, 0x2000); }
#endif

  ADC_Init();

  // esp_register_freertos_tick_hook(&vApplicationTickHook);

}

// ======================================================================================================

// ======================================================================================================

int NVS_Init(void)
{ esp_err_t Err = nvs_flash_init();
  if (Err == ESP_ERR_NVS_NO_FREE_PAGES)
  { nvs_flash_erase();
    Err = nvs_flash_init(); }

  // if(Parameters.ReadFromNVS()!=ESP_OK)
  // { Parameters.setDefault(getUniqueID());
  //   Parameters.WriteToNVS(); }

  return Err; }

// ======================================================================================================

#ifdef WITH_SPIFFS
int SPIFFS_Register(const char *Path, const char *Label, size_t MaxOpenFiles)
{ esp_vfs_spiffs_conf_t FSconf =
  { base_path: Path,
    partition_label: Label,
    max_files: MaxOpenFiles,
    format_if_mount_failed: true };
  return esp_vfs_spiffs_register(&FSconf); }

int SPIFFS_Info(size_t &Total, size_t &Used, const char *Label)
{ return esp_spiffs_info(Label, &Total, &Used); }
#endif

// ======================================================================================================

SemaphoreHandle_t I2C_Mutex;

uint8_t I2C_Read(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait)
{ i2c_cmd_handle_t Cmd = i2c_cmd_link_create();
  i2c_master_start(Cmd);
  i2c_master_write_byte(Cmd, (Addr<<1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
  i2c_master_write_byte(Cmd, Reg, I2C_MASTER_ACK);
  i2c_master_start(Cmd);
  i2c_master_write_byte(Cmd, (Addr<<1) | I2C_MASTER_READ, I2C_MASTER_ACK);
  i2c_master_read(Cmd, Data, Len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(Cmd);
  xSemaphoreTake(I2C_Mutex, portMAX_DELAY);
  esp_err_t Ret = i2c_master_cmd_begin((i2c_port_t)Bus, Cmd, Wait);
  xSemaphoreGive(I2C_Mutex);
  i2c_cmd_link_delete(Cmd);
  return Ret; }

uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait)
{ i2c_cmd_handle_t Cmd = i2c_cmd_link_create();
  i2c_master_start(Cmd);
  i2c_master_write_byte(Cmd, (Addr<<1) | I2C_MASTER_WRITE , I2C_MASTER_ACK);
  i2c_master_write_byte(Cmd, Reg , I2C_MASTER_ACK);
  i2c_master_write(Cmd, Data, Len, I2C_MASTER_NACK);
  i2c_master_stop(Cmd);
  xSemaphoreTake(I2C_Mutex, portMAX_DELAY);
  esp_err_t Ret = i2c_master_cmd_begin((i2c_port_t)Bus, Cmd, Wait);
  xSemaphoreGive(I2C_Mutex);
  i2c_cmd_link_delete(Cmd);
  return Ret; }

uint8_t I2C_Restart(uint8_t Bus)
{ return 0; }

// ======================================================================================================

