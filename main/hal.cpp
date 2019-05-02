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

#include "nvs.h"
#include "nvs_flash.h"

#ifdef WITH_SPIFFS
#include "esp_spiffs.h"
#endif

#ifdef WITH_SD
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
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

#ifdef WITH_OLED
#include "ssd1306.h"
#include "font8x8_basic.h"
#endif

#ifdef WITH_U8G2
#include "u8g2.h"
#endif

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

T-Beam board pinout:

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

GPIO   HELTEC      TTGO       JACEK      T-Beam      FollowMe   Restrictions

 0                Button
 1    CONS/TxD    CONS/TxD   CONS/TxD   CONS/TxD                Console/Program
 2                           SD/MISO    .            LED        Bootstrap: LOW to enter UART download mode
 3    CONS/RxD    CONS/RxD   CONS/RxD   CONS/RxD                Console/Program
 4    OLED/SDA    OLED/SDA   ADC/CS     Beeper       PER/RST
 5    RF/SCK      RF/SCK     RF/SCK     RF/SCK       RF/CS
 6                                                              SD/CLK
 7                                                              SD/DATA0
 8                                                              SD/DATA1
 9                                                              SD/DATA2
10                                                              SD/DATA3
11                                                              SD/CMD
12    GPS/RxD     GPS/RxD    SD/CS      GPS/RxD      SD/MISO    JTAG/TDI Bootstrap: select output voltage to power the flash chip
13    GPS/Ena     GPS/Ena    SD/SCK                  SD/MOSI    JTAG/TCK
14    RF/RST      RF/RST     Beeper     LED          SD/CLK     JTAG/TMS
15    OLED/SCL    OLED/SCL   SD/MOSI    GPS/TxD      SD/CS      JTAG/TDO
16    OLED/RST    OLED/RST   RF/IRQ                  GPS/Tx
17    Beeper      Beeper     RF/RST                  GPS/Rx
18    RF/CS       RF/CS      RF/MISO    RF/CS        RF/SCK
19    RF/MISO     RF/MISO    RF/MOSI    RF/MISO      RF/MISO
20
21                LED        RF/CS      I2C/SDA      I2C/SDA
22                           PWR/ON     I2C/SCL      I2C/CLK
23                           PWR/LDO    RF/RST       RF/MOSI
24
25    LED         DAC2       .          .            TT/RX0
26    RF/IRQ      RF/IRQ     SCL        RF/IRQ       BMX/INT1
27    RF/MOSI     RF/MOSI    SDA                     TT/TX0
28
29
30
31
32                           GPS/TxD    .            TT/BOOT
33                           OLED/RST   .            GPS/WAKE
34    GPS/PPS     GPS/PPS    GPS/RxD                 GPS/PPS
35    GPS/TxD     GPS/TxD    GPS/PPS    BAT/Sense    RF/IRQ
36                           BAT/Sense               BAT/Sense
37
38
39                                                   Button

*/


#ifdef WITH_TTGO
#define PIN_LED_PCB  GPIO_NUM_2   // status LED on the PCB:  2, GPIO25 is DAC2
#endif
#ifdef WITH_HELTEC
#define PIN_LED_PCB  GPIO_NUM_25  // status LED on the PCB: 25, GPIO25 is DAC2
#endif
#ifdef WITH_TBEAM
#define PIN_LED_PCB  GPIO_NUM_14  // status LED on the PCB: 14, posisbly inverted
#define PIN_LED_PCB_INV
#endif
#ifdef WITH_FollowMe
#define PIN_LED_PCB  GPIO_NUM_2   // debug LED
#define PIN_LED_TX   GPIO_NUM_15
#endif

// #define PIN_LED_RX   GPIO_NUM_??

#if defined(WITH_HELTEC) || defined(WITH_TTGO)
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

#ifdef WITH_FollowMe
// #define PIN_RFM_RST  GPIO_NUM_32  // Reset
#define PIN_RFM_IRQ  GPIO_NUM_35  // 39 // packet done on receive or transmit
#define PIN_RFM_SS   GPIO_NUM_5   // SPI chip-select
#define PIN_RFM_SCK  GPIO_NUM_18  // SPI clock
#define PIN_RFM_MISO GPIO_NUM_19  // SPI MISO
#define PIN_RFM_MOSI GPIO_NUM_23  // SPI MOSI
#endif // FollowMe

#define RFM_SPI_HOST  VSPI_HOST   // or HSPI_HOST
#define RFM_SPI_SPEED 4000000     // [Hz] 4MHz SPI clock rate for RF chip

#if defined(WITH_HELTEC) || defined(WITH_TTGO)
                                  // VK2828U   GN-801   MAVlink
#define PIN_GPS_TXD  GPIO_NUM_12  // green     green    green
#define PIN_GPS_RXD  GPIO_NUM_35  // blue      yellow   yellow
#define PIN_GPS_PPS  GPIO_NUM_34  // white     blue
#define PIN_GPS_ENA  GPIO_NUM_13  // yellow    white
#endif // HELTEC || TTGO

// Note: I had a problem with GPS ENABLE on GPIO13, thus I tied the enable wire to 3.3V for the time being.

#ifdef WITH_TBEAM
#define PIN_GPS_TXD  GPIO_NUM_15  // UBX GPS with only UART
#define PIN_GPS_RXD  GPIO_NUM_12
#endif

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
#define ADSB_UART     UART_NUM_2  // UART2
#define PIN_ADSB_TXD  GPIO_NUM_25
#define PIN_ADSB_RXD  GPIO_NUM_27
#endif

#if defined(WITH_HELTEC) || defined(WITH_TTGO)
#define PIN_I2C_SCL GPIO_NUM_15   // SCL pin
#define PIN_I2C_SDA GPIO_NUM_4    // SDA pin
#define OLED_I2C_ADDR 0x3C        // I2C address of the OLED display
#define PIN_OLED_RST GPIO_NUM_16  // OLED RESET: low-active
#endif

#ifdef WITH_TBEAM                 // T-Beam
#define PIN_I2C_SCL GPIO_NUM_22   // SCL pin => this way the pin pattern fits the BMP280 module
#define PIN_I2C_SDA GPIO_NUM_21   // SDA pin
#define OLED_I2C_ADDR 0x3C        // I2C address of the OLED display
#endif

#ifdef WITH_FollowMe              //
#define PIN_I2C_SCL GPIO_NUM_22   // SCL pin
#define PIN_I2C_SDA GPIO_NUM_21   // SDA pin
#define OLED_I2C_ADDR 0x3C        // I2C address of the OLED display
// #define PIN_OLED_RST GPIO_NUM_15  // OLED RESET: low-active
#endif

uint8_t BARO_I2C = (uint8_t)I2C_BUS;

#ifdef WITH_TBEAM
#define PIN_BEEPER    GPIO_NUM_4
#endif

#if defined(WITH_HELTEC) || defined(WITH_TTGO)
#define PIN_BEEPER    GPIO_NUM_17
#endif

#if !defined(WITH_OLED) && !defined(WITH_U8G2) && !defined(WITH_BMP180) && !defined(WITH_BMP280) && !defined(WITH_BME280)
#undef PIN_I2C_SCL
#undef PIN_I2C_SDA
#endif

#define PIN_SD_MISO   GPIO_NUM_12 // SD card in simple SPI mode, using HSPI IOMUX pins
#define PIN_SD_MOSI   GPIO_NUM_13
#define PIN_SD_SCK    GPIO_NUM_14
#define PIN_SD_CS     GPIO_NUM_15

#ifdef WITH_FollowMe
#define PIN_BUTTON    GPIO_NUM_39
#else
#define PIN_BUTTON    GPIO_NUM_0
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
#else
void LED_PCB_Dir  (void) { }
void LED_PCB_On   (void) { }
void LED_PCB_Off  (void) { }
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
      ormat_String(CONS_UART_Write, "]\n");
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

//--------------------------------------------------------------------------------------------------------
// ADS-B UART

#ifdef ADSB_UART
int   ADSB_UART_Read       (uint8_t &Byte) { return uart_read_bytes  (ADSB_UART, &Byte, 1, 0); }  // should be buffered and non-blocking
void  ADSB_UART_Write      (char     Byte) {        uart_write_bytes (ADSB_UART, &Byte, 1);    }  // should be buffered and blocking
void  ADSB_UART_SetBaudrate(int BaudRate)  {        uart_set_baudrate(ADSB_UART, BaudRate);    }
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
inline void RFM_RESET_Dir (void)      { gpio_set_direction(PIN_RFM_RST, GPIO_MODE_OUTPUT); }
inline void RFM_RESET_Set (bool High) { gpio_set_level(PIN_RFM_RST, High); }

#ifdef WITH_RFM95       // for RFM95 reset is low-active
void RFM_RESET(uint8_t On) { RFM_RESET_Set(~On); }
#endif

#ifdef WITH_RFM69       // for RFM69 reset is high-active
void RFM_RESET(uint8_t On) { RFM_RESET_Set(On); }
#endif

#else                   // if no reset pin declared for the RF chip, then make an empty call
inline void RFM_RESET_Dir (void) { }
void RFM_RESET(uint8_t On) { }
#endif // PIN_RFM_RST

inline void RFM_IRQ_Dir (void) { gpio_set_direction(PIN_RFM_IRQ, GPIO_MODE_INPUT); }
       bool RFM_IRQ_isOn(void) { return gpio_get_level(PIN_RFM_IRQ); }

static spi_device_handle_t RFM_SPI;

void RFM_TransferBlock(uint8_t *Data, uint8_t Len)
{ spi_transaction_t Trans;
  memset(&Trans, 0, sizeof(Trans));
  Trans.tx_buffer = Data;
  Trans.rx_buffer = Data;
  Trans.length = 8*Len;
  esp_err_t ret = spi_device_transmit(RFM_SPI, &Trans); }

//--------------------------------------------------------------------------------------------------------
// BEEPER

#ifdef WITH_BEEPER

static ledc_timer_config_t LEDC_Timer =
  {
    speed_mode      : LEDC_HIGH_SPEED_MODE,   // timer mode
    { duty_resolution : LEDC_TIMER_8_BIT },   // resolution of PWM duty: 0..255
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

#ifdef WITH_U8G2

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
        i2c_master_cmd_begin(I2C_BUS, U8G2_Cmd, 10);
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

void U8G2_DrawLogo(u8g2_t *OLED)  // draw logo and hardware options in software
{
  u8g2_DrawCircle(OLED, 96, 32, 30, U8G2_DRAW_ALL);
  u8g2_DrawCircle(OLED, 96, 32, 34, U8G2_DRAW_UPPER_RIGHT);
  u8g2_DrawCircle(OLED, 96, 32, 38, U8G2_DRAW_UPPER_RIGHT);
  // u8g2_SetFont(OLED, u8g2_font_open_iconic_all_4x_t);
  // u8g2_DrawGlyph(OLED, 64, 32, 0xF0);
  u8g2_SetFont(OLED, u8g2_font_ncenB14_tr);
  u8g2_DrawStr(OLED, 74, 31, "OGN");
  u8g2_SetFont(OLED, u8g2_font_8x13_tr);
  u8g2_DrawStr(OLED, 69, 43, "Tracker");

#ifdef WITH_FollowMe
  u8g2_DrawStr(OLED, 0, 16 ,"FollowMe");
#endif
#ifdef WITH_TTGO
  u8g2_DrawStr(OLED, 0, 16 ,"TTGO");
#endif
#ifdef WITH_HELTEC
  u8g2_DrawStr(OLED, 0, 16 ,"HELTEC");
#endif
#ifdef WITH_TBEAM
  u8g2_DrawStr(OLED, 0, 16 ,"T-BEAM");
#endif

#ifdef WITH_GPS_MTK
  u8g2_DrawStr(OLED, 0, 28 ,"MTK GPS");
#endif
#ifdef WITH_GPS_UBX
  u8g2_DrawStr(OLED, 0, 28 ,"UBX GPS");
#endif
#ifdef WITH_GPS_SRF
  u8g2_DrawStr(OLED, 0, 28 ,"SRF GPS");
#endif

#ifdef WITH_RFM95
  u8g2_DrawStr(OLED, 0, 40 ,"RFM95");
#endif
#ifdef WITH_RFM69
  u8g2_DrawStr(OLED, 0, 40 ,"RFM69");
#endif

#ifdef WITH_BMP180
  u8g2_DrawStr(OLED, 0, 52 ,"BMP180");
#endif
#ifdef WITH_BMP280
  u8g2_DrawStr(OLED, 0, 52 ,"BMP280");
#endif
#ifdef WITH_BME280
  u8g2_DrawStr(OLED, 0, 52 ,"BME280");
#endif

#ifdef WITH_BT_SPP
  u8g2_DrawStr(OLED, 0, 64 ,"BT SPP");
#endif
}

void U8G2_Init(void)
{
#ifdef WITH_FollowMe
  u8g2_Setup_sh1106_i2c_128x64_noname_f(&U8G2_OLED, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
#else
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&U8G2_OLED, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
#endif
  u8x8_SetI2CAddress(&U8G2_OLED.u8x8, OLED_I2C_ADDR);
  u8g2_InitDisplay(&U8G2_OLED);
  u8g2_SetPowerSave(&U8G2_OLED, 0);
  u8g2_ClearBuffer(&U8G2_OLED);

  // u8g2_DrawBox  (&U8G2_OLED, 0, 26,  80, 6);
  // u8g2_DrawFrame(&U8G2_OLED, 0, 26, 100, 6);
  U8G2_DrawLogo(&U8G2_OLED);
  u8g2_SendBuffer(&U8G2_OLED);
}

#endif

//--------------------------------------------------------------------------------------------------------
// SD card in SPI mode

#ifdef WITH_SD

static sdmmc_host_t Host;
static sdspi_slot_config_t SlotConfig;
static esp_vfs_fat_sdmmc_mount_config_t MountConfig =
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
{ esp_err_t Ret = esp_vfs_fat_sdmmc_mount("/sdcard", &Host, &SlotConfig, &MountConfig, &SD_Card); // ESP_OK => good, ESP_FAIL => failed to mound the file system, other => HW not working
  if(Ret!=ESP_OK) SD_Unmount();
  return Ret; } // ESP_OK => all good, ESP_FAIL => failed to mount file system, other => failed to init. the SD card

static esp_err_t SD_Init(void)
{
  Host = SDSPI_HOST_DEFAULT();
  SlotConfig = SDSPI_SLOT_CONFIG_DEFAULT();
  SlotConfig.gpio_miso = PIN_SD_MISO;
  SlotConfig.gpio_mosi = PIN_SD_MOSI;
  SlotConfig.gpio_sck  = PIN_SD_SCK;
  SlotConfig.gpio_cs   = PIN_SD_CS;
  SlotConfig.dma_channel = 2;              // otherwise it conflicts with RFM SPI
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

bool Button_SleepRequest = 0;

uint32_t Button_PressTime=0;                              // [ms] counts for how long the button is kept pressed
uint32_t Button_ReleaseTime=0;

const  int8_t Button_FilterTime  = 20;                    // [ms]
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
  Button_PressTime+=Ticks;
  // Button_SleepRequest = Button_PressTime>=30000;           // [ms] setup SleepRequest if button pressed for >= 4sec
  // if(Button_PressTime>=32000)
  //   { Format_String(CONS_UART_Write, "Sleep in 2 sec\n");
  //     vTaskDelay(2000);
  //     Sleep(); }
  if(Button_ReleaseTime)
  { // Format_String(CONS_UART_Write, "Button pressed: released for ");
    // Format_UnsDec(CONS_UART_Write, Button_ReleaseTime, 4, 3);
    // Format_String(CONS_UART_Write, "sec\n");
    ReleaseTime=Button_ReleaseTime;
    Button_ReleaseTime=0; }
  return ReleaseTime; }  // [ms] when button was pressed, return the release time

static uint32_t Button_keptReleased(uint8_t Ticks)
{ uint32_t PressTime=0;
  Button_ReleaseTime+=Ticks;
  if(Button_PressTime)
  { // Format_String(CONS_UART_Write, "Button released: pressed for ");
    // Format_UnsDec(CONS_UART_Write, Button_PressTime, 4, 3);
    // Format_String(CONS_UART_Write, "sec\n");
    // if(Button_SleepRequest)
    // { Format_String(CONS_UART_Write, "Sleep in 2 sec\n");
    //   vTaskDelay(2000);
    //   Sleep(); }
    PressTime=Button_PressTime;
    Button_PressTime=0;
  }
  return PressTime; }  // [ms] when button is released, return the press time

int32_t Button_TimerCheck(uint8_t Ticks)
{ int32_t PressReleaseTime=0;
#ifdef PIN_BUTTON
 // CONS_UART_Write(Button_isPressed()?'^':'_');
 if(Button_isPressed())
 { Button_Filter+=Ticks; if(Button_Filter>Button_FilterTime) Button_Filter=Button_FilterTime;
   if(Button_Filter>=Button_FilterThres)
   { uint32_t ReleaseTime=Button_keptPressed(Ticks);
     if(ReleaseTime) PressReleaseTime=(-(int32_t)ReleaseTime);; }
 }
 else
 { Button_Filter-=Ticks; if(Button_Filter<(-Button_FilterTime)) Button_Filter=(-Button_FilterTime);
   if(Button_Filter<=(-Button_FilterThres))
   { uint32_t PressTime=Button_keptReleased(Ticks);
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
// ADC


static esp_adc_cal_characteristics_t *ADC_characs =
        (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
#ifdef WITH_TBEAM
static adc1_channel_t ADC_channel = ADC1_GPIO35_CHANNEL;
#else
static adc1_channel_t ADC_channel = ADC1_GPIO36_CHANNEL;
#endif
static const adc_atten_t ADC_atten = ADC_ATTEN_DB_11;
static const adc_unit_t ADC_unit = ADC_UNIT_1;
#define ADC_Vref 1100

static int ADC_Init(void)
{ // if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) // Check TP is burned into eFuse
  // if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) // Check Vref is burned into eFuse
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_channel, ADC_atten);
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_unit, ADC_atten, ADC_WIDTH_BIT_12, ADC_Vref, ADC_characs); // calibrate ADC1
  return 0; }

uint16_t BatterySense(int Samples)
{ uint32_t RawVoltage=0;
  for( int Idx=0; Idx<Samples; Idx++)
  { RawVoltage += adc1_get_raw(ADC_channel); }
  RawVoltage = (RawVoltage+Samples/2)/Samples;
  return (uint16_t)esp_adc_cal_raw_to_voltage(RawVoltage, ADC_characs)*2; } // [mV]


//--------------------------------------------------------------------------------------------------------

void IO_Configuration(void)
{
#ifdef PIN_LED_PCB
  LED_PCB_Dir();
  LED_PCB_Off();
#endif
#ifdef PIN_BUTTON
  Button_Dir();
#endif

  RFM_IRQ_Dir();
  RFM_RESET_Dir();
  RFM_RESET(0);

  spi_bus_config_t BusCfg =                               // RF chip SPI
  { mosi_io_num     : PIN_RFM_MOSI,
    miso_io_num     : PIN_RFM_MISO,
    sclk_io_num     : PIN_RFM_SCK,
    quadwp_io_num   : -1,
    quadhd_io_num   : -1,
    max_transfer_sz : 64,
    flags           : SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI
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
  esp_err_t ret=spi_bus_initialize(RFM_SPI_HOST, &BusCfg, 1);
  ret=spi_bus_add_device(RFM_SPI_HOST, &DevCfg, &RFM_SPI);

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
#ifdef WITH_GPS_MTK
  gpio_set_level(PIN_GPS_ENA, 0);                       //
#else
  gpio_set_level(PIN_GPS_ENA, 1);                       //
#endif
#endif

#ifdef GPS_UART
  uart_config_t GPS_UART_Config =                       // GPS UART
  { baud_rate : 9600,
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

#ifdef ADSB_UART
  uart_config_t ADSB_UART_Config =                      // ADSB UART
  { baud_rate : 115200,
    data_bits : UART_DATA_8_BITS,
    parity    : UART_PARITY_DISABLE,
    stop_bits : UART_STOP_BITS_1,
    flow_ctrl : UART_HW_FLOWCTRL_DISABLE,
    rx_flow_ctrl_thresh: 0,
    use_ref_tick: 0
  };
  uart_param_config  (ADSB_UART, &ADSB_UART_Config);
  uart_set_pin       (ADSB_UART, PIN_ADSB_TXD, PIN_ADSB_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(ADSB_UART, 256, 256, 0, 0, 0);
#endif

#if defined(WITH_OLED) || defined(WITH_U8G2)
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

#ifdef WITH_U8G2
  U8G2_Init();
#endif

#ifdef WITH_SD
  SD_Init();
#endif

#ifdef WITH_BEEPER
  Beep_Init();
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

// SemaphoreHandle_t I2C_Mutex;

uint8_t I2C_Read(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait)
{ i2c_cmd_handle_t Cmd = i2c_cmd_link_create();
  i2c_master_start(Cmd);
  i2c_master_write_byte(Cmd, (Addr<<1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
  i2c_master_write_byte(Cmd, Reg, I2C_MASTER_ACK);
  i2c_master_start(Cmd);
  i2c_master_write_byte(Cmd, (Addr<<1) | I2C_MASTER_READ, I2C_MASTER_ACK);
  i2c_master_read(Cmd, Data, Len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(Cmd);
  esp_err_t Ret = i2c_master_cmd_begin((i2c_port_t)Bus, Cmd, Wait);
  i2c_cmd_link_delete(Cmd);
  return Ret; }

uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait)
{ i2c_cmd_handle_t Cmd = i2c_cmd_link_create();
  i2c_master_start(Cmd);
  i2c_master_write_byte(Cmd, (Addr<<1) | I2C_MASTER_WRITE , I2C_MASTER_ACK);
  i2c_master_write_byte(Cmd, Reg , I2C_MASTER_ACK);
  i2c_master_write(Cmd, Data, Len, I2C_MASTER_NACK);
  i2c_master_stop(Cmd);
  esp_err_t Ret = i2c_master_cmd_begin((i2c_port_t)Bus, Cmd, Wait);
  i2c_cmd_link_delete(Cmd);
  return Ret; }

uint8_t I2C_Restart(uint8_t Bus)
{ return 0; }

// ======================================================================================================

