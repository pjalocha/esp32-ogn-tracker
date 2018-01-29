#include <string.h>
// #include <sys/select.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#include "esp_system.h"
#include "esp_freertos_hooks.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_spiffs.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "hal.h"

#ifdef WITH_OLED
#include "ssd1306.h"
#include "font8x8_basic.h"
#endif

// ======================================================================================================
/*
The HELTEC Automation board WiFi LoRa 32 with sx1276 (RFM95) for 868/915MHz

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

*/

#define PIN_LED_PCB  GPIO_NUM_25  // status LED on the PCB

#define PIN_RFM_RST  GPIO_NUM_14  // Reset
#define PIN_RFM_IRQ  GPIO_NUM_26  // packet done on receive or transmit
#define PIN_RFM_SS   GPIO_NUM_18  // SPI chip-select
#define PIN_RFM_SCK  GPIO_NUM_5   // SPI clock
#define PIN_RFM_MISO GPIO_NUM_19  // SPI MISO
#define PIN_RFM_MOSI GPIO_NUM_27  // SPI MOSI

                                  // wire colours for VK2828U GPS
#define PIN_GPS_TXD  GPIO_NUM_12  // green
#define PIN_GPS_RXD  GPIO_NUM_35  // blue
#define PIN_GPS_PPS  GPIO_NUM_34  // white
#define PIN_GPS_ENA  GPIO_NUM_13  // yellow -> well, I had a problem here, thus I tied the enable wire to 3.3V for the time being.

#define CONS_UART UART_NUM_0      // UART0 for the console (the system does this for us)
#define GPS_UART  UART_NUM_1      // UART1 for GPS data read and dialog

#define I2C_BUS     I2C_NUM_1     // use bus #1 to talk to OLED and Baro sensor
#define I2C_SPEED   1000000       // 1MHz clock on I2C
#define PIN_I2C_SCL GPIO_NUM_15   // SCL pin
#define PIN_I2C_SDA GPIO_NUM_4    // SDA pin

#define OLED_I2C_ADDR 0x3C        // I2C address of the OLED display
#define PIN_OLED_RST GPIO_NUM_16  // OLED RESET pin

// ======================================================================================================
// 48-bit unique ID of the chip

uint64_t getUniqueID(void)
{ uint64_t ID=0; esp_err_t ret=esp_efuse_mac_get_default((uint8_t *)&ID); return ID; }

// ======================================================================================================

FlashParameters Parameters;

//--------------------------------------------------------------------------------------------------------
// STatus LED

void LED_PCB_On   (void) { gpio_set_level(PIN_LED_PCB, 1); }  // LED is on GPIO25
void LED_PCB_Off  (void) { gpio_set_level(PIN_LED_PCB, 0); }

//--------------------------------------------------------------------------------------------------------
// Console UART

SemaphoreHandle_t CONS_Mutex;

// int  CONS_UART_Read       (uint8_t &Byte) { return uart_read_bytes  (CONS_UART, &Byte, 1, 0); }  // non-blocking
// void CONS_UART_Write      (char     Byte) {        uart_write_bytes (CONS_UART, &Byte, 1);    }  // blocking ?
void CONS_UART_Write (char     Byte)  { putchar(Byte); }
int  CONS_UART_Read  (uint8_t &Byte)  { int Ret=getchar(); if(Ret>=0) { Byte=Ret; return 1; } else return Ret; }
// int  CONS_UART_Free  (void)           { return UART2_Free(); }
// int  CONS_UART_Full  (void)           { return UART2_Full(); }

//--------------------------------------------------------------------------------------------------------
// GPS UART

// int   GPS_UART_Full       (void)          { size_t Full=0; uart_get_buffered_data_len(GPS_UART, &Full); return Full; }
int   GPS_UART_Read       (uint8_t &Byte) { return uart_read_bytes  (GPS_UART, &Byte, 1, 0); }  // should be buffered and non-blocking
void  GPS_UART_Write      (char     Byte) {        uart_write_bytes (GPS_UART, &Byte, 1);    }  // should be buffered and blocking
void  GPS_UART_SetBaudrate(int BaudRate)  {        uart_set_baudrate(GPS_UART, BaudRate);    }

#ifdef WITH_GPS_ENABLE
void GPS_DISABLE(void) { gpio_set_level(PIN_GPS_ENA, 0); }
void GPS_ENABLE (void) { gpio_set_level(PIN_GPS_ENA, 1); }
#endif
bool GPS_PPS_isOn(void) { return gpio_get_level(PIN_GPS_PPS); }

//--------------------------------------------------------------------------------------------------------
// RF chip

inline void RFM_RESET_High(void) { gpio_set_level(PIN_RFM_RST, 1); }
inline void RFM_RESET_Low (void) { gpio_set_level(PIN_RFM_RST, 0); }

void RFM_RESET(uint8_t On)
{ if(On) RFM_RESET_Low();
    else RFM_RESET_High(); }

bool RFM_DIO0_isOn(void)   { return gpio_get_level(PIN_RFM_IRQ); }

static spi_device_handle_t RFM_SPI;

void RFM_TransferBlock(uint8_t *Data, uint8_t Len)
{ spi_transaction_t Trans;
  memset(&Trans, 0, sizeof(Trans));
  Trans.tx_buffer = Data;
  Trans.rx_buffer = Data;
  Trans.length = 8*Len;
  esp_err_t ret = spi_device_transmit(RFM_SPI, &Trans); }

//--------------------------------------------------------------------------------------------------------
// OLED display

#ifdef WITH_OLED
void OLED_RESET(bool Level) { gpio_set_level(PIN_OLED_RST, Level); }

esp_err_t OLED_Init(void)
{ i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
  i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
  i2c_master_write_byte(cmd, 0x14, true);
  i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
  i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping
  i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
  i2c_master_stop(cmd);
  esp_err_t espRc = i2c_master_cmd_begin(I2C_BUS, cmd, 10);
  i2c_cmd_link_delete(cmd);
  return espRc; }

esp_err_t OLED_SetContrast(uint8_t Contrast)
{ i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
  i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);
  i2c_master_write_byte(cmd, Contrast, true);
  i2c_master_stop(cmd);
  esp_err_t espRc = i2c_master_cmd_begin(I2C_BUS, cmd, 10);
  i2c_cmd_link_delete(cmd);
  return espRc; }

esp_err_t OLED_PutLine(uint8_t Line, const char *Text)
{ if(Line>=8) return ESP_OK;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
  i2c_master_write_byte(cmd, 0x00, true);
  i2c_master_write_byte(cmd, 0x10, true);
  i2c_master_write_byte(cmd, 0xB0 | Line, true);
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
    i2c_master_write_byte(cmd, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
    i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)Char], 8, true);
    i2c_master_stop(cmd);
    espRc = i2c_master_cmd_begin(I2C_BUS, cmd, 10);
    i2c_cmd_link_delete(cmd);
    if(espRc!=ESP_OK) break; }
  return espRc; }

esp_err_t OLED_Clear(void)
{ esp_err_t espRc;
  for(uint8_t Line=0; Line<8; Line++)
  { espRc=OLED_PutLine(Line, 0); if(espRc!=ESP_OK) break; }
  return espRc; }
#endif

//--------------------------------------------------------------------------------------------------------

volatile uint8_t LED_PCB_Counter = 0;

void LED_PCB_Flash(uint8_t Time) { if(Time>LED_PCB_Counter) LED_PCB_Counter=Time; } // [ms]

void LED_TimerCheck(uint8_t Ticks)
{ uint8_t Counter=LED_PCB_Counter;
  if(Counter)
  { if(Ticks<Counter) Counter-=Ticks;
                 else Counter =0;
    if(Counter) LED_PCB_On();
           else LED_PCB_Off();
    LED_PCB_Counter=Counter; }
}

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

void IO_Configuration(void)
{
  CONS_Mutex = xSemaphoreCreateMutex();

  gpio_set_direction(PIN_LED_PCB, GPIO_MODE_OUTPUT);      // LED
  LED_PCB_Off();

  gpio_set_direction(PIN_RFM_RST, GPIO_MODE_OUTPUT);      // RF chip GPIO pins
  gpio_set_direction(PIN_RFM_IRQ, GPIO_MODE_INPUT);
  RFM_RESET(0);

  spi_bus_config_t BusCfg =                               // RF chip SPI
  { mosi_io_num: PIN_RFM_MOSI,
    miso_io_num: PIN_RFM_MISO,
    sclk_io_num: PIN_RFM_SCK,
    quadwp_io_num: -1,
    quadhd_io_num: -1,
    max_transfer_sz: 64
  };
  spi_device_interface_config_t DevCfg =
  { command_bits: 0,
    address_bits: 0,
    dummy_bits: 0,
    mode: 0,
    duty_cycle_pos: 0,
    cs_ena_pretrans: 0,
    cs_ena_posttrans: 0,
    clock_speed_hz: 4000000,
    spics_io_num: PIN_RFM_SS,
    flags: 0,
    queue_size: 3,
    pre_cb: 0,
    post_cb: 0
  };
  esp_err_t ret=spi_bus_initialize(HSPI_HOST, &BusCfg, 1);
  ret=spi_bus_add_device(HSPI_HOST, &DevCfg, &RFM_SPI);

  gpio_set_direction(PIN_GPS_PPS, GPIO_MODE_INPUT);
#ifdef WITH_GPS_ENABLE
  gpio_set_direction(PIN_GPS_ENA, GPIO_MODE_OUTPUT);    // GPS GPIO
  GPS_ENABLE();
#endif

  uart_config_t GPS_UART_Config =                       // GPS UART
  { baud_rate: 9600,
    data_bits: UART_DATA_8_BITS,
    parity:    UART_PARITY_DISABLE,
    stop_bits: UART_STOP_BITS_1,
    flow_ctrl: UART_HW_FLOWCTRL_DISABLE,
    rx_flow_ctrl_thresh: 0,
    use_ref_tick: 0
  };
  uart_param_config  (GPS_UART, &GPS_UART_Config);
  uart_set_pin       (GPS_UART, PIN_GPS_TXD, PIN_GPS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(GPS_UART, 256, 256, 0, 0, 0);

  gpio_set_direction(PIN_OLED_RST, GPIO_MODE_OUTPUT);

  i2c_config_t I2C_Config =                            // I2C for OLED and pressue sensor
  { mode:          I2C_MODE_MASTER,
    sda_io_num:    PIN_I2C_SDA,
    sda_pullup_en: GPIO_PULLUP_ENABLE,
    scl_io_num:    PIN_I2C_SCL,
    scl_pullup_en: GPIO_PULLUP_ENABLE
  } ;
  I2C_Config.master.clk_speed =  I2C_SPEED;
  i2c_param_config  (I2C_BUS, &I2C_Config);
  i2c_driver_install(I2C_BUS, I2C_Config.mode, 0, 0, 0);

#ifdef WITH_OLED
  OLED_RESET(0);
  vTaskDelay(10);
  OLED_RESET(1);
  vTaskDelay(10);

  OLED_Init();
  OLED_Clear();
  OLED_SetContrast(128);
#endif

  // esp_register_freertos_tick_hook(&vApplicationTickHook);
}

// ======================================================================================================

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{ xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "BT_SPP: ");
  Format_Hex(CONS_UART_Write, (uint32_t)event);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex); }

int BT_SPP_Init(void)
{ esp_bt_controller_config_t BTconf = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_err_t Err = esp_bt_controller_init(&BTconf); if(Err!=ESP_OK) return Err;
  Err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT); if(Err!=ESP_OK) return Err;
  Err = esp_bluedroid_init(); if(Err!=ESP_OK) return Err;
  Err = esp_bluedroid_enable(); if(Err!=ESP_OK) return Err;
  Err = esp_spp_register_callback(esp_spp_cb); if(Err!=ESP_OK) return Err;
  Err = esp_spp_init(esp_spp_mode); if(Err!=ESP_OK) return Err;
  return Err; }

// ======================================================================================================

int NVS_Init(void)
{ esp_err_t Err = nvs_flash_init();
  if (Err == ESP_ERR_NVS_NO_FREE_PAGES)
  { nvs_flash_erase();
    Err = nvs_flash_init(); }

  if(Parameters.ReadFromNVS()!=ESP_OK)
  { Parameters.setDefault(getUniqueID());
    Parameters.WriteToNVS(); }

  return Err; }

// ======================================================================================================

int SPIFFS_Register(const char *Path, const char *Label, size_t MaxOpenFiles)
{ esp_vfs_spiffs_conf_t FSconf =
  { base_path: Path,
    partition_label: Label,
    max_files: MaxOpenFiles,
    format_if_mount_failed: true };
  return esp_vfs_spiffs_register(&FSconf); }

int SPIFFS_Info(size_t &Total, size_t &Used, const char *Label)
{ return esp_spiffs_info(Label, &Total, &Used); }

// ======================================================================================================
/*
uint8_t I2C_Read(i2c_port_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10)
{ i2c_cmd_handle_t Cmd = i2c_cmd_link_create();
  i2c_master_start(Cmd);
  i2c_master_write_byte(Cmd, (Addr<<1) | I2C_MASTER_READ , I2C_MASTER_ACK);
  i2c_master_write_byte(Cmd, Reg , I2C_MASTER_ACK);
  i2c_master_start(Cmd);
  i2c_master_read(Cmd, Data, Len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(Cmd);
  esp_err_t Ret = i2c_master_cmd_begin(Bus, Cmd, Wait);
  i2c_cmd_link_delete(Cmd);
  return Ret; }


uint8_t I2C_Write(i2c_port_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10)
{ i2c_cmd_handle_t Cmd = i2c_cmd_link_create();
  i2c_master_start(Cmd);
  i2c_master_write_byte(Cmd, (Addr<<1) | I2C_MASTER_WRITE , I2C_MASTER_ACK);
  i2c_master_write_byte(Cmd, Reg , I2C_MASTER_ACK);
  i2c_master_write(Cmd, Data, Len, I2C_MASTER_NACK);
  i2c_master_stop(Cmd);
  esp_err_t Ret = i2c_master_cmd_begin(Bus, Cmd, Wait);
  i2c_cmd_link_delete(Cmd);
  return Ret; }
*/
// ======================================================================================================

