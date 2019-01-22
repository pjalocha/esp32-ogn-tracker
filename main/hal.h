#ifndef __HAL_H__
#define __HAL_H__

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ============================================================================================================

#define WITH_ESP32
// #define WITH_AUTOCR                        // we do the auto-CR after LF thus take care to disable CRLF in ESP-IDF

#define WITH_OGN1                          // OGN protocol version 1/2
#define OGN_Packet OGN1_Packet

#define HARDWARE_ID 0x02
#define SOFTWARE_ID 0x01

// #define DEFAULT_AcftType        1          // [0..15] default aircraft-type: glider
// #define DEFAULT_GeoidSepar     40          // [m]
// #define DEFAULT_CONbaud    115200
// #define DEFAULT_PPSdelay       80

#define USE_BLOCK_SPI                      // use block SPI interface for RF chip
#define I2C_SPEED 1000000                  // [Hz] bit rate on the I2C (nominally up to 400000)

// #define WITH_HELTEC                        // HELTEC module: PCB LED on GPI025
// #define WITH_TTGO                          // TTGO module: PCB LED on GPIO2, GPIO25 free to use as DAC2 output
// #define WITH_TBEAM                          // T-Beam module
// #define WITH_JACEK                         // JACEK ESP32 OGN-Tracker

// #define WITH_OLED                          // OLED display on the I2C: some TTGO modules are without OLED display
// #define WITH_OLED2                         // 2nd OLED display, I2C address next higher

// #define WITH_RFM95                         // RF chip selection:  both HELTEC and TTGO use sx1276 which is same af RFM95
// #define WITH_RFM69                         // Jacek design uses RFM69

// #define WITH_LED_RX
// #define WITH_LED_TX

// #define WITH_GPS_ENABLE                    // use GPS_ENABLE control line to turn the GPS ON/OFF
// #define WITH_GPS_PPS                       // use the PPS signal from GPS for precise time-sync.
// #define WITH_GPS_CONFIG                    // attempt to configure higher GPS baud rate and airborne mode
// #define WITH_GPS_UBX                       // GPS understands UBX
// #define WITH_GPS_MTK                       // GPS understands MTK
// #define WITH_GPS_SRF
// #define WITH_MAVLINK

// #define WITH_GPS_UBX_PASS                  // to pass directly UBX packets to/from GPS
// #define WITH_GPS_NMEA_PASS                  // to pass directly NMEA to/from GPS

// #define WITH_BMP180                        // BMP180 pressure sensor
// #define WITH_BMP280                        // BMP280 pressure sensor
// #define WITH_BME280                        // with humidity
// #define WITH_MS5607                        // MS5607 pressure sensor

// #define WITH_FLARM                         // Receive FLARM
// #define WITH_PFLAA                         // PFLAU and PFLAA for compatibility with XCsoar and LK8000

// #define WITH_CONFIG                        // interpret the console input: $POGNS to change parameters

// #define WITH_BEEPER

// #define WITH_SD                            // use the SD card in SPI mode and FAT file system
// #define WITH_SPIFFS                        // use SPIFFS file system in Flash
// #define WITH_LOG                           // log own positions and other received to SPIFFS and possibly to uSD

// #define WITH_BT_SPP                        // Bluetooth serial port for smartphone/tablet link
// #define WITH_WIFI                          // attempt to connect to the wifi router for uploading the log files
// #define WITH_SPIFFS_LOG                    // log transmitted and received packets to SPIFFS

#include "config.h"                        // user options

// ============================================================================================================

extern uint8_t BARO_I2C;

#ifdef WITH_MAVLINK
const  uint8_t  MAV_SysID = 1;             // System-ID for MAVlink messages we send out
extern uint8_t  MAV_Seq;                   // sequence number for MAVlink message sent out
#endif

// ============================================================================================================

extern SemaphoreHandle_t CONS_Mutex;       // console port Mutex
// extern SemaphoreHandle_t I2C_Mutex;        // I2C port Mutex (OLED and Baro)

uint64_t getUniqueID(void);                // get some unique ID of the CPU/chip
uint32_t getUniqueAddress(void);           // get unique 24-bit address for the transmitted IF

#include "parameters.h"
extern FlashParameters Parameters;

int  CONS_UART_Read       (uint8_t &Byte); // non-blocking
void CONS_UART_Write      (char     Byte); // blocking
int  CONS_UART_Free       (void);          // how many bytes can be written to the transmit buffer
int  CONS_UART_Full       (void);          // how many bytes already in the transmit buffer
void CONS_UART_SetBaudrate(int BaudRate);

int   GPS_UART_Read       (uint8_t &Byte); // non-blocking
void  GPS_UART_Write      (char     Byte); // blocking
void  GPS_UART_SetBaudrate(int BaudRate);

bool GPS_PPS_isOn(void);

void RFM_TransferBlock(uint8_t *Data, uint8_t Len);
void RFM_RESET(uint8_t On);              // RF module reset
bool RFM_IRQ_isOn(void);                 // query the IRQ state

#ifdef WITH_OLED
int OLED_SetContrast(uint8_t Contrast, uint8_t DispIdx=0);
int OLED_PutLine(uint8_t Line, const char *Text, uint8_t DispIdx=0);
#endif

#ifdef WITH_SD
esp_err_t SD_Mount(void);
void      SD_Unmount();
bool      SD_isMounted();
int       SD_getSectors(void);
int       SD_getSectorSize(void);
#endif

#ifdef WITH_BEEPER
const uint8_t KNOB_Tick=15;              // for now, when there is no knob

const uint8_t Play_Vol_0 = 0x00;
const uint8_t Play_Vol_1 = 0x40;
const uint8_t Play_Vol_2 = 0x80;
const uint8_t Play_Vol_3 = 0xC0;

const uint8_t Play_Oct_0 = 0x00;
const uint8_t Play_Oct_1 = 0x10;
const uint8_t Play_Oct_2 = 0x20;
const uint8_t Play_Oct_3 = 0x30;

void Play(uint8_t Note, uint8_t Len);    // put anote to play in the queue
uint8_t Play_Busy(void);                 // check is the queue is empty or still busy playing ?

void Play_TimerCheck(void);              // every ms serve the note playing

void Beep(uint16_t Freq, uint8_t Duty, uint8_t DoubleAmpl);
void Beep_Note(uint8_t Note);
#endif // WITH_BEEPER

void LED_PCB_On   (void);                // LED on the PCB for vizual indications
void LED_PCB_Off  (void);
void LED_PCB_Flash(uint8_t Time=100);    // Flash the PCB LED for a period of [ms]

#ifdef WITH_LED_TX
void LED_TX_On    (void);
void LED_TX_Off   (void);
void LED_TX_Flash (uint8_t Time=100);
#endif

#ifdef WITH_LED_RX
void LED_TX_On    (void);
void LED_TX_Off   (void);
void LED_RX_Flash(uint8_t Time=100);
#endif

void LED_TimerCheck(uint8_t Ticks=1);

void IO_Configuration(void);             // Configure I/O

int  NVS_Init(void);                     // initialize non-volatile-storage in the Flash

#ifdef WITH_BT_SPP
int  BT_SPP_Init(void);
#endif

#ifdef WITH_SPIFFS
int  SPIFFS_Register(const char *Path="/spiffs", const char *Label=0, size_t MaxOpenFiles=5);
int  SPIFFS_Info(size_t &Total, size_t &Used, const char *Label=0);
#endif

uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10);
uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10);

template <class Type>
 inline uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, Type &Object, uint8_t Wait=10)
{ return I2C_Write(Bus, Addr, Reg, (uint8_t *)&Object, sizeof(Type), Wait); }

template <class Type>
 inline uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, Type &Object, uint8_t Wait=10)
{ return I2C_Read (Bus, Addr, Reg, (uint8_t *)&Object, sizeof(Type), Wait); }

uint8_t I2C_Restart(uint8_t Bus);

uint16_t BatterySense(int Samples=4); // [mV]

#endif // __HAL_H__
