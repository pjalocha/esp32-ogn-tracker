#ifndef __HAL_H__
#define __HAL_H__

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ============================================================================================================

#define WITH_ESP32

#define HARDWARE_ID 0x02
#define SOFTWARE_ID 0x01

#define USE_BLOCK_SPI                      // use block SPI interface for RF chip

#define WITH_RFM95                         // RF chip selection
// #define WITH_RFM69
// #define WITH_LED_RX
// #define WITH_LED_TX

// #define WITH_GPS_ENABLE                    // use GPS_ENABLE control line to turn the GPS ON/OFF
#define WITH_GPS_PPS                       // use the PPS signal from GPS for precise time-sync.
#define WITH_GPS_CONFIG                    // attempt to configure higher GPS baud rate and airborne mode
#define WITH_GPS_UBX                       // GPS understands UBX
// #define WITH_GPS_MTK                       // GPS understands MTK
// #define WITH_GPS_SRF
// #define WITH_MAVLINK

// #define WITH_BMP180                        // BMP180 pressure sensor
// #define WITH_BMP280                        // BMP280 pressure sensor
// #define WITH_MS5607                        // MS5607 pressure sensor

#define I2C_SPEED 1000000                  // [Hz]

#define WITH_PFLAA                         // PFLAU and PFLAA for compatibility with XCsoar and LK8000

#define WITH_CONFIG                        // interpret the console input: $POGNS to change parameters

#define WITH_OLED                          // OLED display on the I2C

// #define WITH_BT_SPP                        // Bluetooth serial port fo smartphone/tablet link

// ============================================================================================================

extern uint8_t BARO_I2C;

#ifdef WITH_MAVLINK
const  uint8_t  MAV_SysID = 1;             // System-ID for MAVlink messages we send out
extern uint8_t  MAV_Seq;                   // sequence number for MAVlink message sent out
#endif

// ============================================================================================================

extern SemaphoreHandle_t CONS_Mutex;       // console port Mutex
extern SemaphoreHandle_t I2C_Mutex;        // I2C port Mutex (OLED and Baro)

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
int OLED_SetContrast(uint8_t Contrast);
int OLED_PutLine(uint8_t Line, const char *Text);
#endif

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

int  SPIFFS_Register(const char *Path="/spiffs", const char *Label=0, size_t MaxOpenFiles=5);
int  SPIFFS_Info(size_t &Total, size_t &Used, const char *Label=0);

uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10);
uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait=10);

template <class Type>
 inline uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, Type &Object, uint8_t Wait=10)
{ return I2C_Write(Bus, Addr, Reg, (uint8_t *)&Object, sizeof(Type), Wait); }

template <class Type>
 inline uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, Type &Object, uint8_t Wait=10)
{ return I2C_Read (Bus, Addr, Reg, (uint8_t *)&Object, sizeof(Type), Wait); }

uint8_t I2C_Restart(uint8_t Bus);

#endif // __HAL_H__
