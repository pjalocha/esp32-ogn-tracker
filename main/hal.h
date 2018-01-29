#ifndef __HAL_H__
#define __HAL_H__

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define HARDWARE_ID 0x02
#define SOFTWARE_ID 0x01

#define USE_BLOCK_SPI                      // use block SPI interface for RF chip

#define WITH_RFM95                         // RF chip selection
// #define WITH_RFM69

// #define WITH_GPS_ENABLE                 // use GPS_ENABLE control line to turn the GPS ON/OFF
#define WITH_GPS_PPS                       // use the PPS signal from GPS for precise time-sync.
#define WITH_GPS_CONFIG                    // attempt to configure higher GPS baud rate and airborne mode
#define WITH_GPS_UBX                       // GPS understands UBX
// #define WITH_GPS_MTK                       // GPS understands MTK
// #define WITH_GPS_SRF

#define WITH_PFLAA
#define WITH_CONFIG                        // interpret the console input: $POGNS to change parameters
#define WITH_OLED                          // OLED display on the I2C

extern SemaphoreHandle_t CONS_Mutex;       // console port Mutex

#include "parameters.h"
extern FlashParameters Parameters;

uint64_t getUniqueID(void);              // get some unique ID of the CPU/chip

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
bool RFM_DIO0_isOn(void);                // query the IRQ state

#ifdef WITH_OLED
int OLED_SetContrast(uint8_t Contrast);
int OLED_PutLine(uint8_t Line, const char *Text);
#endif

void LED_PCB_On   (void);                // LED on the PCB for vizual indications
void LED_PCB_Off  (void);

void LED_PCB_Flash(uint8_t Time=100);    // Flash the PCB LED for a period of [ms]
void LED_TimerCheck(uint8_t Ticks=1);

void IO_Configuration(void);             // Configure I/O

int  NVS_Init(void);                     // initialize non-volatile-storage in the Flash

int  BT_SPP_Init(void);
int  SPIFFS_Register(const char *Path="/spiffs", const char *Label=0, size_t MaxOpenFiles=5);
int  SPIFFS_Info(size_t &Total, size_t &Used, const char *Label=0);

#endif // __HAL_H__
