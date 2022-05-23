#ifndef __SDLOG_H__
#define __SDLOG_H__

#include <stdint.h>

#include "hal.h"

#include "igc-key.h"

void Log_Write(char Byte);
int  Log_Free(void);
extern SemaphoreHandle_t Log_Mutex;

extern FIFO<OGN_RxPacket<OGN_Packet>, 32> IGClog_FIFO;

extern IGC_Key IGC_SignKey;

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskSDLOG(void* pvParameters);

/*
#ifdef __cplusplus
  extern "C"
#endif
 void vTaskIGC(void* pvParameters);
*/
#endif // __SDLOG_H__
