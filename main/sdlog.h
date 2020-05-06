#ifndef __SDLOG_H__
#define __SDLOG_H__

#include <stdint.h>

#include "hal.h"

void Log_Write(char Byte);
int  Log_Free(void);
extern SemaphoreHandle_t Log_Mutex;

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskSDLOG(void* pvParameters);

#endif // __SDLOG_H__
