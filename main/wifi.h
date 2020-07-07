#include "hal.h"

bool WIFI_isConnected(void);
bool APRS_isConnected(void);

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskWIFI(void* pvParameters);

