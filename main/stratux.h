#include "hal.h"

bool Stratux_isConnected(void);
 int Stratux_Read (uint8_t &Byte);
void Stratux_Write (char Byte);

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskSTX(void* pvParameters);

