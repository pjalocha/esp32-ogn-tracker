#include "fifo.h"
#include "hal.h"

extern FIFO<uint8_t, 8> KeyBuffer;

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskCTRL(void* pvParameters);

