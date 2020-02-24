#include "hal.h"

void SoundMsg(char ch);
int SoundMsg(const char *Msg);

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskSOUND(void* pvParameters);


