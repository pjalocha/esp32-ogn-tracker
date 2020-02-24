extern uint32_t BatteryVoltage;       // [1/256 mV] averaged
extern  int32_t BatteryVoltageRate;   // [1/256 mV] averaged

#ifdef WITH_LOOKOUT                   // traffic awareness and warnings
#include "lookout.h"
extern LookOut Look;
#endif


#ifdef WITH_ESP32
const uint8_t RelayQueueSize = 32;
#else
const uint8_t RelayQueueSize = 16;
#endif

extern OGN_PrioQueue<OGN_Packet, RelayQueueSize> RelayQueue;       // received packets and candidates to be relayed

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskPROC(void* pvParameters);

