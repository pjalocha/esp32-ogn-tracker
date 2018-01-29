#include <stdint.h>

#include "hal.h"

#ifdef __cplusplus

#include "ogn.h"
#include "rfm.h"
#include "fifo.h"
#include "freqplan.h"

  extern FIFO<RFM_RxPktData, 16> RF_RxFIFO;   // buffer for received packets
  extern FIFO<OGN_TxPacket,   4> RF_TxFIFO;   // buffer for transmitted packets

  extern uint8_t RX_OGN_Packets;              // [packets] counts received packets
  extern uint8_t   RX_AverRSSI;               // [-0.5dBm] average RSSI
  extern  int8_t       RF_Temp;               // [degC] temperature of the RF chip: uncalibrated
  extern FreqPlan  RF_FreqPlan;               // frequency hopping pattern calculator
  extern uint16_t    TX_Credit;               // counts transmitted packets vs. time to avoid using more than 1% of the time
  extern uint16_t RX_OGN_Count64;             // counts received packets for the last 64 seconds
  extern uint32_t RX_Random;                  // Random number from LSB of RSSI readouts

         void XorShift32(uint32_t &Seed);     // simple random number generator
#endif

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskRF(void* pvParameters);

