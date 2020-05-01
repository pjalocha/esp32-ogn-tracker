#ifndef __RF_H__
#define __RF_H__

#include <stdint.h>

#include "hal.h"

#ifdef __cplusplus

#include "ogn.h"
#include "rfm.h"
#include "fifo.h"
#include "freqplan.h"

#ifdef WITH_FANET
#include "fanet.h"
#endif

  extern FIFO<RFM_RxPktData, 16> RF_RxFIFO;   // buffer for received packets
  extern FIFO<OGN_TxPacket<OGN_Packet>, 4> RF_TxFIFO;   // buffer for transmitted packets

#ifdef WITH_FANET
  extern FIFO<FANET_RxPacket, 8> FNT_RxFIFO;
  extern FIFO<FANET_Packet, 4> FNT_TxFIFO;
#endif

  extern uint8_t RX_OGN_Packets;              // [packets] counts received packets
  // extern uint8_t   RX_AverRSSI;              // [-0.5dBm] average RSSI
  // extern  int8_t       RF_Temp;              // [degC] temperature of the RF chip: uncalibrated
  extern RFM_TRX           TRX;               // RF transceiver
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

#endif // __RF_H__
