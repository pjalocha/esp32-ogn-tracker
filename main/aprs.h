#ifndef __APRS_H__
#define __APRS_H__

#include "hal.h"
#include "ogn.h"
#include "fifo.h"

extern FIFO<OGN_RxPacket<OGN_Packet>, 16> APRSrx_FIFO;
extern FIFO<OGN_TxPacket<OGN_Packet>,  4> APRStx_FIFO;

bool WIFI_isConnected(void);
bool APRS_isConnected(void);

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskAPRS(void* pvParameters);

#endif // __APRS_H__

