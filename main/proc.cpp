#include <stdint.h>

#include "hal.h"
#include "proc.h"
#include "ctrl.h"

#include "ogn.h"

#include "rf.h"
#include "gps.h"

#ifdef WITH_FLASHLOG
#include "flashlog.h"
#endif

static char           Line[128];      // for printing out to serial port, etc.

static LDPC_Decoder     Decoder;      // error corrector for the OGN Gallager code

// #define DEBUG_PRINT

// ==================================================================

// ---------------------------------------------------------------------------------------------------------------------------------------

static OGN_PrioQueue<16> RelayQueue;  // received packets and candidates to be relayed

#ifdef DEBUG_PRINT
static void PrintRelayQueue(uint8_t Idx)                    // for debug
{ uint8_t Len=0;
  // Len+=Format_String(Line+Len, "");
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  // Format_String(CONS_UART_Write, Line, Len);
  Line[Len++]='['; Len+=Format_Hex(Line+Len, Idx); Line[Len++]=']'; Line[Len++]=' ';
  Len+=RelayQueue.Print(Line+Len);
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex); }
#endif

static bool GetRelayPacket(OGN_TxPacket *Packet)      // prepare a packet to be relayed
{ if(RelayQueue.Sum==0) return 0;                     // if no packets in the relay queue
  XorShift32(RX_Random);                              // produce a new random number
  uint8_t Idx=RelayQueue.getRand(RX_Random);          // get weight-random packet from the relay queue
  if(RelayQueue.Packet[Idx].Rank==0) return 0;        // should not happen ...
  memcpy(Packet->Packet.Byte(), RelayQueue[Idx]->Byte(), OGN_Packet::Bytes); // copy the packet
  Packet->Packet.Header.RelayCount+=1;                // increment the relay count (in fact we only do single relay)
  Packet->Packet.Whiten(); Packet->calcFEC();         // whiten and calc. the FEC code => packet ready for transmission
  // PrintRelayQueue(Idx);  // for debug
  RelayQueue.decrRank(Idx);                           // reduce the rank of the packet selected for relay
  return 1; }

static void CleanRelayQueue(uint32_t Time, uint32_t Delay=20) // remove "old" packets from the relay queue
{ RelayQueue.cleanTime((Time-Delay)%60); }            // remove packets 20(default) seconds into the past

// ---------------------------------------------------------------------------------------------------------------------------------------

static void ReadStatus(OGN_TxPacket &StatPacket)                            // read the device status and fill the status packet
{

#ifdef WITH_STM32
  uint16_t MCU_VCC   = Measure_MCU_VCC();                                    // [0.001V]
  StatPacket.Packet.EncodeVoltage(((MCU_VCC<<3)+62)/125);                    // [1/64V]
  int16_t MCU_Temp  = Measure_MCU_Temp();                                   // [0.1degC]
#endif

  if(StatPacket.Packet.Status.Pressure==0) StatPacket.Packet.EncodeTemperature(RF_Temp*10); // [0.1degC]
  StatPacket.Packet.Status.RadioNoise = RX_AverRSSI;                         // [-0.5dBm] write radio noise to the status packet

  StatPacket.Packet.Status.TxPower = Parameters.getTxPower()-4;

  uint16_t RxRate = RX_OGN_Count64+1;
  uint8_t RxRateLog2=0; RxRate>>=1; while(RxRate) { RxRate>>=1; RxRateLog2++; }
  StatPacket.Packet.Status.RxRate = RxRateLog2;
                                                                             // produce the POGNR sentence
  { uint8_t Len=0;
    Len+=Format_String(Line+Len, "$POGNR,");                                 // NMEA report: radio status
    Len+=Format_UnsDec(Line+Len, RF_FreqPlan.Plan);                          // which frequency plan
    Line[Len++]=',';
    Len+=Format_UnsDec(Line+Len, RX_OGN_Count64);                            // number of OGN packets received
    Line[Len++]=',';
    Line[Len++]=',';
    Len+=Format_SignDec(Line+Len, -5*RX_AverRSSI, 2, 1);                     // average RF level (over all channels)
    Line[Len++]=',';
    Len+=Format_UnsDec(Line+Len, (uint16_t)TX_Credit);
    Line[Len++]=',';
    Len+=Format_SignDec(Line+Len, (int16_t)RF_Temp);                         // the temperature of the RF chip
    Line[Len++]=',';
    // Len+=Format_SignDec(Line+Len, MCU_Temp, 2, 1);
    Line[Len++]=',';
    // Len+=Format_UnsDec(Line+Len, (MCU_VCC+5)/10, 3, 2);

    Len+=NMEA_AppendCheckCRNL(Line, Len);                                    // append NMEA check-sum and CR+NL
    // LogLine(Line);
    // if(CONS_UART_Free()>=128)
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, Line, 0, Len);                               // send the NMEA out to the console
      xSemaphoreGive(CONS_Mutex); }
#ifdef WITH_SDLOG
    if(Log_Free()>=128)
    { xSemaphoreTake(Log_Mutex, portMAX_DELAY);
      Format_String(Log_Write, Line, Len, 0);                                     // send the NMEA out to the log file
      xSemaphoreGive(Log_Mutex); }
#endif
  }
}

// ---------------------------------------------------------------------------------------------------------------------------------------

static uint8_t WritePFLAU(char *NMEA, uint8_t GPS=1)    // produce the (mostly dummy) PFLAU to satisfy XCsoar and LK8000
{ uint8_t Len=0;
  Len+=Format_String(NMEA+Len, "$PFLAU,");
  NMEA[Len++]='0';
  NMEA[Len++]=',';
  NMEA[Len++]='0'+GPS;                                  // TX status
  NMEA[Len++]=',';
  NMEA[Len++]='0'+GPS;                                  // GPS status
  NMEA[Len++]=',';
  NMEA[Len++]='1';                                      // power status: one could monitor the supply
  NMEA[Len++]=',';
  NMEA[Len++]='0';
  NMEA[Len++]=',';
  NMEA[Len++]=',';
  NMEA[Len++]='0';
  NMEA[Len++]=',';
  NMEA[Len++]=',';
  Len+=NMEA_AppendCheckCRNL(NMEA, Len);
  NMEA[Len]=0;
  return Len; }

// ---------------------------------------------------------------------------------------------------------------------------------------

static void ProcessRxPacket(OGN_RxPacket *RxPacket, uint8_t RxPacketIdx)              // process every (correctly) received packet
{ int32_t LatDist=0, LonDist=0; uint8_t Warn=0;
  if( RxPacket->Packet.Header.Other || RxPacket->Packet.Header.Encrypted ) return ;   // status packet or encrypted: ignore
  uint8_t MyOwnPacket = ( RxPacket->Packet.Header.Address  == Parameters.Address  )
                     && ( RxPacket->Packet.Header.AddrType == Parameters.AddrType );
  if(MyOwnPacket) return;                                                             // don't process my own (relayed) packets
  bool DistOK = RxPacket->Packet.calcDistanceVector(LatDist, LonDist, GPS_Latitude, GPS_Longitude, GPS_LatCosine)>=0;
  if(DistOK)
  { RxPacket->calcRelayRank(GPS_Altitude/10);                                         // calculate the relay-rank (priority for relay)
    RelayQueue.addNew(RxPacketIdx);
    uint8_t Len=RxPacket->WritePOGNT(Line);                                           // print on the console as $POGNT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, Line, 0, Len);
    xSemaphoreGive(CONS_Mutex);
#ifdef WITH_BEEPER
    if(KNOB_Tick>12) Play(Play_Vol_1 | Play_Oct_2 | 7, 3);                            // if Knob>12 => make a beep for every received packet
#endif
#ifdef WITH_SDLOG
    if(Log_Free()>=128)
    { xSemaphoreTake(Log_Mutex, portMAX_DELAY);
      Format_String(Log_Write, Line, Len, 0);
      xSemaphoreGive(Log_Mutex); }
#endif
#ifdef WITH_PFLAA
    Len=RxPacket->Packet.WritePFLAA(Line, Warn, LatDist, LonDist, RxPacket->Packet.DecodeAltitude()-GPS_Altitude/10); // print on the console
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, Line, 0, Len);
    xSemaphoreGive(CONS_Mutex);
#endif
#ifdef WITH_MAVLINK
    MAV_ADSB_VEHICLE MAV_RxReport;
    RxPacket->Packet.Encode(&MAV_RxReport);
    MAV_RxMsg::Send(sizeof(MAV_RxReport), MAV_Seq++, MAV_SysID, MAV_COMP_ID_ADSB, MAV_ID_ADSB_VEHICLE, (const uint8_t *)&MAV_RxReport, GPS_UART_Write);
//    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
//    MAV_RxMsg::Send(sizeof(MAV_RxReport), MAV_Seq++, MAV_SysID, MAV_COMP_ID_ADSB, MAV_ID_ADSB_VEHICLE, (const uint8_t *)&MAV_RxReport, CONS_UART_Write);
//    xSemaphoreGive(CONS_Mutex);
#endif
  }
}

static void DecodeRxPacket(RFM_RxPktData *RxPkt)
{
  uint8_t RxPacketIdx  = RelayQueue.getNew();                   // get place for this new packet
  OGN_RxPacket *RxPacket = RelayQueue[RxPacketIdx];
  // PrintRelayQueue(RxPacketIdx);                              // for debug
  // RxPacket->RxRSSI=RxPkt.RSSI;
  // TickType_t ExecTime=xTaskGetTickCount();

  { RX_OGN_Packets++;
    uint8_t Check = RxPkt->Decode(*RxPacket, Decoder);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "RxPacket: ");
    Format_Hex(CONS_UART_Write, RxPacket->Packet.HeaderWord);
    CONS_UART_Write(' ');
    Format_UnsDec(CONS_UART_Write, (uint16_t)Check);
    CONS_UART_Write('/');
    Format_UnsDec(CONS_UART_Write, (uint16_t)RxPacket->RxErr);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    if( (Check==0) && (RxPacket->RxErr<15) )                     // what limit on number of detected bit errors ?
    { RxPacket->Packet.Dewhiten();
      ProcessRxPacket(RxPacket, RxPacketIdx); }
  }

}

// -------------------------------------------------------------------------------------------------------------------

#ifdef __cplusplus
  extern "C"
#endif
void vTaskPROC(void* pvParameters)
{
#ifdef WITH_FLASHLOG
  uint16_t kB = FlashLog_OpenForWrite();
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "TaskPROC: ");
  Format_UnsDec(CONS_UART_Write, kB);
  Format_String(CONS_UART_Write, "KB FlashLog\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  RelayQueue.Clear();

  static uint16_t AverSpeed=0;                                          // [0.1m/s] average speed (including vertical)
  static bool     isMoving=0;                                           // is the aircraft moving ?

  static OGN_TxPacket PosPacket;                                        // position packet
  static uint32_t     PosTime=0;                                        // [sec] when the position was recorded
  static OGN_TxPacket StatPacket;                                       // status report packet
  static OGN_TxPacket InfoPacket;                                       // information packet

  for( ; ; )
  { vTaskDelay(1);

    RFM_RxPktData *RxPkt = RF_RxFIFO.getRead();                         // check for new received packets
    if(RxPkt)
    {
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60, 2);
      CONS_UART_Write('.');
      Format_UnsDec(CONS_UART_Write, TimeSync_msTime(), 3);
      Format_String(CONS_UART_Write, " RF_RxFIFO -> ");
      RxPkt->Print(CONS_UART_Write);
      // CONS_UART_Write('\r'); CONS_UART_Write('\n');
      xSemaphoreGive(CONS_Mutex);
#endif
      DecodeRxPacket(RxPkt);                                            // decode and process the received packet
      RF_RxFIFO.Read(); }

    static uint32_t PrevSlotTime=0;                                     // remember previous time slot to detect a change
    uint32_t SlotTime = TimeSync_Time();                                // time slot
    if(TimeSync_msTime()<300) SlotTime--;                               // lasts up to 0.300sec after the PPS

    if(SlotTime==PrevSlotTime) continue;                                // stil same time slot, go back to RX processing
    PrevSlotTime=SlotTime;                                              // new slot started
                                                                        // this part of the loop is executed only once per slot-time
    uint8_t BestIdx; int16_t BestResid;
#ifdef WITH_MAVLINK
    GPS_Position *Position = GPS_getPosition(BestIdx, BestResid, (SlotTime-1)%60, 0);
#else
    GPS_Position *Position = GPS_getPosition(BestIdx, BestResid, SlotTime%60, 0);
#endif
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "getPos() => ");
    Format_UnsDec(CONS_UART_Write, SlotTime%60, 2);
    CONS_UART_Write(' ');
    Format_UnsDec(CONS_UART_Write, (uint16_t)BestIdx);
    CONS_UART_Write(':');
    Format_SignDec(CONS_UART_Write, BestResid, 3, 2);
    Format_String(CONS_UART_Write, "s\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    // GPS_Position *Position = GPS_getPosition();
    if(Position) Position->EncodeStatus(StatPacket.Packet);             // encode GPS altitude and pressure
    if( Position && Position->isReady && (!Position->Sent) && Position->isReady && Position->isValid() )
    { AverSpeed=GPS_AverageSpeed();                                     // [0.1m/s] average speed, including the vertical speed
      isMoving = AverSpeed>10;
      if(Parameters.FreqPlan==0)
        RF_FreqPlan.setPlan(Position->Latitude, Position->Longitude);     // set the frequency plan according to the GPS position
      else RF_FreqPlan.setPlan(Parameters.FreqPlan);
/*
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60);
      CONS_UART_Write('.');
      Format_UnsDec(CONS_UART_Write, TimeSync_msTime(), 3);
      Format_String(CONS_UART_Write, " -> Sent\n");
      xSemaphoreGive(CONS_Mutex);
#endif
*/
      PosTime=Position->getUnixTime();
      PosPacket.Packet.HeaderWord=0;
      PosPacket.Packet.Header.Address    = Parameters.Address;         // set address
      PosPacket.Packet.Header.AddrType   = Parameters.AddrType;        // address-type
      PosPacket.Packet.calcAddrParity();                               // parity of (part of) the header
      if(BestResid==0) Position->Encode(PosPacket.Packet);             // encode position/altitude/speed/etc. from GPS position
                  else Position->Encode(PosPacket.Packet, BestResid);
      PosPacket.Packet.Position.Stealth  = Parameters.Stealth;
      PosPacket.Packet.Position.AcftType = Parameters.AcftType;        // aircraft-type
      OGN_TxPacket *TxPacket = RF_TxFIFO.getWrite();
      TxPacket->Packet = PosPacket.Packet;                             // copy the position packet to the TxFIFO
      TxPacket->Packet.Whiten(); TxPacket->calcFEC();                  // whiten and calculate FEC code
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60);
      CONS_UART_Write('.');
      Format_UnsDec(CONS_UART_Write, TimeSync_msTime(), 3);
      Format_String(CONS_UART_Write, " TxFIFO <- ");
      Format_Hex(CONS_UART_Write, TxPacket->Packet.HeaderWord);
      CONS_UART_Write('\r'); CONS_UART_Write('\n');
      xSemaphoreGive(CONS_Mutex);
#endif
      XorShift32(RX_Random);
      if( isMoving || ((RX_Random&0x3)==0) )                            // send only some positions if the speed is less than 1m/s
        RF_TxFIFO.Write();                                              // complete the write into the TxFIFO
      Position->Sent=1;
#ifdef WITH_PFLAA
      { uint8_t Len=WritePFLAU(Line);
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, Line, 0, Len);
        xSemaphoreGive(CONS_Mutex); }
#endif // WITH_PFLAA
#ifdef WITH_FLASHLOG
      bool Written=FlashLog_Process(PosPacket.Packet, PosTime);
      // if(Written)
      // { uint8_t Len=FlashLog_Print(Line);
      //   xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      //   Format_String(CONS_UART_Write, Line);
      //   xSemaphoreGive(CONS_Mutex);
      // }
#endif // WITH_FLASHLOG
    } else // if GPS position is not complete, contains no valid position, etc.
    { if((SlotTime-PosTime)>=30) { PosPacket.Packet.Position.Time=0x3F; } // if no valid position for more than 30 seconds then set the time as unknown for the transmitted packet
      OGN_TxPacket *TxPacket = RF_TxFIFO.getWrite();
      TxPacket->Packet = PosPacket.Packet;
      TxPacket->Packet.Whiten(); TxPacket->calcFEC();                 // whiten and calculate FEC code
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "TxFIFO: ");
      Format_Hex(CONS_UART_Write, TxPacket->Packet.HeaderWord);
      CONS_UART_Write('\r'); CONS_UART_Write('\n');
      xSemaphoreGive(CONS_Mutex);
#endif
      XorShift32(RX_Random);
      if(PosTime && ((RX_Random&0x3)==0) )                              // send if some position in the packet and at 1/4 normal rate
        RF_TxFIFO.Write();                                              // complete the write into the TxFIFO
      if(Position) Position->Sent=1;
    }
// #ifdef WITH_MAVLINK
//     { MAV_HEARTBEAT MAV_HeartBeat;
//     // = { custom_mode:0,
//     //     type:0,
//     //     autopilot:0,
//     //     base_mode:0,
//     //     system_status:4,
//     //     mavlink_version:1
//     //   };
//       MAV_HeartBeat.custom_mode=0;
//       MAV_HeartBeat.type=0;
//       MAV_HeartBeat.autopilot=0;
//       MAV_HeartBeat.base_mode=0;
//       MAV_HeartBeat.system_status=4;
//       MAV_HeartBeat.mavlink_version=1;
//       MAV_RxMsg::Send(sizeof(MAV_HeartBeat), MAV_Seq++, MAV_SysID, MAV_COMP_ID_ADSB, MAV_ID_HEARTBEAT, (const uint8_t *)&MAV_HeartBeat, GPS_UART_Write);
//     }
// #endif
#ifdef DEBUG_PRINT
    // char Line[128];
    Line[0]='0'+RF_TxFIFO.Full(); Line[1]=' ';                  // print number of packets in the TxFIFO
    RelayQueue.Print(Line+2);                                   // dump the relay queue
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, Line);
    xSemaphoreGive(CONS_Mutex);
#endif
    // Parameters.WriteHeader(InfoPacket.Packet);
    InfoPacket.Packet.HeaderWord=0;
    InfoPacket.Packet.Header.Address    = Parameters.Address;    // set address
    InfoPacket.Packet.Header.AddrType   = Parameters.AddrType;   // address-type
    InfoPacket.Packet.Header.Other=1;
    InfoPacket.Packet.calcAddrParity();                          // parity of (part of) the header

    StatPacket.Packet.HeaderWord=0;
    StatPacket.Packet.Header.Address    = Parameters.Address;    // set address
    StatPacket.Packet.Header.AddrType   = Parameters.AddrType;   // address-type
    StatPacket.Packet.Header.Other=1;
    StatPacket.Packet.calcAddrParity();                          // parity of (part of) the header
    StatPacket.Packet.Status.Hardware=HARDWARE_ID;
    StatPacket.Packet.Status.Firmware=SOFTWARE_ID;

    ReadStatus(StatPacket);
    XorShift32(RX_Random);
    if( ((RX_Random&0x1F)==0) && (RF_TxFIFO.Full()<2) )
    { OGN_TxPacket *StatusPacket = RF_TxFIFO.getWrite();
     *StatusPacket = StatPacket;
      StatusPacket->Packet.Whiten();
      StatusPacket->calcFEC();
      RF_TxFIFO.Write(); }

    while(RF_TxFIFO.Full()<2)
    { OGN_TxPacket *RelayPacket = RF_TxFIFO.getWrite();
      if(!GetRelayPacket(RelayPacket)) break;
      // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      // Format_String(CONS_UART_Write, "Relayed: ");
      // Format_Hex(CONS_UART_Write, RelayPacket->Packet.HeaderWord);
      // CONS_UART_Write('\r'); CONS_UART_Write('\n');
      // xSemaphoreGive(CONS_Mutex);
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "TxFIFO: ");
      Format_Hex(CONS_UART_Write, RelayPacket->Packet.HeaderWord);
      CONS_UART_Write('\r'); CONS_UART_Write('\n');
      xSemaphoreGive(CONS_Mutex);
#endif
      RF_TxFIFO.Write();
    }
    CleanRelayQueue(SlotTime);

  }


}
