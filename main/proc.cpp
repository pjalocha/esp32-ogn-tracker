#include <stdint.h>

#include "hal.h"                      // Hardware Abstraction Layer

#include "proc.h"                     // PROC task: decode/correct received packets
#include "ctrl.h"                     // CTRL task:
#include "log.h"                      // LOG task: packet logging

#include "ogn.h"                      // OGN packet structures, encoding/decoding/etc.

#include "rf.h"                       // RF task: transmission and reception of radio packets
#include "gps.h"                      // GPS task: get own time and position, set the GPS baudrate and navigation mode

#include "fifo.h"

#ifdef WITH_FLASHLOG                  // log own track to unused Flash pages (STM32 only)
#include "flashlog.h"
#endif
#ifdef WITH_SDLOG
#include "sdlog.h"
#endif

#ifdef WITH_SOUND
#include "sound.h"
#endif

#ifdef WITH_GDL90
#include "gdl90.h"
GDL90_HEARTBEAT GDL_HEARTBEAT;
GDL90_REPORT GDL_REPORT;
#endif

#ifdef WITH_LOOKOUT                   // traffic awareness and warnings
#include "lookout.h"
LookOut Look;
#ifdef WITH_SOUND
const char *Dir[16] = { "N", "NNE", "NE", "NEE", "E", "SEE", "SE", "SSE", "S", "SSW", "SW", "SWW", "W", "NWW", "NW", "NNW" };
const char *RelDir[8] = { "A", "AR", "R", "BR", "B", "BL", "L", "AL" };

void Sound_TrafficWarn(const LookOut_Target *Tgt)
{ if(!Tgt) return;
  uint8_t WarnLevel = Tgt->WarnLevel;
  // uint16_t DistMargin = Tgt->DistMargin; // [0.5m]
  uint16_t TimeMargin = Tgt->TimeMargin; // [0.5s]
  uint16_t HorDist = Tgt->HorDist;       // [0.5]
  uint16_t Bearing = Tgt->getBearing();  //
  int16_t RelBearing = Look.getRelBearing(Tgt);
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "Traffic: ");
  CONS_UART_Write('#');
  CONS_UART_Write('0'+WarnLevel);
  CONS_UART_Write(' ');
  // Format_Hex(CONS_UART_Write, Bearing);
  // CONS_UART_Write(' ');
  uint16_t DirIdx = (Bearing+0x800)>>12; DirIdx&=0x0F;
  Format_String(CONS_UART_Write, Dir[DirIdx]);
  CONS_UART_Write(' ');
  uint16_t RelDirIdx = (RelBearing+0x1000)>>13; RelDirIdx&=0x07;
  Format_String(CONS_UART_Write, RelDir[RelDirIdx]);
  CONS_UART_Write(' ');
  Format_UnsDec(CONS_UART_Write, (uint16_t)(HorDist/2));
  Format_String(CONS_UART_Write, "m ");
  Format_UnsDec(CONS_UART_Write, (uint16_t)(TimeMargin/2));
  Format_String(CONS_UART_Write, "s\n");
  xSemaphoreGive(CONS_Mutex);
  // SoundMsg("Traffic");
}
#endif
#endif

// static uint16_t PrevBattVolt = 0;     // [mV]
static Delay<uint16_t, 32> BatteryVoltagePipe;
uint32_t BatteryVoltage = 0;          // [1/256 mV] low-pass filtered battery voltage
 int32_t BatteryVoltageRate = 0;      // [1/256 mV/sec] low-pass filtered battery voltage rise/drop rate

static char           Line[128];      // for printing out to the console, etc.

static LDPC_Decoder     Decoder;      // decoder and error corrector for the OGN Gallager/LDPC code

// #define DEBUG_PRINT

// =======================================================================================================================================

#ifdef WITH_LOG

static int SPIFFSlog(OGN_RxPacket<OGN_Packet> *Packet, uint32_t Time)
{ OGN_LogPacket<OGN_Packet> *LogPacket = LOG_FIFO.getWrite(); if(LogPacket==0) return -1; // allocate new packet in the LOG_FIFO
  LogPacket->Packet = Packet->Packet;                                                     // copy the packet
  LogPacket->Flags=0x80;
  LogPacket->setTime(Time);
  LogPacket->setCheck();
  LOG_FIFO.Write();                                                                       // finalize the write
  return 1; }

static int SPIFFSlog(OGN_TxPacket<OGN_Packet> *Packet, uint32_t Time)
{ OGN_LogPacket<OGN_Packet> *LogPacket = LOG_FIFO.getWrite(); if(LogPacket==0) return -1;
  LogPacket->Packet = Packet->Packet;
  LogPacket->Flags=0x00;
  LogPacket->setTime(Time);
  LogPacket->setCheck();
  LOG_FIFO.Write();
  return 1; }

#endif // WITH_LOG

// ---------------------------------------------------------------------------------------------------------------------------------------

// #ifdef WITH_ESP32
// const uint8_t RelayQueueSize = 32;
// #else
// const uint8_t RelayQueueSize = 16;
// #endif

OGN_PrioQueue<OGN_Packet, RelayQueueSize> RelayQueue;       // received packets and candidates to be relayed

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

static bool GetRelayPacket(OGN_TxPacket<OGN_Packet> *Packet)      // prepare a packet to be relayed
{ if(RelayQueue.Sum==0) return 0;                     // if no packets in the relay queue
  XorShift32(RX_Random);                              // produce a new random number
  uint8_t Idx=RelayQueue.getRand(RX_Random);          // get weight-random packet from the relay queue
  if(RelayQueue.Packet[Idx].Rank==0) return 0;        // should not happen ...
  memcpy(Packet->Packet.Byte(), RelayQueue[Idx]->Byte(), OGN_Packet::Bytes); // copy the packet
  Packet->Packet.Header.Relay=1;                      // increment the relay count (in fact we only do single relay)
  // Packet->Packet.calcAddrParity();
  Packet->Packet.Whiten(); Packet->calcFEC();         // whiten and calc. the FEC code => packet ready for transmission
  // PrintRelayQueue(Idx);  // for debug
  RelayQueue.decrRank(Idx);                           // reduce the rank of the packet selected for relay
  return 1; }

static void CleanRelayQueue(uint32_t Time, uint32_t Delay=20) // remove "old" packets from the relay queue
{ RelayQueue.cleanTime((Time-Delay)%60); }            // remove packets 20(default) seconds into the past

// ---------------------------------------------------------------------------------------------------------------------------------------

static uint16_t InfoParmIdx = 0;            // the round-robin index to info records in info packets

static int ReadInfo(OGN1_Packet &Packet)
{ Packet.clrInfo();
  uint8_t ParmIdx;
  for( ParmIdx=InfoParmIdx; ; )
  { const char *Parm = Parameters.InfoParmValue(ParmIdx);
    if(Parm)
    { // printf("Parm[%d]=%s\n", ParmIdx, Parm);
      if(Parm[0])
      { int Add=Packet.addInfo(Parm, ParmIdx); if(Add==0) break; }
    }
    ParmIdx++; if(ParmIdx>=Parameters.InfoParmNum) ParmIdx=0;
    if(ParmIdx==InfoParmIdx) break;
  }
  InfoParmIdx = ParmIdx;
  Packet.setInfoCheck();
  return Packet.Info.DataChars; }                                      // zero => no info parameters were stored

// ---------------------------------------------------------------------------------------------------------------------------------------

static void ReadStatus(OGN_Packet &Packet)
{
// #ifdef WITH_JACEK
/*
  xSemaphoreTake(ADC1_Mutex, portMAX_DELAY);
  uint16_t MCU_Vtemp  = ADC_Read_MCU_Vtemp();                                // T = 25+(V25-Vtemp)/Avg_Slope; V25=1.43+/-0.1V, Avg_Slope=4.3+/-0.3mV/degC
  uint16_t MCU_Vref   = ADC_Read_MCU_Vref();                                 // VDD = 1.2*4096/Vref
           MCU_Vtemp += ADC_Read_MCU_Vtemp();                                // measure again and average
           MCU_Vref  += ADC_Read_MCU_Vref();
#ifdef WITH_BATT_SENSE
  uint16_t Vbatt       = ADC_Read_Vbatt();                                   // measure voltage on PB1
           Vbatt      += ADC_Read_Vbatt();
#endif
  xSemaphoreGive(ADC1_Mutex);
   int16_t MCU_Temp = -999;                                                  // [0.1degC]
  uint16_t MCU_VCC = 0;                                                      // [0.01V]
  if(MCU_Vref)
  { MCU_Temp = 250 + ( ( ( (int32_t)1430 - ((int32_t)1200*(int32_t)MCU_Vtemp+(MCU_Vref>>1))/MCU_Vref )*(int32_t)37 +8 )>>4); // [0.1degC]
    MCU_VCC  = ( ((uint32_t)240<<12)+(MCU_Vref>>1))/MCU_Vref; }              // [0.01V]
  Packet.EncodeVoltage(((MCU_VCC<<4)+12)/25);                     // [1/64V]  write supply voltage to the status packet
#ifdef WITH_BATT_SENSE
  if(MCU_Vref)
    Packet.EncodeVoltage(((int32_t)154*(int32_t)Vbatt+(MCU_Vref>>1))/MCU_Vref); // [1/64V] battery voltage assuming 1:1 divider form battery to PB1
#endif
*/

  // Packet.clrHumidity();
#ifdef WITH_STM32
#ifdef WITH_JACEK
  uint16_t MCU_Vbatt   = Measure_Vbatt();                                    // [0.001V]
  Packet.EncodeVoltage(((MCU_Vbatt<<3)+62)/125);                             // [1/64V]
  if(MCU_Vbatt<3600)
  { uint16_t FlashLen = 3600-MCU_Vbatt; if(FlashLen>250) FlashLen=250;
    LED_BAT_Flash(FlashLen); }
#else // WITH_JACEK
  uint16_t MCU_VCC   = Measure_MCU_VCC();                                    // [0.001V]
  Packet.EncodeVoltage(((MCU_VCC<<3)+62)/125);                               // [1/64V]
#endif
  int16_t MCU_Temp  = Measure_MCU_Temp();                                    // [0.1degC]
#endif

#ifdef WITH_ESP32
  // Packet.clrTemperature();

  uint16_t BattVolt = BatterySense();                                        // [mV] measure battery voltage
  if(BatteryVoltage>0)
  { // int32_t PrevVolt = BatteryVoltage;
    int32_t Rate = ((uint32_t)BattVolt<<8) - BatteryVoltage;                 // [1/256 mV]
    BatteryVoltage += (Rate+32)>>6;                                          // [1/256 mV] low-pass battery voltage measurement
    uint16_t Volt = (BatteryVoltage+16)>>5;
    int16_t Diff = Volt-BatteryVoltagePipe.Input(Volt);
    BatteryVoltageRate = Diff; }
    // BatteryVoltageRate = BatteryVoltage - PrevVolt; }
    // int16_t BattVoltDiff = BattVolt - PrevBattVolt;
    // int32_t Diff = ((int32_t)BattVoltDiff<<8) - BatteryVoltageRate;
    // BatteryVoltageRate += Diff/256; }
    // BatteryVoltageRate = (((int32_t)BattVoltDiff<<8) + BatteryVoltageRate*255 + 128)>>8; }
  else
  { BatteryVoltage = BattVolt<<8;
    // PrevBattVolt = BattVolt;
    BatteryVoltagePipe.Clear(BattVolt<<3);
    BatteryVoltageRate = 0; }
  // PrevBattVolt = BattVolt;
  Packet.EncodeVoltage(((BatteryVoltage>>2)+500)/1000);            // [1/64V] encode into the status packet

#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "Battery: ");
  // Format_UnsDec(CONS_UART_Write, BattVolt);
  // CONS_UART_Write(' ');
  // Format_UnsDec(CONS_UART_Write, PrevBattVolt);
  // CONS_UART_Write(' ');
  // Format_UnsDec(CONS_UART_Write, BatteryVoltage, 2);
  // CONS_UART_Write(' ');
  // Format_SignDec(CONS_UART_Write, BatteryVoltageRate, 2);
  // CONS_UART_Write(' ');
  Format_UnsDec(CONS_UART_Write, (10*BatteryVoltage+128)>>8, 5, 4);
  Format_String(CONS_UART_Write, "V ");
  Format_SignDec(CONS_UART_Write, (600*BatteryVoltageRate+128)>>8, 3, 1);
  Format_String(CONS_UART_Write, "mV/min\n");
  xSemaphoreGive(CONS_Mutex);
#endif
#endif

  if(Packet.Status.Pressure==0) Packet.EncodeTemperature(TRX.chipTemp*10); // [0.1degC]
  Packet.Status.RadioNoise = TRX.averRSSI;                         // [-0.5dBm] write radio noise to the status packet

  uint8_t TxPower = Parameters.getTxPower()-4;
  if(TxPower>15) TxPower=15;
  Packet.Status.TxPower = TxPower;

  uint16_t RxRate = RX_OGN_Count64+1;
  uint8_t RxRateLog2=0; RxRate>>=1; while(RxRate) { RxRate>>=1; RxRateLog2++; }
  Packet.Status.RxRate = RxRateLog2;

  if(Parameters.Verbose)
  { uint8_t Len=0;
    Len+=Format_String(Line+Len, "$POGNR,");                                  // NMEA report: radio status
    Len+=Format_UnsDec(Line+Len, RF_FreqPlan.Plan);                           // which frequency plan
    Line[Len++]=',';
    Len+=Format_UnsDec(Line+Len, RX_OGN_Count64);                             // number of OGN packets received
    Line[Len++]=',';
    Line[Len++]=',';
    Len+=Format_SignDec(Line+Len, -5*TRX.averRSSI, 2, 1);                     // average RF level (over all channels)
    Line[Len++]=',';
    Len+=Format_UnsDec(Line+Len, (uint16_t)TX_Credit);
    Line[Len++]=',';
    Len+=Format_SignDec(Line+Len, (int16_t)TRX.chipTemp);                     // the temperature of the RF chip
    Line[Len++]=',';
    // Len+=Format_SignDec(Line+Len, MCU_Temp, 2, 1);
    Line[Len++]=',';
#ifdef WITH_STM32
#ifdef WITH_JACEK
    Len+=Format_UnsDec(Line+Len, (MCU_Vbatt+5)/10, 3, 2);
#else
    // Len+=Format_UnsDec(Line+Len, (MCU_VCC+5)/10, 3, 2);
#endif
#endif

    Len+=NMEA_AppendCheckCRNL(Line, Len);                                    // append NMEA check-sum and CR+NL
    // LogLine(Line);
    // if(CONS_UART_Free()>=128)
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, Line, 0, Len);                          // send the NMEA out to the console
      xSemaphoreGive(CONS_Mutex); }
#ifdef WITH_SDLOG
    if(Log_Free()>=128)
    { xSemaphoreTake(Log_Mutex, portMAX_DELAY);
      Format_String(Log_Write, Line, 0, Len);                                // send the NMEA out to the log file
      xSemaphoreGive(Log_Mutex); }
#endif
  }
}

// static void ReadStatus(OGN_TxPacket<OGN_Packet> &StatPacket)
// { ReadStatus(StatPacket.Packet); }

#ifndef WITH_LOOKOUT                                    // with LookOut the PFLAU is produced inside LookOut
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
#endif

// ---------------------------------------------------------------------------------------------------------------------------------------

static void ProcessRxPacket(OGN_RxPacket<OGN_Packet> *RxPacket, uint8_t RxPacketIdx, uint32_t RxTime)  // process every (correctly) received packet
{ int32_t LatDist=0, LonDist=0; uint8_t Warn=0;
  if( RxPacket->Packet.Header.NonPos || RxPacket->Packet.Header.Encrypted ) return ;   // status packet or encrypted: ignore
  uint8_t MyOwnPacket = ( RxPacket->Packet.Header.Address  == Parameters.Address  )
                     && ( RxPacket->Packet.Header.AddrType == Parameters.AddrType );
  if(MyOwnPacket) return;                                                             // don't process my own (relayed) packets
  bool DistOK = RxPacket->Packet.calcDistanceVector(LatDist, LonDist, GPS_Latitude, GPS_Longitude, GPS_LatCosine)>=0;
  if(DistOK)
  { RxPacket->calcRelayRank(GPS_Altitude/10);                                         // calculate the relay-rank (priority for relay)
    OGN_RxPacket<OGN_Packet> *PrevRxPacket = RelayQueue.addNew(RxPacketIdx);
#ifdef WITH_POGNT
    { uint8_t Len=RxPacket->WritePOGNT(Line);                                           // print on the console as $POGNT
      if(Parameters.Verbose)
      { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, Line, 0, Len);
        xSemaphoreGive(CONS_Mutex); }
#ifdef WITH_SDLOG
      if(Log_Free()>=128)
      { xSemaphoreTake(Log_Mutex, portMAX_DELAY);
        Format_String(Log_Write, Line, 0, Len);
        xSemaphoreGive(Log_Mutex); }
#endif
    }
#endif
//     Len=RxPacket->Packet.WriteAPRS(Line, RxTime);                                     // print on the console as APRS message
//     xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
//     Format_String(CONS_UART_Write, Line, 0, Len);
//     xSemaphoreGive(CONS_Mutex);
#ifdef WITH_LOOKOUT
    const LookOut_Target *Tgt=Look.ProcessTarget(RxPacket->Packet);                   // process the received target postion
    if(Tgt) Warn=Tgt->WarnLevel;                                                      // remember warning level of this target
#ifdef WITH_GDL90
    if(Tgt)
    { Look.Write(GDL_REPORT, Tgt);                                                    // produce GDL90 report for this target
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      GDL_REPORT.Send(CONS_UART_Write, 20);                                           // transmit as traffic position report (not own-ship)
      xSemaphoreGive(CONS_Mutex); }
#endif
#ifdef WITH_BEEPER
    if(KNOB_Tick>12) Play(Play_Vol_1 | Play_Oct_2 | (7+2*Warn), 3+16*Warn);
#endif
#else // if not WITH_LOOKOUT
#ifdef WITH_BEEPER
    if(KNOB_Tick>12) Play(Play_Vol_1 | Play_Oct_2 | 7, 3);                            // if Knob>12 => make a beep for every received packet
#endif
#endif // WITH_LOOKOUT
#ifdef WITH_LOG
     bool Signif = PrevRxPacket!=0;
     if(!Signif) Signif=OGN_isSignif(&(RxPacket->Packet), &(PrevRxPacket->Packet));
     if(Signif) SPIFFSlog(RxPacket, RxTime);                                          // log only significant packets
#endif
#ifdef WITH_PFLAA
    if( Parameters.Verbose    // print PFLAA on the console for received packets
#ifdef WITH_LOOKOUT
    && (!Tgt)
#endif
    )
    { uint8_t Len=RxPacket->WritePFLAA(Line, Warn, LatDist, LonDist, RxPacket->Packet.DecodeAltitude()-GPS_Altitude/10);
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, Line, 0, Len);
      xSemaphoreGive(CONS_Mutex);
#ifdef WITH_SDLOG
    if(Log_Free()>=128)
    { xSemaphoreTake(Log_Mutex, portMAX_DELAY);
      Format_String(Log_Write, Line, 0, Len);                                // send the NMEA out to the log file
      xSemaphoreGive(Log_Mutex); }
#endif
    }
#endif
#ifdef WITH_MAVLINK
   MAV_ADSB_VEHICLE MAV_RxReport;
   RxPacket->Packet.Encode(&MAV_RxReport);
   MAV_RxMsg::Send(sizeof(MAV_RxReport), MAV_Seq++, MAV_SysID, MAV_COMP_ID_ADSB, MAV_ID_ADSB_VEHICLE, (const uint8_t *)&MAV_RxReport, GPS_UART_Write);
   // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
   // MAV_RxMsg::Send(sizeof(MAV_RxReport), MAV_Seq++, MAV_SysID, MAV_COMP_ID_ADSB, MAV_ID_ADSB_VEHICLE, (const uint8_t *)&MAV_RxReport, CONS_UART_Write);
   // xSemaphoreGive(CONS_Mutex);
#endif
  }
}

static void DecodeRxPacket(RFM_RxPktData *RxPkt)
{
  uint8_t RxPacketIdx  = RelayQueue.getNew();                   // get place for this new packet
  OGN_RxPacket<OGN_Packet> *RxPacket = RelayQueue[RxPacketIdx];
  // PrintRelayQueue(RxPacketIdx);                              // for debug
  // RxPacket->RxRSSI=RxPkt.RSSI;
  // TickType_t ExecTime=xTaskGetTickCount();

  { RX_OGN_Packets++;
    uint8_t Check = RxPkt->Decode(*RxPacket, Decoder);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "DecodeRxPkt: ");
    Format_Hex(CONS_UART_Write, RxPacket->Packet.HeaderWord);
    CONS_UART_Write(' ');
    Format_UnsDec(CONS_UART_Write, (uint16_t)Check, 2);
    CONS_UART_Write('/');
    Format_UnsDec(CONS_UART_Write, (uint16_t)RxPacket->RxErr);
    Format_String(CONS_UART_Write, "e\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    if( (Check==0) && (RxPacket->RxErr<15) )                     // what limit on number of detected bit errors ?
    { RxPacket->Packet.Dewhiten();
      ProcessRxPacket(RxPacket, RxPacketIdx, RxPkt->Time); }
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

#ifdef WITH_LOOKOUT
  Look.Clear();
#endif

  OGN_TxPacket<OGN_Packet> PosPacket;                                  // position packet
  OGN_Packet        PrevLoggedPacket;                                  // most recent logged packet
  uint32_t                 PosTime=0;                                  // [sec] when the position was recorded
  OGN_TxPacket<OGN_Packet> StatPacket;                                 // status report packet
  // OGN_TxPacket<OGN_Packet> InfoPacket;                                 // information packet

  for( ; ; )
  { vTaskDelay(1);

    RFM_RxPktData *RxPkt = RF_RxFIFO.getRead();                         // check for new received packets
    if(RxPkt)                                                           // if there is a new received packet
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
      RF_RxFIFO.Read(); }                                               // remove this packet from the queue

    static uint32_t PrevSlotTime=0;                                     // remember previous time slot to detect a change
    uint32_t SlotTime = TimeSync_Time();                                // time slot
    if(TimeSync_msTime()<340) SlotTime--;                               // lasts up to 0.300sec after the PPS

    if(SlotTime==PrevSlotTime) continue;                                // stil same time slot, go back to RX processing
    PrevSlotTime=SlotTime;                                              // new slot started
                                                                        // this part of the loop is executed only once per slot-time
    uint8_t BestIdx; int16_t BestResid;
#ifdef WITH_MAVLINK
    GPS_Position *Position = GPS_getPosition(BestIdx, BestResid, (SlotTime-1)%60, 0);
#else
    GPS_Position *Position = GPS_getPosition(BestIdx, BestResid, SlotTime%60, 0); // get GPS position which isReady
#endif
    // GPS_Position *Position = GPS_getPosition();
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "getPos(");
    Format_UnsDec(CONS_UART_Write, SlotTime%60, 2);
    Format_String(CONS_UART_Write, ") => ");
    Format_UnsDec(CONS_UART_Write, (uint16_t)BestIdx);
    CONS_UART_Write(':');
    Format_SignDec(CONS_UART_Write, BestResid, 3, 2);
    Format_String(CONS_UART_Write, "s\n");
    xSemaphoreGive(CONS_Mutex);
#endif
#ifdef WITH_GDL90
    GDL_HEARTBEAT.Clear();
    GDL_HEARTBEAT.Initialized=1;
    if(Position)
    { if(Position->isTimeValid())
      { GDL_HEARTBEAT.UTCvalid=1;
        GDL_HEARTBEAT.setTimeStamp(SlotTime);
        if(Position->isValid()) GDL_HEARTBEAT.PosValid = 1; }
    }
    GDL_REPORT.Clear();
    GDL_REPORT.setAddress(Parameters.Address);
    GDL_REPORT.setAddrType(Parameters.AddrType!=1);
    GDL_REPORT.setAcftType(Parameters.AcftType);
    if(Parameters.Reg[0]) GDL_REPORT.setAcftCall(Parameters.Reg);
                     // else GDL_REPORT.setAcftCall();
    if(Position && Position->isValid()) Position->Encode(GDL_REPORT);
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    GDL_HEARTBEAT.Send(CONS_UART_Write);
    GDL_REPORT.Send(CONS_UART_Write);
    xSemaphoreGive(CONS_Mutex);
#endif
    if(Position) Position->EncodeStatus(StatPacket.Packet);             // encode GPS altitude and pressure/temperature/humidity
      else { StatPacket.Packet.Status.FixQuality=0; StatPacket.Packet.Status.Satellites=0; } // or lack of the GPS lock
    { uint8_t SatSNR = (GPS_SatSNR+2)/4;
      if(SatSNR>8) { SatSNR-=8; if(SatSNR>31) SatSNR=31; }
              else { SatSNR=0; }
      StatPacket.Packet.Status.SatSNR = SatSNR; }
    if( Position && Position->isReady && (!Position->Sent) && Position->isValid() )
    { int16_t AverSpeed=GPS_AverageSpeed();                             // [0.1m/s] average speed, including the vertical speed
      if(Parameters.FreqPlan==0)
        RF_FreqPlan.setPlan(Position->Latitude, Position->Longitude);     // set the frequency plan according to the GPS position
      else RF_FreqPlan.setPlan(Parameters.FreqPlan);
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60);
      CONS_UART_Write('.');
      Format_UnsDec(CONS_UART_Write, TimeSync_msTime(), 3);
      Format_String(CONS_UART_Write, " -> Sent\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      PosTime=Position->getUnixTime();
      PosPacket.Packet.HeaderWord=0;
      PosPacket.Packet.Header.Address    = Parameters.Address;         // set address
      PosPacket.Packet.Header.AddrType   = Parameters.AddrType;        // address-type
#ifdef WITH_ENCRYPT
      if(Parameters.Encrypt)                                           // if position encryption is requested
      { PosPacket.Packet.Header.Encrypted = 1; }                       // then set the flg in the header
#endif
      PosPacket.Packet.calcAddrParity();                               // parity of (part of) the header
      if(BestResid==0) Position->Encode(PosPacket.Packet);             // encode position/altitude/speed/etc. from GPS position
      else                                                             // extrapolate the position when if not at an exact UTC second
      { while(BestResid>=50) BestResid-=100;                           // remove full seconds
        Position->Encode(PosPacket.Packet, BestResid); }
      PosPacket.Packet.Position.AcftType = Parameters.AcftType;        // aircraft-type
      PosPacket.Packet.Position.Stealth = 0; // Parameters.Stealth;
#ifdef DEBUG_PRINT
      { uint8_t Len=PosPacket.Packet.WriteAPRS(Line, PosTime);           // print on the console as APRS message
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, Line, 0, Len);
        xSemaphoreGive(CONS_Mutex); }
#endif
      OGN_TxPacket<OGN_Packet> *TxPacket = RF_TxFIFO.getWrite();
      TxPacket->Packet = PosPacket.Packet;                             // copy the position packet to the TxFIFO

#ifdef WITH_ENCRYPT
      if(Parameters.Encrypt) TxPacket->Packet.Encrypt(Parameters.EncryptKey); // if encryption is requested then encrypt
                        else TxPacket->Packet.Whiten();                       // otherwise only whiten
#else
      TxPacket->Packet.Whiten();
#endif
      TxPacket->calcFEC();                  // whiten and calculate FEC code
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60, 2);
      CONS_UART_Write('.');
      Format_UnsDec(CONS_UART_Write, TimeSync_msTime(), 3);
      Format_String(CONS_UART_Write, " TxFIFO <- ");
      Format_Hex(CONS_UART_Write, TxPacket->Packet.HeaderWord);
      CONS_UART_Write('\r'); CONS_UART_Write('\n');
      xSemaphoreGive(CONS_Mutex);
#endif
      XorShift32(RX_Random);
      if( (AverSpeed>10) || ((RX_Random&0x3)==0) )                        // send only some positions if the speed is less than 1m/s
        RF_TxFIFO.Write();                                                // complete the write into the TxFIFO
      Position->Sent=1;
#ifdef WITH_FANET
    if( (SlotTime&0x07)==(RX_Random&0x07) )                                              // every 8sec
    { FANET_Packet *FNTpkt = FNT_TxFIFO.getWrite();
      FNTpkt->setAddress(Parameters.Address);
      Position->EncodeAirPos(*FNTpkt, Parameters.AcftType, !Parameters.Stealth);
      XorShift32(RX_Random);
      FNT_TxFIFO.Write(); }
#endif
#ifdef WITH_LOOKOUT
      const LookOut_Target *Tgt=Look.ProcessOwn(PosPacket.Packet);        // process own position, get the most dangerous target
#ifdef WITH_PFLAA
      if(Parameters.Verbose)
      { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Look.WritePFLA(CONS_UART_Write);                                  // produce PFLAU and PFLAA for all tracked targets
        xSemaphoreGive(CONS_Mutex);
#ifdef WITH_SDLOG
       if(Log_Free()>=512)
       { xSemaphoreTake(Log_Mutex, portMAX_DELAY);
         Look.WritePFLA(Log_Write);
         xSemaphoreGive(Log_Mutex); }
#endif
      }
#else
      if(Parameters.Verbose)
      { uint8_t Len=Look.WritePFLAU(Line);                                // $PFLAU, overall status
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, Line, 0, Len);
        xSemaphoreGive(CONS_Mutex);
#ifdef WITH_SDLOG
        if(Log_Free()>=128)
        { xSemaphoreTake(Log_Mutex, portMAX_DELAY);
          Format_String(Log_Write, Line, 0, Len);                                // send the NMEA out to the log file
          xSemaphoreGive(Log_Mutex); }
#endif
      }
#endif // WITH_PFLAA
      uint8_t Warn = 0;
      if(Tgt) Warn = Tgt->WarnLevel;                                       // what is the warning level ?
      if( (Warn>0) /* && (AverSpeed>=10) */ )                                    // if non-zero warning level and we seem to be moving
      { // int16_t RelBearing = Look.getRelBearing(Tgt);                      // relative bearing to the Target
        // int8_t Bearing = (12*(int32_t)RelBearing+0x8000)>>16;              // [-12..+12]
#ifdef WITH_BEEPER                                                         // make the sound according to the level
        if(Warn<=1)
        { if(KNOB_Tick>8)
          { Play(Play_Vol_1 | Play_Oct_1 | 4, 200); }
        }
        else if(Warn<=2)
        { if(KNOB_Tick>4)
          { Play(Play_Vol_3 | Play_Oct_1 | 8, 150); Play(Play_Oct_1 | 8, 150);
            Play(Play_Vol_3 | Play_Oct_1 | 8, 150); }
        }
        else if(Warn<=3)
        { if(KNOB_Tick>2)
          { Play(Play_Vol_3 | Play_Oct_1 |11, 100); Play(Play_Oct_1 |11, 100);
            Play(Play_Vol_3 | Play_Oct_1 |11, 100); Play(Play_Oct_1 |11, 100);
            Play(Play_Vol_3 | Play_Oct_1 |11, 100); }
        }

#endif // WITH_BEEPER
#ifdef WITH_SOUND
        Sound_TrafficWarn(Tgt);
#endif
      }
#else  // WITH_LOOKOUT
      if(Parameters.Verbose)
      { uint8_t Len=Look.WritePFLAU(Line);                                // $PFLAU, overall status
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, Line, 0, Len);
        xSemaphoreGive(CONS_Mutex);
#ifdef WITH_SDLOG
        if(Log_Free()>=128)
        { xSemaphoreTake(Log_Mutex, portMAX_DELAY);
          Format_String(Log_Write, Line, 0, Len);                                // send the NMEA out to the log file
          xSemaphoreGive(Log_Mutex); }
#endif
      }
#endif // WITH_LOOKOUT
#ifdef WITH_FLASHLOG
      bool Written=FlashLog_Process(PosPacket.Packet, PosTime);
      // if(Written)
      // { uint8_t Len=FlashLog_Print(Line);
      //   xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      //   Format_String(CONS_UART_Write, Line);
      //   xSemaphoreGive(CONS_Mutex);
      // }
#endif // WITH_FLASHLOG
#ifdef WITH_LOG
      bool isSignif = OGN_isSignif(&(PosPacket.Packet), &PrevLoggedPacket);
      if(isSignif)
      { SPIFFSlog(&PosPacket, PosTime);
        PrevLoggedPacket = PosPacket.Packet; }
#endif
    } else // if GPS position is not complete, contains no valid position, etc.
    { if((SlotTime-PosTime)>=30) { PosPacket.Packet.Position.Time=0x3F; } // if no valid position for more than 30 seconds then set the time as unknown for the transmitted packet
      OGN_TxPacket<OGN_Packet> *TxPacket = RF_TxFIFO.getWrite();
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
#ifdef DEBUG_PRINT
    // char Line[128];
    Line[0]='0'+RF_TxFIFO.Full(); Line[1]=' ';                  // print number of packets in the TxFIFO
    RelayQueue.Print(Line+2);                                   // dump the relay queue
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, Line);
    xSemaphoreGive(CONS_Mutex);
#endif

#ifdef WITH_FANET
    if(Parameters.Pilot[0] && (SlotTime&0xFF)==(RX_Random&0xFF) )              // every 256sec
    { FANET_Packet *FNTpkt = FNT_TxFIFO.getWrite();
      FNTpkt->setAddress(Parameters.Address);
      FNTpkt->setName(Parameters.Pilot);
      XorShift32(RX_Random);
      FNT_TxFIFO.Write(); }
#endif

    StatPacket.Packet.HeaderWord=0;
    StatPacket.Packet.Header.Address    = Parameters.Address;    // set address
    StatPacket.Packet.Header.AddrType   = Parameters.AddrType;   // address-type
    StatPacket.Packet.Header.NonPos=1;
    StatPacket.Packet.calcAddrParity();                          // parity of (part of) the header
    StatPacket.Packet.Status.Hardware=HARDWARE_ID;
    StatPacket.Packet.Status.Firmware=SOFTWARE_ID;

    ReadStatus(StatPacket.Packet);                               // read status data and put them into the StatPacket
    XorShift32(RX_Random);                                       // generate a new random number
    if( ((RX_Random&0x0F)==0) && (RF_TxFIFO.Full()<2) )          // decide whether to transmit the status/info packet
    { OGN_TxPacket<OGN_Packet> *StatusPacket = RF_TxFIFO.getWrite(); // ask for space in the Tx queue
      uint8_t doTx=1;
      if(RX_Random&0x10)                                         // decide to transmit info packet, not status
      { doTx=ReadInfo(StatPacket.Packet); }                      // and overwrite the StatPacket with the Info data
      if(doTx)
      {
#ifdef WITH_LOG
        SPIFFSlog(&StatPacket, PosTime);                         // log the status packet
#endif
       *StatusPacket = StatPacket;                               // copy status packet into the Tx queue
        StatusPacket->Packet.Whiten();                           // whiten for transmission
        StatusPacket->calcFEC();                                 // calc. the FEC code
        RF_TxFIFO.Write();                                       // finalize write into the Tx queue
      }
    }

    while(RF_TxFIFO.Full()<2)
    { OGN_TxPacket<OGN_Packet> *RelayPacket = RF_TxFIFO.getWrite();
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
