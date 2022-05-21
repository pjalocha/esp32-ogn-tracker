
#include <stdint.h>
#include <stdlib.h>

#include "hal.h"
#include "gps.h"
#include "ctrl.h"
#include "nmea.h"
#include "ubx.h"
#ifdef WITH_MAVLINK
#include "mavlink.h"
#include "atmosphere.h"
#endif
#ifdef WITH_SDLOG
#include "sdlog.h"
#endif

#include "rf.h"
#include "ogn.h"

// #include "ctrl.h"
// #include "knob.h"

#include "lowpass2.h"

// #define DEBUG_PRINT

// #ifdef DEBUG_PRINT
static char Line[128];
static void CONS_HexDump(char Byte) { Format_Hex(CONS_UART_Write, (uint8_t)Byte); }
// #endif

// ----------------------------------------------------------------------------

// void Debug_Print(uint8_t Byte) { while(!UART1_TxEmpty()) taskYIELD(); UART1_TxChar(Byte); }

static NMEA_RxMsg  NMEA;                 // NMEA sentences catcher
#ifdef WITH_GPS_UBX
static UBX_RxMsg   UBX;                  // UBX messages catcher
#endif
#ifdef WITH_MAVLINK
static MAV_RxMsg   MAV;                  // MAVlink message catcher
#endif

uint16_t GPS_PosPeriod = 0;                    // [mss] time between succecive GPS readouts

// uint8_t  GPS_PowerMode = 2;                    // 0=shutdown, 1=reduced power, 2=normal

const  uint8_t PosPipeIdxMask = GPS_PosPipeSize-1;
uint8_t      GPS_PosIdx;                   // Pipe index, increments with every GPS position received
GPS_Position GPS_Pos[GPS_PosPipeSize];     // GPS position pipe

static   TickType_t PPS_Tick;              // [msec] System Tick when the PPS arrived
static   TickType_t Burst_Tick;            // [msec] System Tick when the data burst from GPS started

         uint32_t   GPS_TimeSinceLock;     // [sec] time since the GPS has a lock
          int32_t   GPS_Altitude  = 0;     // [0.1m] last valid altitude
          int32_t   GPS_Latitude  = 0;     // [1/60000deg]
          int32_t   GPS_Longitude = 0;     // [1/60000deg]
         GPS_Time   GPS_DateTime = { -1, -1, -1, -1, -1, -1, -1 } ;
          int16_t   GPS_GeoidSepar= 0;     // [0.1m]
         uint16_t   GPS_LatCosine = 3000;  // [1.0/4096]

         uint32_t   GPS_Random = 0x12345678; // random number from the LSB of the GPS data

         uint16_t   GPS_SatSNR = 0;        // [0.25dB] average SNR from the GSV sentences
         uint8_t    GPS_SatCnt = 0;

         Status     GPS_Status;            // GPS status flags

static union
{ uint8_t Flags;
  struct
  { bool     GxRMC:1;  // GPRMC or GNRMC registered
    bool     GxGGA:1;  // GPGGA or GNGGA registered
    bool     GxGSA:1;  // GPGSA or GNGSA registered
    bool     Spare:1;
    bool    Active:1;  // has started and data from the GPS is flowing
    bool  Complete:1;  // all GPS data we need is supplied and thus ready for processing
  } ;
} GPS_Burst;
                                                                                                   // for the autobaud on the GPS port
const int GPS_BurstTimeout = 100; // [ms]

// static const uint8_t  BaudRates=7;                                                                 // number of possible baudrates choices
// static       uint8_t  BaudRateIdx=0;                                                               // actual choice
// static const uint32_t BaudRate[BaudRates] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400 } ;  // list of baudrate the autobaud scans through

// uint32_t GPS_getBaudRate (void) { return BaudRate[BaudRateIdx]; }
// uint32_t GPS_nextBaudRate(void) { BaudRateIdx++; if(BaudRateIdx>=BaudRates) BaudRateIdx=0; return GPS_getBaudRate(); }

static uint32_t GPS_BaudRate = 4800;     // [bps] current baudrate on the GPS port
static uint32_t GPS_nextBaudRate(void)   // produce next (possible) GPS baudrate (for autobaud)
{ if(GPS_BaudRate>=230400) GPS_BaudRate=4800;
  else if(GPS_BaudRate==38400) GPS_BaudRate=57600;
  else GPS_BaudRate<<=1;
  return GPS_BaudRate; }

uint32_t GPS_getBaudRate (void) { return GPS_BaudRate; }

const uint32_t GPS_TargetBaudRate = 57600; // [bps]
// const uint8_t  GPS_TargetDynModel =      7; // for UBX GPS's: 6 = airborne with >1g, 7 = with >2g

#ifdef WITH_MAVLINK
uint16_t MAVLINK_BattVolt = 0;   // [mV]
uint16_t MAVLINK_BattCurr = 0;   // [10mA]
uint8_t  MAVLINK_BattCap  = 0;   // [%]
#endif

EventGroupHandle_t GPS_Event = 0;

// ----------------------------------------------------------------------------

FlightMonitor Flight;
static uint32_t RndID_TimeToChange = 0;

void FlightProcess(void)
{ bool PrevInFlight=Flight.inFlight();
  GPS_Position &GPS = GPS_Pos[GPS_PosIdx];
  Flight.Process(GPS_Pos[GPS_PosIdx]);
  GPS.InFlight=Flight.inFlight();
  if(Parameters.AddrType!=0) return;
  uint32_t Random = GPS_Random^RX_Random;
  if(RndID_TimeToChange==0)
  { if(Parameters.Stealth) RndID_TimeToChange = 57+Random%5; }
  else
  { if(RndID_TimeToChange==1)
    { Parameters.Address = (Random%0xFFFFFE)+1;
      Parameters.WritePOGNS(Line);
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, Line);
      // Format_String(CONS_UART_Write, "$POGNS,Address=0x");
      // Format_Hex(CONS_UART_Write, (uint8_t)(Parameters.Address>>16));
      // Format_Hex(CONS_UART_Write, (uint16_t)(Parameters.Address));
      // Format_String(CONS_UART_Write, ",AddrType=");
      // CONS_UART_Write('0'+Parameters.AddrType);
      // Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex); }
    RndID_TimeToChange--; }
  if(PrevInFlight==1 && GPS.InFlight==0) RndID_TimeToChange+=20;
}

// ----------------------------------------------------------------------------

static char GPS_Cmd[64];         // command to be send to the GPS

// Satellite count and SNR per system, 0=GPS, 1=GLONASS, 2=GALILEO, 3=BEIDO
static uint16_t SatSNRsum[4]   = { 0, 0, 0, 0 }; // sum up the satellite SNR's
static uint8_t  SatSNRcount[4] = { 0, 0, 0, 0 }; // sum counter

struct GPS_Sat          // store GPS satellite data in single 32-bit word
{ union
  { uint32_t Word;
    struct
    { uint16_t Azim: 9; // [deg]
      uint8_t  Elev: 7; // [deg]
      uint8_t   SNR: 7; // [dB/Hz]
      uint16_t  PRN: 9; // [1..96] GPS:1..32, SBAS:33..64, GNSS:65..96
    } ;
  } ;
} ;

static void ProcessGSV(NMEA_RxMsg &GSV)              // process GxGSV to extract satellite data
{
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, (const char *)GSV.Data, 0, GSV.Len);
  Format_String(CONS_UART_Write, " (");
  Format_UnsDec(CONS_UART_Write, (uint16_t)GSV.Parms);
  Format_String(CONS_UART_Write, ")\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  uint8_t SatSys=0;
       if(GSV.isGPGSV()) { SatSys=0; }
  else if(GSV.isGLGSV()) { SatSys=1; }
  else if(GSV.isGAGSV()) { SatSys=2; }
  else if(GSV.isBDGSV()) { SatSys=3; }
  else return;
  if(GSV.Parms<3) return;
  int8_t Pkts=Read_Dec1((const char *)GSV.ParmPtr(0)); if(Pkts<0) return;            // how many packets to pass all sats
  int8_t Pkt =Read_Dec1((const char *)GSV.ParmPtr(1)); if(Pkt <0) return;            // which packet in the sequence
  int8_t Sats=Read_Dec2((const char *)GSV.ParmPtr(2));                               // total number of satellites
  if(Sats<0) Sats=Read_Dec1((const char *)GSV.ParmPtr(2));                           // could be a single or double digit number
  if(Sats<0) return;
  if(Pkt==1) { SatSNRsum[SatSys]=0; SatSNRcount[SatSys]=0; }                         // if 1st packet then clear the sum and counter
  for( int Parm=3; Parm<GSV.Parms; )                                                 // up to 4 sats per packet
  { int8_t PRN =Read_Dec2((const char *)GSV.ParmPtr(Parm++)); if(PRN <0) break;      // PRN number
    int8_t Elev=Read_Dec2((const char *)GSV.ParmPtr(Parm++)); if(Elev<0) break;      // [deg] eleveation
   int16_t Azim=Read_Dec3((const char *)GSV.ParmPtr(Parm++)); if(Azim<0) break;      // [deg] azimuth
    int8_t SNR =Read_Dec2((const char *)GSV.ParmPtr(Parm++)); if(SNR<=0) continue;   // [dB] SNR or absent when not tracked
    SatSNRsum[SatSys]+=SNR; SatSNRcount[SatSys]++; }                                 // add up SNR
  if(Pkt==Pkts)                                                                      // if the last packet
  {
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "SatSNR[");
    CONS_UART_Write('0'+SatSys);
    Format_String(CONS_UART_Write, "] ");
    Format_UnsDec(CONS_UART_Write, SatSNRsum[SatSys]);
    CONS_UART_Write('/');
    Format_UnsDec(CONS_UART_Write, (uint16_t)SatSNRcount[SatSys]);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    uint8_t Count=0; uint16_t Sum=0;
    for(uint8_t Sys=0; Sys<4; Sys++)
    { Count+=SatSNRcount[Sys]; Sum+=SatSNRsum[Sys]; }
    GPS_SatCnt = Count;
    if(Count) GPS_SatSNR = (4*Sum+Count/2)/Count;
        else  GPS_SatSNR = 0;
  }
}

// ----------------------------------------------------------------------------

int16_t GPS_AverageSpeed(void)                        // get average speed based on stored GPS positions
{ uint8_t Count=0;
  int16_t Speed=0;
  for(uint8_t Idx=0; Idx<GPS_PosPipeSize; Idx++)      // loop over GPS positions
  { GPS_Position *Pos = GPS_Pos+Idx;
    if( !Pos->hasGPS || !Pos->isValid() ) continue;   // skip invalid positions
    Speed += Pos->Speed +abs(Pos->ClimbRate); Count++;
  }
  if(Count==0) return -1;
  if(Count>1) Speed/=Count;
  return Speed; }                                     // [0.1m/s]

// ----------------------------------------------------------------------------

static void GPS_PPS_On(void)                          // called on rising edge of PPS
{ static TickType_t PrevTickCount=0;
  PPS_Tick = xTaskGetTickCount();                     // [ms] TickCount now
  TickType_t Delta = PPS_Tick-PrevTickCount;          // [ms] time difference to the previous PPS
  PrevTickCount = PPS_Tick;                           // [ms]
  if(abs((int)Delta-1000)>=20) return;                // [ms] filter out difference away from 1.00sec
  TimeSync_HardPPS(PPS_Tick);                         // [ms] synchronize the UTC time to the PPS at given Tick
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60, 2);
  CONS_UART_Write('.');
  Format_UnsDec(CONS_UART_Write, TimeSync_msTime(), 3);
  Format_String(CONS_UART_Write, " -> PPS\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  GPS_Status.PPS=1;
  LED_PCB_Flash(100);
  // uint8_t Sec=GPS_Sec; Sec++; if(Sec>=60) Sec=0; GPS_Sec=Sec;
  // GPS_UnixTime++;
// #ifdef WITH_MAVLINK
//   static MAV_SYSTEM_TIME MAV_Time;
//   MAV_Time.time_unix_usec = (uint64_t)1000000*TimeSync_Time();
//   MAV_Time.time_boot_ms   = TickCount;
//   xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
//   MAV_RxMsg::Send(sizeof(MAV_Time), MAV_Seq++, MAV_SysID, MAV_COMP_ID_GPS, MAV_ID_SYSTEM_TIME, (const uint8_t *)&MAV_Time, CONS_UART_Write);
//   xSemaphoreGive(CONS_Mutex);
// #endif
}

static void GPS_PPS_Off(void)                       // called on falling edge of PPS
{ }

// ----------------------------------------------------------------------------

static void GPS_LockStart(void)                     // called when GPS catches a lock
{

#ifdef WITH_BEEPER
  if(KNOB_Tick>12)
  { Play(Play_Vol_1 | Play_Oct_1 | 0x00, 100);
    Play(Play_Vol_0 | Play_Oct_1 | 0x00, 100);
    Play(Play_Vol_1 | Play_Oct_1 | 0x02, 100);
    Play(Play_Vol_0 | Play_Oct_1 | 0x02, 100); }
#endif

}

static void GPS_LockEnd(void)                       // called when GPS looses a lock
{

#ifdef WITH_BEEPER
  if(KNOB_Tick>12)
  { Play(Play_Vol_1 | Play_Oct_1 | 0x02, 100);
    Play(Play_Vol_0 | Play_Oct_1 | 0x02, 100);
    Play(Play_Vol_1 | Play_Oct_1 | 0x00, 100);
    Play(Play_Vol_0 | Play_Oct_1 | 0x00, 100); }
#endif

}

// ----------------------------------------------------------------------------

static void GPS_BurstStart(int CharDelay=0)  // when GPS starts sending the data on the serial port
{ GPS_Burst.Active=1;
  Burst_Tick=xTaskGetTickCount();
  if(CharDelay) Burst_Tick -= (CharDelay*10000)/GPS_BaudRate;           // correct for the data already received on the GPS port
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_UnsDec(CONS_UART_Write, TimeSync_Time(Burst_Tick)%60, 2);
  CONS_UART_Write('.');
  Format_UnsDec(CONS_UART_Write, TimeSync_msTime(Burst_Tick), 3);
  Format_String(CONS_UART_Write, " -> GPS_BurstStart   () GPS:");
  Format_Hex(CONS_UART_Write, GPS_Status.Flags);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
#ifdef WITH_GPS_CONFIG
  static uint16_t QueryWait=0;
  if(GPS_Status.NMEA || GPS_Status.UBX)                                  // if there is communication with the GPS already
  { if(QueryWait)
    { QueryWait--; }
    else
    { if(!GPS_Status.ModeConfig)                                             // if GPS navigation mode is not done yet
      { // Format_String(CONS_UART_Write, "CFG_NAV5 query...\n");
#ifdef WITH_GPS_UBX
        if(Parameters.NavRate)
        { UBX_CFG_RATE CFG_RATE;
          CFG_RATE.measRate = 1000/Parameters.NavRate;
          CFG_RATE.navRate = 1;
          CFG_RATE.timeRef = 0;
          UBX_RxMsg::Send(0x06, 0x08, GPS_UART_Write, (uint8_t*)(&CFG_RATE), sizeof(CFG_RATE));
#ifdef DEBUG_PRINT
          Format_String(CONS_UART_Write, "GPS <- CFG-RATE: ");
          UBX_RxMsg::Send(0x06, 0x08, CONS_HexDump, (uint8_t*)(&CFG_RATE), sizeof(CFG_RATE));
          Format_String(CONS_UART_Write, "\n");
#endif
        }
        { UBX_CFG_NAV5 CFG_NAV5;
          CFG_NAV5.setDynModel(Parameters.NavMode);
          UBX_RxMsg::Send(0x06, 0x24, GPS_UART_Write, (uint8_t*)(&CFG_NAV5), sizeof(CFG_NAV5));
#ifdef DEBUG_PRINT
          Format_String(CONS_UART_Write, "GPS <- CFG-NAV5: ");
          UBX_RxMsg::Send(0x06, 0x24, CONS_HexDump, (uint8_t*)(&CFG_NAV5), sizeof(CFG_NAV5));
          Format_String(CONS_UART_Write, "\n");
#endif
        }
        UBX_RxMsg::Send(0x06, 0x08, GPS_UART_Write);                     // send the query for the navigation rate
        UBX_RxMsg::Send(0x06, 0x24, GPS_UART_Write);                     // send the query for the navigation mode setting
        UBX_RxMsg::Send(0x06, 0x3E, GPS_UART_Write);                     // send the query for the GNSS configuration
        UBX_RxMsg::Send(0x06, 0x16, GPS_UART_Write);                     // send the query for the SBAS configuration
        // if(!GPS_Status.NMEA)                                             // if NMEA sentences are not there
        { UBX_CFG_MSG CFG_MSG;                                           // send CFG_MSG to enable the NMEA sentences
          CFG_MSG.msgClass = 0xF0;                                       // NMEA class
          CFG_MSG.rate     =    1;                                       // send every measurement event
          CFG_MSG.msgID    = 0x00;                                        // ID for GGA
          UBX_RxMsg::Send(0x06, 0x01, GPS_UART_Write, (uint8_t *)(&CFG_MSG), sizeof(CFG_MSG));
          CFG_MSG.msgID    = 0x02;                                        // ID for RMC
          UBX_RxMsg::Send(0x06, 0x01, GPS_UART_Write, (uint8_t *)(&CFG_MSG), sizeof(CFG_MSG));
          CFG_MSG.msgID    = 0x04;                                        // ID for GSA
          UBX_RxMsg::Send(0x06, 0x01, GPS_UART_Write, (uint8_t *)(&CFG_MSG), sizeof(CFG_MSG));
          CFG_MSG.rate     = Parameters.NavRate*4;                        // send only at some interval
          if(CFG_MSG.rate<4) CFG_MSG.rate=4;
          CFG_MSG.msgID    = 0x03;                                        // ID for GSV
          UBX_RxMsg::Send(0x06, 0x01, GPS_UART_Write, (uint8_t *)(&CFG_MSG), sizeof(CFG_MSG));
        }
#endif
#ifdef WITH_GPS_MTK
        Format_String(GPS_UART_Write, "\r\n\r\n");                     // apparently this is needed, otherwise the next command is missed
        if(Parameters.NavRate)
        { // uint8_t Len = Format_String(GPS_Cmd, "$PMTK220,");                   // report rate
          uint8_t Len = Format_String(GPS_Cmd, "$PMTK300,");                   // fix rate
          uint16_t OneSec = 1000;
          Len += Format_UnsDec(GPS_Cmd+Len, OneSec/Parameters.NavRate);
          Len += Format_String(GPS_Cmd+Len, ",0,0,0,0");
          Len += NMEA_AppendCheck(GPS_Cmd, Len);
          GPS_Cmd[Len++]='\r';
          GPS_Cmd[Len++]='\n';
          GPS_Cmd[Len]=0;
          // Format_String(CONS_UART_Write, GPS_Cmd, Len, 0); // for debug
          Format_String(GPS_UART_Write, GPS_Cmd, Len, 0);
          GPS_Status.ModeConfig=1; }
        if(Parameters.NavMode)
        { uint8_t Len = Format_String(GPS_Cmd, "$PMTK886,");                                        // MTK command to change the navigation mode
          GPS_Cmd[Len++]='0'+Parameters.NavMode;
          Len += NMEA_AppendCheck(GPS_Cmd, Len);
          GPS_Cmd[Len++]='\r';
          GPS_Cmd[Len++]='\n';
          GPS_Cmd[Len]=0;
          // Format_String(CONS_UART_Write, GPS_Cmd, Len, 0);  // for debug
          Format_String(GPS_UART_Write, GPS_Cmd, Len, 0);
          GPS_Status.ModeConfig=1; }
        if(Parameters.GNSS)
        { uint8_t Len = Format_String(GPS_Cmd, "$PMTK353,"); // GNSS configuration
          GPS_Cmd[Len++]='0'+Parameters.EnableGPS;   // search (or not) for GPS satellites
          GPS_Cmd[Len++]=',';
          GPS_Cmd[Len++]='0'+Parameters.EnableGLO;   // search (or not) for GLONASS satellites (L86 supports)
          GPS_Cmd[Len++]=',';
          GPS_Cmd[Len++]='0'+Parameters.EnableGAL;   // search (or not) for GALILEO satellites (not clear if already supported)
          GPS_Cmd[Len++]=',';
          GPS_Cmd[Len++]='0';                        // GALILEO full mode (whatever it is ?)  (not supported yet)
          GPS_Cmd[Len++]=',';
          GPS_Cmd[Len++]='0'+Parameters.EnableBEI;   // search (or not) for BAIDOU satellites (not supported yet ?)
          Len += NMEA_AppendCheck(GPS_Cmd, Len);
          GPS_Cmd[Len++]='\r';
          GPS_Cmd[Len++]='\n';
          GPS_Cmd[Len]=0;
          // Format_String(CONS_UART_Write, GPS_Cmd, Len, 0); // for debug
          Format_String(GPS_UART_Write, GPS_Cmd, Len, 0); }
#endif
      }
      if(!GPS_Status.BaudConfig)                                             // if GPS baud config is not done yet
      { // Format_String(CONS_UART_Write, "CFG_PRT query...\n");
#ifdef WITH_GPS_UBX
        UBX_CFG_PRT CFG_PRT;                       // send in blind the config message for the UART
        CFG_PRT.portID=1;
        CFG_PRT.reserved1=0x00;
        CFG_PRT.txReady=0x0000;
        CFG_PRT.mode=0x08D0;                       // some sources say 0x08C0 if baud=9600
        CFG_PRT.baudRate=GPS_TargetBaudRate;
        CFG_PRT.inProtoMask=3;
        CFG_PRT.outProtoMask=3;
        CFG_PRT.flags=0x0000;
        CFG_PRT.reserved2=0x0000;
        UBX_RxMsg::Send(0x06, 0x00, GPS_UART_Write, (uint8_t*)(&CFG_PRT), sizeof(CFG_PRT));
#ifdef DEBUG_PRINT
        Format_String(CONS_UART_Write, "GPS <- CFG_PRT: ");
        UBX_RxMsg::Send(0x06, 0x00, CONS_HexDump, (uint8_t*)(&CFG_PRT), sizeof(CFG_PRT));
        Format_String(CONS_UART_Write, "\n");
#endif
        UBX_RxMsg::Send(0x06, 0x00, GPS_UART_Write);                     // send the query for the port config to have a template configuration packet
#ifdef DEBUG_PRINT
        Format_String(CONS_UART_Write, "GPS <- CFG-PRT: ");
        UBX_RxMsg::Send(0x06, 0x00, CONS_HexDump);                       // send the query for the port config to have a template configuration packet
        Format_String(CONS_UART_Write, "\n");
#endif
#endif // WITH_GPS_UBX
#ifdef WITH_GPS_MTK
        { strcpy(GPS_Cmd, "$PMTK251,");                                        // MTK command to change the baud rate
          uint8_t Len = strlen(GPS_Cmd);
          Len += Format_UnsDec(GPS_Cmd+Len, GPS_TargetBaudRate);
          Len += NMEA_AppendCheck(GPS_Cmd, Len);
          GPS_Cmd[Len++]='\r';                                                 // this is apparently needed but it should not, as ESP32 does auto-CR ??
          GPS_Cmd[Len++]='\n';
          GPS_Cmd[Len]=0;
          Format_String(GPS_UART_Write, GPS_Cmd, Len, 0); }
#ifdef DEBUG_PRINT
        uint8_t Len = strlen(GPS_Cmd);
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "GPS <- ");
        Format_String(CONS_UART_Write, GPS_Cmd, Len, 0);
        xSemaphoreGive(CONS_Mutex);
#endif
#endif // WITH_GPS_MTK
#ifdef WITH_GPS_SRF
        strcpy(GPS_Cmd, "$PSRF100,1,");                                        // SiRF command to change the baud rate
        Len = strlen(GPS_Cmd);
        Len += Format_UnsDec(GPS_Cmd+Len, GPS_TargetBaudRate);
        strcpy(GPS_Cmd+Len, ",8,1,0");
        Len = strlen(GPS_Cmd);
        Len += NMEA_AppendCheck(GPS_Cmd, Len);
        GPS_Cmd[Len++]='\r';                                                 // this is apparently needed but it should not, as ESP32 does auto-CR ??
        GPS_Cmd[Len++]='\n';
        GPS_Cmd[Len]=0;
        Format_String(GPS_UART_Write, GPS_Cmd, Len, 0);
#endif // WITH_GPS_SRF
        // GPS_UART_Flush(500);                                                 // wait for all data to be sent to the GPS
        // GPS_UART_SetBaudrate(GPS_TargetBaudRate); GPS_BaudRate=GPS_TargetBaudRate;   // switch serial port to the new baudrate
      }
      QueryWait=60;
    }
  }
  else { QueryWait=0; }
#endif // WITH_GPS_CONFIG
}

static void GPS_Random_Update(uint8_t Bit)
{ GPS_Random = (GPS_Random<<1) | (Bit&1); }

static void GPS_Random_Update(GPS_Position *Pos)
{ if(Pos==0) return;
  GPS_Random_Update(Pos->Altitude);
  GPS_Random_Update(Pos->Speed);
  GPS_Random_Update(Pos->Latitude);
  GPS_Random_Update(Pos->Longitude);
  if(Pos->hasBaro) GPS_Random_Update(Pos->Pressure);
  XorShift32(GPS_Random); }

static void GPS_BurstComplete(void)                                        // when GPS has sent the essential data for position fix
{ GPS_Burst.Complete=1;
#ifdef WITH_MAVLINK
  GPS_Position *GPS = GPS_Pos+GPS_PosIdx;
  if(GPS->hasTime && GPS->hasGPS && GPS->hasBaro)
  { int32_t StdAlt1 = Atmosphere::StdAltitude((GPS->Pressure+2)/4);   // [0.1m] we try to fix the cheap chinese ArduPilot with baro chip cs5607 instead of cs5611
    int32_t StdAlt2 = Atmosphere::StdAltitude((GPS->Pressure+1)/2);   // [0.1m] the cx5607 is very close but gives pressure is twice as larger units
    int32_t Alt    = GPS->Altitude;                                   // [0.1m] thus it appears to give pressure readout lower by a factor of two.
    int32_t Delta1 = StdAlt1-Alt;                                     // [0.1m] Here we check which pressure fits better the GPS altitude
    int32_t Delta2 = StdAlt2-Alt;                                     // [0.1m]
    if( abs(Delta1)< abs(Delta2)) { GPS->StdAltitude=StdAlt1; }       //
           else { GPS->Pressure*=2; GPS->StdAltitude=StdAlt2; }
  }
#endif
#ifdef DEBUG_PRINT
  GPS_Pos[GPS_PosIdx].PrintLine(Line);                                   // print out the GPS position in a single-line format
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60, 2);
  CONS_UART_Write('.');
  Format_UnsDec(CONS_UART_Write, TimeSync_msTime(), 3);
  Format_String(CONS_UART_Write, " -> GPS_BurstComplete() GPS:");
  Format_Hex(CONS_UART_Write, GPS_Status.Flags);
  Format_String(CONS_UART_Write, "\nGPS[");
  CONS_UART_Write('0'+GPS_PosIdx); CONS_UART_Write(']'); CONS_UART_Write(' ');
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex);
#endif
  GPS_Random_Update(GPS_Pos+GPS_PosIdx);
  if(GPS_Pos[GPS_PosIdx].hasGPS)                                              // GPS position data complete
  { GPS_Pos[GPS_PosIdx].isReady=1;                                            // mark this record as ready for processing => producing packets for transmission
    if(GPS_Pos[GPS_PosIdx].isTimeValid())                                     // if time is valid already
    { if(GPS_Pos[GPS_PosIdx].isDateValid())                                   // if date is valid as well
      { uint32_t UnixTime=GPS_Pos[GPS_PosIdx].getUnixTime();
        // GPS_FatTime=GPS_Pos[GPS_PosIdx].getFatTime();
#ifndef WITH_MAVLINK                                                       // with MAVlink we sync. with the SYSTEM_TIME message
        int32_t msDiff = GPS_Pos[GPS_PosIdx].mSec;
        if(msDiff>=500) { msDiff-=1000; UnixTime++; }                      // [ms]
        TimeSync_SoftPPS(Burst_Tick, UnixTime, msDiff+Parameters.PPSdelay);
        // if(abs(msDiff)<=200)                                               // if (almost) full-second burst
        // { // TickType_t PPS_Age = Burst_Tick-PPS_Tick;
          // if(PPS_Age>10000) TimeSync_SoftPPS(Burst_Tick, UnixTime, Parameters.PPSdelay);
          //              else TimeSync_SetTime(Burst_Tick-Parameters.PPSdelay, UnixTime);
          // TimeSync_SoftPPS(Burst_Tick, UnixTime, msDiff+Parameters.PPSdelay);
        // }
#endif
      }
    }
    if(GPS_Pos[GPS_PosIdx].isValid())                                         // position is complete and locked
    { if(Parameters.manGeoidSepar)                                            // if GeoidSepar is "manual" - this implies the GPS does not correct for it
      { GPS_Pos[GPS_PosIdx].GeoidSeparation = Parameters.GeoidSepar;          // copy the manually set GeoidSepar
        GPS_Pos[GPS_PosIdx].Altitude -= Parameters.GeoidSepar; }              // correct the Altitude - we likely need a separate flag for this
      GPS_Pos[GPS_PosIdx].calcLatitudeCosine();
      GPS_TimeSinceLock++;
      GPS_Altitude=GPS_Pos[GPS_PosIdx].Altitude;
      GPS_Latitude=GPS_Pos[GPS_PosIdx].Latitude;
      GPS_Longitude=GPS_Pos[GPS_PosIdx].Longitude;
      GPS_GeoidSepar=GPS_Pos[GPS_PosIdx].GeoidSeparation;
      GPS_LatCosine=GPS_Pos[GPS_PosIdx].LatitudeCosine;
      // GPS_FreqPlan=GPS_Pos[GPS_PosIdx].getFreqPlan();
      if(GPS_TimeSinceLock==1)                                                 // if we just acquired the lock a moment ago
      { GPS_LockStart(); }
      if(GPS_TimeSinceLock>1)                                                  // if the lock is more persistant
      { uint8_t PrevIdx=(GPS_PosIdx+PosPipeIdxMask)&PosPipeIdxMask;            // previous GPS data
        int16_t TimeDiff = GPS_Pos[GPS_PosIdx].calcTimeDiff(GPS_Pos[PrevIdx]); // difference in time
        for( ; ; )                                                             // loop
        { if(TimeDiff>=950) break;                                             // if at least 0.950sec then enough to calc. the differentials
          uint8_t PrevIdx2=(PrevIdx+PosPipeIdxMask)&PosPipeIdxMask;            // go back one GPS position
          if(PrevIdx2==GPS_PosIdx) break;                                      // if we looped all the way back: stop
          if(!GPS_Pos[PrevIdx2].isValid()) break;                              // if GPS position not valid: stop
          TimeDiff = GPS_Pos[GPS_PosIdx].calcTimeDiff(GPS_Pos[PrevIdx2]);      // time difference between the positions
          PrevIdx=PrevIdx2; }                                                  //
        TimeDiff=GPS_Pos[GPS_PosIdx].calcDifferentials(GPS_Pos[PrevIdx]);
#ifdef DEBUG_PRINT
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "calcDiff() => ");
        Format_UnsDec(CONS_UART_Write, (uint16_t)GPS_PosIdx);
        Format_String(CONS_UART_Write, "->");
        Format_UnsDec(CONS_UART_Write, (uint16_t)PrevIdx);
        CONS_UART_Write(' ');
        Format_SignDec(CONS_UART_Write, TimeDiff, 4, 3);
        Format_String(CONS_UART_Write, "s\n");
        xSemaphoreGive(CONS_Mutex);
#endif
        LED_PCB_Flash(200); }
    }
    else                                                                  // complete but no valid lock
    { if(GPS_TimeSinceLock) { GPS_LockEnd(); GPS_TimeSinceLock=0; }
    }
// #ifdef WITH_MAVLINK
//     static MAV_GPS_RAW_INT MAV_Position;
//     GPS_Pos[GPS_PosIdx].Encode(MAV_Position);
//     xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
//     MAV_RxMsg::Send(sizeof(MAV_Position), MAV_Seq++, MAV_SysID, MAV_COMP_ID_GPS, MAV_ID_GPS_RAW_INT, (const uint8_t *)&MAV_Position, CONS_UART_Write);
//     xSemaphoreGive(CONS_Mutex);
// #endif
  }
  else                                                                    // posiiton not complete, no GPS lock
  { if(GPS_TimeSinceLock) { GPS_LockEnd(); GPS_TimeSinceLock=0; }
  }
  uint8_t NextPosIdx = (GPS_PosIdx+1)&PosPipeIdxMask;                         // next position to be recorded
  if( GPS_Pos[GPS_PosIdx].isTimeValid() && GPS_Pos[NextPosIdx].isTimeValid() )
  { int32_t Period = GPS_Pos[GPS_PosIdx].calcTimeDiff(GPS_Pos[NextPosIdx]);   // [msec]
    if(Period>0) GPS_PosPeriod = (Period+GPS_PosPipeSize/2)/(GPS_PosPipeSize-1);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write,"GPS[");
    CONS_UART_Write('0'+GPS_PosIdx); CONS_UART_Write(']'); CONS_UART_Write(' ');
    Format_UnsDec(CONS_UART_Write, (uint16_t)GPS_Pos[GPS_PosIdx].Sec, 2);
    CONS_UART_Write('.');
    Format_UnsDec(CONS_UART_Write, (uint16_t)GPS_Pos[GPS_PosIdx].mSec, 3);
    Format_String(CONS_UART_Write, "s ");
    Format_SignDec(CONS_UART_Write, Period, 4, 3);
    Format_String(CONS_UART_Write, "s\n");
    xSemaphoreGive(CONS_Mutex);
#endif
  }
  GPS_Pos[NextPosIdx].Clear();                                              // clear the next position
  GPS_Pos[NextPosIdx].copyTime(GPS_Pos[GPS_PosIdx]);                        // copy time from current position
  GPS_Pos[NextPosIdx].incrTimeFrac(GPS_PosPeriod);                          // increment time by the expected period
  GPS_Pos[NextPosIdx].copyBaro(GPS_Pos[GPS_PosIdx], (int16_t)GPS_PosPeriod);
  if(GPS_Pos[GPS_PosIdx].Sec!=GPS_Pos[NextPosIdx].Sec) FlightProcess();
  // Flight.Process(GPS_Pos[GPS_PosIdx]);
  // GPS_Pos[NextPosIdx].copyDate(GPS_Pos[GPS_PosIdx]);
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "GPS -> ");
  Format_UnsDec(CONS_UART_Write, (uint16_t)GPS_Pos[NextPosIdx].Sec, 2);
  CONS_UART_Write('.');
  Format_UnsDec(CONS_UART_Write, (uint16_t)GPS_Pos[NextPosIdx].mSec, 3);
  Format_String(CONS_UART_Write, "s ");
  Format_UnsDec(CONS_UART_Write, GPS_PosPeriod, 4, 3);
  Format_String(CONS_UART_Write, "s\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  GPS_PosIdx=NextPosIdx;                                                      // advance the index
  xEventGroupSetBits(GPS_Event, GPSevt_NewPos);
}

static void GPS_BurstEnd(void)                                             // when GPS stops sending the data on the serial port
{
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_UnsDec(CONS_UART_Write, TimeSync_Time(Burst_Tick)%60, 2);
  CONS_UART_Write('.');
  Format_UnsDec(CONS_UART_Write, TimeSync_msTime(Burst_Tick), 3);
  Format_String(CONS_UART_Write, " -> GPS_BurstEnd     () GPS:");
  Format_Hex(CONS_UART_Write, GPS_Status.Flags);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  GPS_Burst.Flags=0; }                                                     // clear all flags: active and complete

// ----------------------------------------------------------------------------

GPS_Position *GPS_getPosition(uint8_t &BestIdx, int16_t &BestRes, int8_t Sec, int16_t Frac, bool Ready) // return GPS position closest to the given Sec.Frac
{ int32_t TargetTime = Frac+(int32_t)Sec*1000;                            // target time including the seconds
  BestIdx=0; BestRes=0x7FFF;
  for(uint8_t Idx=0; Idx<GPS_PosPipeSize; Idx++)                         // run through the GPS positions stored in the pipe
  { GPS_Position *Pos=GPS_Pos+Idx;
    if(Ready) if(!Pos->isReady) continue;                                // if only Ready positions requested: skip those not-ready yet
    int32_t Diff = TargetTime - (Pos->mSec + (int32_t)Pos->Sec*1000);    // difference from the target time
    if(Diff<(-30000)) Diff+=60000;                                       // wrap-around 60 sec
    else if(Diff>=30000) Diff-=60000;
    if(abs(Diff)<abs(BestRes)) { BestRes=Diff; BestIdx=Idx; }            // store the smallest difference from target
  }
  return BestRes==0x7FFF ? 0:GPS_Pos+BestIdx; }

GPS_Position *GPS_getPosition(void)                                       // return most recent GPS_Position which has time/position data
{ uint8_t PrevIdx=GPS_PosIdx;
  GPS_Position *PrevPos = GPS_Pos+PrevIdx;
  if(PrevPos->isReady) return PrevPos;
  PrevIdx=(PrevIdx+PosPipeIdxMask)&PosPipeIdxMask;
  PrevPos = GPS_Pos+PrevIdx;
  if(PrevPos->isReady) return PrevPos;
  return 0; }

GPS_Position *GPS_getPosition(int8_t Sec)                                // return the GPS_Position which corresponds to given Sec (may be incomplete and not valid)
{ for(uint8_t Idx=0; Idx<GPS_PosPipeSize; Idx++)
  { int8_t PosSec = GPS_Pos[Idx].Sec; if(GPS_Pos[Idx].mSec>=500) { PosSec++; if(PosSec>=60) PosSec-=60; }
    if(Sec==PosSec) return GPS_Pos+Idx; }
  return 0; }

// ----------------------------------------------------------------------------

static void GPS_NMEA(void)                                                 // when GPS gets a correct NMEA sentence
{ GPS_Status.NMEA=1;
  GPS_Status.BaudConfig = (GPS_getBaudRate() == GPS_TargetBaudRate);
  LED_PCB_Flash(10);                                                        // Flash the LED for 2 ms
       if(NMEA.isGxGSV()) ProcessGSV(NMEA);                                      // process satellite data
  else if(NMEA.isGxRMC())
  { int8_t SameTime = GPS_DateTime.ReadTime((const char *)NMEA.ParmPtr(0)); // 1=same time, 0=diff. time, -1=error
    if(SameTime==0 && GPS_Burst.GxGGA) { GPS_BurstComplete(); GPS_BurstEnd(); GPS_BurstStart(NMEA.Len); }
    GPS_DateTime.ReadDate((const char *)NMEA.ParmPtr(8));
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "GPS_NMEA() RMC ");
    Format_SignDec(CONS_UART_Write, (int16_t)(GPS_DateTime.Year), 2);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    GPS_Burst.GxRMC=1; }
  else if(NMEA.isGxGGA())
  { int8_t SameTime = GPS_DateTime.ReadTime((const char *)NMEA.ParmPtr(0)); // 1=same time, 0=diff. time, -1=error
    if(SameTime==0 && GPS_Burst.GxRMC) { GPS_BurstComplete(); GPS_BurstEnd(); GPS_BurstStart(NMEA.Len); }
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "GPS_NMEA() GGA ");
    Format_SignDec(CONS_UART_Write, (int16_t)(GPS_DateTime.Sec), 2);
    Format_String(CONS_UART_Write, "s\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    GPS_Burst.GxGGA=1; }
  else if(NMEA.isGxGSA())
  { GPS_Burst.GxGSA=1; }
  GPS_Pos[GPS_PosIdx].ReadNMEA(NMEA);                                          // read position elements from NMEA
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60, 2);
  CONS_UART_Write('.');
  Format_UnsDec(CONS_UART_Write, TimeSync_msTime(), 3);
  Format_String(CONS_UART_Write, " -> ");
  Format_Bytes(CONS_UART_Write, NMEA.Data, 6);
  CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, GPS_Burst.Flags);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
#ifndef WITH_GPS_NMEA_PASS
  // these NMEA from GPS we want to pass to the console
  // static uint8_t Count=0;
  // bool RatePass=0;
  // Count++; if(Count>=5) { Count=0; RatePass=1; }
  // if( NMEA.isP() || NMEA.isGxRMC() || NMEA.isGxGGA() || NMEA.isGxGSA() || NMEA.isGxGSV() || NMEA.isGPTXT()) )
  // if( NMEA.isP() || NMEA.isBD() || NMEA.isGx() )
  // we would need to patch the GGA here for the GPS which does not calc. nor correct for GeoidSepar
#endif
  { if(Parameters.Verbose)
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, (const char *)NMEA.Data, 0, NMEA.Len);
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex); }
#ifdef WITH_SDLOG
    if(Log_Free()>=128)
    { xSemaphoreTake(Log_Mutex, portMAX_DELAY);
      Format_String(Log_Write, (const char *)NMEA.Data, 0, NMEA.Len);
      Log_Write('\n');
      xSemaphoreGive(Log_Mutex); }
#endif
  }
}

#ifdef WITH_GPS_UBX
#ifdef DEBUG_PRINT
static void DumpUBX(void)
{ Format_String(CONS_UART_Write, "UBX: ");
  Format_UnsDec(CONS_UART_Write, xTaskGetTickCount(), 6, 3);
  CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, UBX.Class);
  CONS_UART_Write(':'); Format_Hex(CONS_UART_Write, UBX.ID);
  CONS_UART_Write('_'); Format_UnsDec(CONS_UART_Write, (uint16_t)UBX.Bytes);
  for(uint8_t Idx=0; Idx<UBX.Bytes; Idx++)
  { CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, UBX.Byte[Idx]); }
  Format_String(CONS_UART_Write, "\n"); }
#endif // DEBUG_PRINT

static void GPS_UBX(void)                                                         // when GPS gets an UBX packet
{ GPS_Status.UBX=1;
  GPS_Status.BaudConfig = (GPS_getBaudRate() == GPS_TargetBaudRate);
  LED_PCB_Flash(10);
#ifdef DEBUG_PRINT
  DumpUBX();
#endif
  // GPS_Pos[GPS_PosIdx].ReadUBX(UBX);
#ifdef WITH_GPS_UBX_PASS
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                                    // send ther UBX packet to the console
    UBX.Send(CONS_UART_Write);
    // DumpUBX();
    // Format_String(CONS_UART_Write, "UBX");
    // Format_Hex(CONS_UART_Write, UBX.Class);
    // Format_Hex(CONS_UART_Write, UBX.ID);
    xSemaphoreGive(CONS_Mutex); }
#endif
#ifdef WITH_GPS_CONFIG
  if(UBX.isCFG_PRT())                                                             // if port configuration
  { class UBX_CFG_PRT *CFG = (class UBX_CFG_PRT *)UBX.Word;                       // create pointer to the packet content
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "CFG-PRT: ");
    DumpUBX();
    Format_Hex(CONS_UART_Write, CFG->portID);
    CONS_UART_Write(':');
    Format_UnsDec(CONS_UART_Write, CFG->baudRate);
    Format_String(CONS_UART_Write, "bps\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    if(CFG->baudRate==GPS_TargetBaudRate) GPS_Status.BaudConfig=1;                // if baudrate same as our target then declare the baud config is done
    else                                                                          // otherwise use the received packet as the template
    { CFG->baudRate=GPS_TargetBaudRate;                                           // set the baudrate to our target
      CFG->outProtoMask|=0x02;                                                    // enable NMEA protocol
      UBX.RecalcCheck();                                                          // reclaculate the check sum
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_UnsDec(CONS_UART_Write, GPS_TargetBaudRate);
      Format_String(CONS_UART_Write, "bps\n");
      DumpUBX();
      xSemaphoreGive(CONS_Mutex);
#endif
      UBX.Send(GPS_UART_Write);                                                   // send this UBX packet to the GPS
    }
  }
  if(UBX.isCFG_NAV5())                                                            // Navigation config
  { class UBX_CFG_NAV5 *CFG = (class UBX_CFG_NAV5 *)UBX.Word;
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "CFG-NAV5: ");
    Format_Hex(CONS_UART_Write, CFG->dynModel);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    // if(CFG->dynModel==GPS_TargetDynModel) GPS_Status.ModeConfig=1;                // dynamic model = 6 => Airborne with >1g acceleration
    if(CFG->dynModel==Parameters.NavMode) GPS_Status.ModeConfig=1;                // dynamic model = 6 => Airborne with >1g acceleration
    else
    { CFG->dynModel=Parameters.NavMode; CFG->mask = 0x01;                         //
      UBX.RecalcCheck();                                                          // reclaculate the check sum
      UBX.Send(GPS_UART_Write);                                                   // send this UBX packet
    }
  }
  if(UBX.isCFG_SBAS())                                                          // if CFG-SBAS
  { class UBX_CFG_SBAS *CFG = (class UBX_CFG_SBAS *)UBX.Word;
    CFG->mode = Parameters.EnableSBAS;
    CFG->usage=3;                                                               // integrity | diff.corr. | range
    CFG->maxSBAS=3;
    CFG->scanmode1=0;
    CFG->scanmode2=0;
    UBX.RecalcCheck();                                                          // reclaculate the check sum
    UBX.Send(GPS_UART_Write);                                                   // send this UBX packet
  }
#ifdef DEBUG_PRINT
  if(UBX.isACK())
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "TaskGPS: ACK_ ");
    Format_Hex(CONS_UART_Write,  UBX.ID);
    CONS_UART_Write(' ');
    Format_Hex(CONS_UART_Write,  UBX.Byte[0]);
    CONS_UART_Write(':');
    Format_Hex(CONS_UART_Write,  UBX.Byte[1]);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
/*
    if(UBX.Byte[0]==0x06 && UBX.Byte[1]==0x00 && UBX.ID==0)  // negative ACK to CFG-PRT
    {
      strcpy(GPS_Cmd, "$PUBX,41,1,0007,0003,");              // $PUBX command to change the baud rate
      uint8_t Len = strlen(GPS_Cmd);
      Len += Format_UnsDec(GPS_Cmd+Len, GPS_TargetBaudRate);
      GPS_Cmd[Len++]=','; GPS_Cmd[Len++]='0';
      Len += NMEA_AppendCheck(GPS_Cmd, Len);
      GPS_Cmd[Len++]='\n';
      GPS_Cmd[Len]=0;
      Format_String(GPS_UART_Write, GPS_Cmd, Len, 0);
      Format_String(CONS_UART_Write, GPS_Cmd, Len, 0);
    }
*/
/*
    { UBX_CFG_PRT Cmd =
      { portID:1,                        // 0 = I2C, 1 = UART1, 2 = UART2, 3 = USB, 4 = SPI
        reserved0:0,
        txReady:0,
        mode:0x08D0,                     // 00 10x x 11 x 1 xxxx => 0x08D0
        baudRate:GPS_TargetBaudRate,     // [bps]
        inProtoMask:7,                   // bit 0:UBX, bit 1:NMEA
        outProtoMask:3,                  // bit 0:UBX, bit 1:NMEA
        reserved4:0,
        reserved5:0,
      } ;
      UBX_RxMsg::Send(0x06, 0x00, GPS_UART_Write, (uint8_t*)&Cmd, sizeof(Cmd));
    }
*/
  }
#endif
#endif // WITH_GPS_CONFIG
}
#endif // WITH_GPS_UBX

#ifdef WITH_MAVLINK
static int64_t MAV_TimeOfs_ms=0;                                           // [ms] diff. between UTC time and boot time reported in MAV messages

static uint64_t MAV_getUnixTime(void)                                      // [ms] extract time from the MAVlink message
{ int32_t TimeCorr_ms = (int32_t)Parameters.TimeCorr*1000;                 // [ms] apparently ArduPilot needs some time correction, as it "manually" converts from GPS to UTC time
  uint8_t MsgID = MAV.getMsgID();
  if(MsgID==MAV_ID_SYSTEM_TIME)         return ((const MAV_SYSTEM_TIME         *)MAV.getPayload())->time_unix_usec/1000 + TimeCorr_ms;
  if(MsgID==MAV_ID_GLOBAL_POSITION_INT) return ((const MAV_GLOBAL_POSITION_INT *)MAV.getPayload())->time_boot_ms + MAV_TimeOfs_ms;
  if(MsgID==MAV_ID_SCALED_PRESSURE)     return ((const MAV_SCALED_PRESSURE     *)MAV.getPayload())->time_boot_ms + MAV_TimeOfs_ms;
  uint64_t UnixTime_ms = 0;
  // if(MsgID==MAV_ID_RAW_IMU)     UnixTime_ms = ((const MAV_RAW_IMU     *)MAV.getPayload())->time_usec/1000;
  if(MsgID==MAV_ID_GPS_RAW_INT) UnixTime_ms = ((const MAV_GPS_RAW_INT *)MAV.getPayload())->time_usec/1000;
  if(UnixTime_ms==0) return UnixTime_ms;
  if(UnixTime_ms<1000000000000) UnixTime_ms += MAV_TimeOfs_ms;
                           else UnixTime_ms += TimeCorr_ms;
  return UnixTime_ms; }

static void GPS_MAV(void)                                                  // when GPS gets an MAV packet
{ TickType_t TickCount=xTaskGetTickCount();
  GPS_Status.MAV=1;
  LED_PCB_Flash(10);
  GPS_Status.BaudConfig = (GPS_getBaudRate() == GPS_TargetBaudRate);
  uint8_t MsgID = MAV.getMsgID();
  uint64_t UnixTime_ms = MAV_getUnixTime();                                   // get the time from the MAVlink message
  if( (MsgID!=MAV_ID_SYSTEM_TIME) && UnixTime_ms)
  { if(GPS_Pos[GPS_PosIdx].hasTime)
    { uint64_t PrevUnixTime_ms = GPS_Pos[GPS_PosIdx].getUnixTime_ms();
      int32_t TimeDiff_ms = UnixTime_ms-PrevUnixTime_ms;
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "MAV_TimeDiff: ");
    Format_UnsDec(CONS_UART_Write, (uint16_t)MsgID, 3); CONS_UART_Write(' ');
    Format_SignDec(CONS_UART_Write, TimeDiff_ms, 3);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
      if(TimeDiff_ms>GPS_BurstTimeout) GPS_BurstComplete();
    }
  }
  if(MsgID==MAV_ID_HEARTBEAT)
  { const MAV_HEARTBEAT *Heartbeat = (const MAV_HEARTBEAT *)MAV.getPayload();
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "MAV_HEARTBEAT: ");
    Format_Hex(CONS_UART_Write, Heartbeat->system_status);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
  } else if(MsgID==MAV_ID_SYSTEM_TIME)
  { const MAV_SYSTEM_TIME *SysTime = (const MAV_SYSTEM_TIME *)MAV.getPayload();
    uint32_t UnixTime = UnixTime_ms/1000;                                  // [ s] Unix Time
    uint32_t UnixFrac = UnixTime_ms-(uint64_t)UnixTime*1000;               // [ms] Second fraction of the Unix time
    MAV_TimeOfs_ms=UnixTime_ms-SysTime->time_boot_ms;                      // [ms] difference between the Unix Time and the Ardupilot time-since-boot
    TimeSync_SoftPPS(TickCount-UnixFrac, UnixTime, Parameters.PPSdelay);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "MAV_SYSTEM_TIME: ");
    Format_UnsDec(CONS_UART_Write, UnixTime, 10);
    CONS_UART_Write('.');
    Format_UnsDec(CONS_UART_Write, UnixFrac, 3);
    CONS_UART_Write(' ');
    Format_SignDec(CONS_UART_Write, MAV_TimeOfs_ms, 13, 3);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
  } else if(MsgID==MAV_ID_GLOBAL_POSITION_INT)                            // position based on GPS and inertial sensors
  { const MAV_GLOBAL_POSITION_INT *Pos = (const MAV_GLOBAL_POSITION_INT *)MAV.getPayload();
    GPS_Pos[GPS_PosIdx].Read(Pos, UnixTime_ms);                              // read position/altitude/speed/etc. into GPS_Position structure
#ifdef DEBUG_PRINT
    GPS_Pos[GPS_PosIdx].PrintLine(Line);
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "MAV_GLOBAL_POSITION_INT: ");
    Format_UnsDec(CONS_UART_Write, UnixTime_ms, 13, 3);
    Format_String(CONS_UART_Write, "\nGPS"); CONS_UART_Write('0'+GPS_PosIdx); CONS_UART_Write(':'); CONS_UART_Write(' ');
    Format_String(CONS_UART_Write, Line);
    xSemaphoreGive(CONS_Mutex);
#endif
  } else if(MsgID==MAV_ID_GPS_RAW_INT)                                    // position form the GPS
  { const MAV_GPS_RAW_INT *RawGPS = (const MAV_GPS_RAW_INT *)MAV.getPayload();
    GPS_Pos[GPS_PosIdx].Read(RawGPS, UnixTime_ms);                           // read position/altitude/speed/etc. into GPS_Position structure
#ifdef DEBUG_PRINT
    GPS_Pos[GPS_PosIdx].PrintLine(Line);
    uint32_t UnixTime = (UnixTime_ms+500)/1000;
     int32_t TimeDiff = (int64_t)UnixTime_ms-(int64_t)UnixTime*1000;
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "MAV_GPS_RAW_INT: ");
    Format_UnsDec(CONS_UART_Write, UnixTime_ms, 13, 3);
    CONS_UART_Write(' ');
    Format_SignDec(CONS_UART_Write, TimeDiff, 4, 3);
    CONS_UART_Write(abs(TimeDiff)<250 ? '*':' ');
    Format_String(CONS_UART_Write, "\nGPS"); CONS_UART_Write('0'+GPS_PosIdx); CONS_UART_Write(':'); CONS_UART_Write(' ');
    Format_String(CONS_UART_Write, Line);
    xSemaphoreGive(CONS_Mutex);
#endif
  } else if(MsgID==MAV_ID_SCALED_PRESSURE)
  { const MAV_SCALED_PRESSURE *Press = (const MAV_SCALED_PRESSURE *)MAV.getPayload();
    // uint64_t UnixTime_ms = Press->time_boot_ms + MAV_TimeOfs_ms;
    GPS_Pos[GPS_PosIdx].Read(Press, UnixTime_ms);
#ifdef DEBUG_PRINT
    GPS_Pos[GPS_PosIdx].PrintLine(Line);
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "MAV_SCALED_PRESSURE: ");
    Format_UnsDec(CONS_UART_Write, UnixTime_ms, 13, 3);
    Format_String(CONS_UART_Write, "\nGPS"); CONS_UART_Write('0'+GPS_PosIdx); CONS_UART_Write(':'); CONS_UART_Write(' ');
    Format_String(CONS_UART_Write, Line);
    xSemaphoreGive(CONS_Mutex);
#endif
  } else if(MsgID==MAV_ID_SYS_STATUS)
  { const MAV_SYS_STATUS *Status = (const MAV_SYS_STATUS *)MAV.getPayload();
    MAVLINK_BattVolt = Status->battery_voltage;   // [mV]
    MAVLINK_BattCurr = Status->battery_current;   // [10mA]
    MAVLINK_BattCap  = Status->battery_remaining; // [%]
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "MAV_SYS_STATUS: ");
    Format_UnsDec(CONS_UART_Write, Status->battery_voltage, 4, 3);
    Format_String(CONS_UART_Write, "V ");
    Format_SignDec(CONS_UART_Write, Status->battery_current, 3, 2);
    Format_String(CONS_UART_Write, "A\n");
    xSemaphoreGive(CONS_Mutex);
#endif
  // } else if(MsgID==MAV_ID_STATUSTEXT)
  // {
  }
#ifdef DEBUG_PRINT
  else
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "MAV: MsgID=");
    Format_UnsDec(CONS_UART_Write, (uint16_t)MAV.getMsgID(), 3);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
  }
#endif
}
#endif

// ----------------------------------------------------------------------------

// Baud setting for SIRF GPS:
//    9600/8/N/1      $PSRF100,1,9600,8,1,0*0D<cr><lf>
//   19200/8/N/1      $PSRF100,1,19200,8,1,0*38<cr><lf>
//   38400/8/N/1      $PSRF100,1,38400,8,1,0*3D<cr><lf>
//                    $PSRF100,1,57600,8,1,0*36
//                    $PSRF100,1,115200,8,1,0*05

// static const char *SiRF_SetBaudrate_57600  = "$PSRF100,1,57600,8,1,0*36\r\n";
// static const char *SiRF_SetBaudrate_115200 = "$PSRF100,1,115200,8,1,0*05\r\n";


// Baud setting for MTK GPS:
// $PMTK251,38400*27<CR><LF>
// $PMTK251,57600*2C<CR><LF>
// $PMTK251,115200*1F<CR><LF>

// Baud setting for UBX GPS:
// $PUBX,41,1,0007,0003,9600,0*10<CR><LF>
// $PUBX,41,1,0007,0003,38400,0*20<CR><LF>

// static const char *MTK_SetBaudrate_115200 = "$PMTK251,115200*1F\r\n";


// Baud setting for UBX GPS:
// "$PUBX,41,1,0003,0001,19200,0*23\r\n"
// "$PUBX,41,1,0003,0001,38400,0*26\r\n"
// "$PUBX,41,1,0003,0001,57600,0*2D\r\n"
// static const char *UBX_SetBaudrate_115200 = "$PUBX,41,1,0003,0001,115200,0*1E\r\n";

// ----------------------------------------------------------------------------

#ifdef __cplusplus
  extern "C"
#endif
void vTaskGPS(void* pvParameters)
{
  GPS_Event = xEventGroupCreate();
  GPS_Status.Flags = 0;

  // PPS_TickCount=0;
  Burst_Tick=0;

  vTaskDelay(5);                                                         // put some initial delay for lighter startup load

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "TaskGPS:");
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);

  GPS_Burst.Flags=0;
  bool PPS=0;
  int LineIdle=0;                                                        // [ms] counts idle time for the GPS data
  int NoValidData=0;                                                     // [ms] count time without valid data (to decide to change baudrate)
  NMEA.Clear();
#ifdef WITH_GPS_UBX
  UBX.Clear();                                             // scans GPS input for NMEA and UBX frames
#endif
#ifdef WITH_MAVLINK
  MAV.Clear();
#endif
  for(uint8_t Idx=0; Idx<4; Idx++)
    GPS_Pos[Idx].Clear();
  GPS_PosIdx=0;

  TickType_t RefTick = xTaskGetTickCount();
  for( ; ; )                                                              // main task loop: every milisecond (RTOS time tick)
  { vTaskDelay(1);                                                        // wait for the next time tick (but apparently it can wait more than one OS tick)
    TickType_t NewTick = xTaskGetTickCount();
    TickType_t Delta = NewTick-RefTick;
    RefTick = NewTick;
/*
#ifdef DEBUG_PRINT
    if(Delta>1)
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_UnsDec(CONS_UART_Write, TimeSync_Time(RefTick)%60);
      CONS_UART_Write('.');
      Format_UnsDec(CONS_UART_Write, TimeSync_msTime(RefTick),3);
      Format_String(CONS_UART_Write, " -> ");
      Format_UnsDec(CONS_UART_Write, Delta);
      Format_String(CONS_UART_Write, "t\n");
      xSemaphoreGive(CONS_Mutex); }
#endif
*/
#ifdef WITH_GPS_PPS
    if(GPS_PPS_isOn()) { if(!PPS) { PPS=1; GPS_PPS_On();  } }             // monitor GPS PPS signal
                  else { if( PPS) { PPS=0; GPS_PPS_Off(); } }             // and call handling calls
#endif
    LineIdle+=Delta;                                                      // count idle time
    NoValidData+=Delta;                                                   // count time without any valid NMEA nor UBX packet
    uint16_t Bytes=0;
    uint16_t MaxBytesPerTick = 1+(GPS_getBaudRate()+2500)/5000;
    for( ; ; )                                                            // loop over bytes in the GPS UART buffer
    { uint8_t Byte; int Err=GPS_UART_Read(Byte); if(Err<=0) break;        // get Byte from serial port, if no bytes then break this loop
      // CONS_UART_Write(Byte);                                              // copy the GPS output to console (for debug only)
      Bytes++;
      LineIdle=0;                                                         // if there was a byte: restart idle counting
      NMEA.ProcessByte(Byte);                                             // process through the NMEA interpreter
#ifdef WITH_GPS_UBX
      UBX.ProcessByte(Byte);
#endif
#ifdef WITH_MAVLINK
      MAV.ProcessByte(Byte);
#endif
      if(NMEA.isComplete())                                               // NMEA completely received ?
      { if(NMEA.isChecked()) { GPS_NMEA(); NoValidData=0; }               // NMEA check sum is correct ?
        NMEA.Clear(); break; }
#ifdef WITH_GPS_UBX
      if(UBX.isComplete()) { GPS_UBX(); NoValidData=0; UBX.Clear(); break; }
#endif
#ifdef WITH_MAVLINK
      if(MAV.isComplete()) { GPS_MAV(); NoValidData=0; MAV.Clear(); break; }
#endif
      if(Bytes>=MaxBytesPerTick) break;
    }
/*
#ifdef DEBUG_PRINT
    if(Bytes)
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_UnsDec(CONS_UART_Write, TimeSync_Time(RefTick)%60);
      CONS_UART_Write('.');
      Format_UnsDec(CONS_UART_Write, TimeSync_msTime(RefTick),3);
      Format_String(CONS_UART_Write, "..");
      Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60);
      CONS_UART_Write('.');
      Format_UnsDec(CONS_UART_Write, TimeSync_msTime(),3);
      Format_String(CONS_UART_Write, " -> ");
      Format_UnsDec(CONS_UART_Write, Bytes);
      Format_String(CONS_UART_Write, "B\n");
      xSemaphoreGive(CONS_Mutex); }
#endif
*/
    if(LineIdle==0)                                                        // if any bytes were received ?
    { if(!GPS_Burst.Active) GPS_BurstStart();                              // if not already started then declare burst started
      if( (!GPS_Burst.Complete) && (GPS_Burst.GxGGA && GPS_Burst.GxRMC && GPS_Burst.GxGSA) ) // if GGA+RMC+GSA received
      { GPS_BurstComplete(); }                                             // declare burst complete
    }
    else if(LineIdle>=GPS_BurstTimeout)                                    // if GPS sends no more data for GPS_BurstTimeout ticks
    { if(GPS_Burst.Active)                                                 // if burst was active
      { if(!GPS_Burst.Complete && GPS_Burst.GxGGA && GPS_Burst.GxRMC)
        { GPS_BurstComplete(); }                                           // if not complete yet, then declare burst complete
      }
      else if(LineIdle>=1500)                                              // if idle for more than 1.5 sec
      { GPS_Status.Flags=0; }
      if(GPS_Burst.Flags) GPS_BurstEnd();                                  // declare burst ended, if not yet done
    }

    if(NoValidData>=2000)                                                  // if no valid data from GPS for 2sec
    { GPS_Status.Flags=0; GPS_Burst.Flags=0;                               // assume GPS state is unknown
      uint32_t NewBaudRate = GPS_nextBaudRate();                           // switch to the next baud rate
      if(PowerMode>0)
      {
#ifdef WITH_GPS_UBX
#ifdef WITH_GPS_ENABLE
        GPS_ENABLE();
#endif
        GPS_UART_Write('\n');
#endif
#ifdef WITH_GPS_MTK
#ifdef WITH_GPS_ENABLE
        GPS_DISABLE();
        vTaskDelay(1);
        GPS_ENABLE();
#endif
        GPS_UART_Write('\n');
#endif
      }
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "TaskGPS: ");
      Format_UnsDec(CONS_UART_Write, NewBaudRate);
      Format_String(CONS_UART_Write, "bps\n");
      xSemaphoreGive(CONS_Mutex);
      GPS_UART_SetBaudrate(NewBaudRate);
      NoValidData=0;
    }
  }
}


