#include <stdint.h>
#include <stdlib.h>

#include "gps.h"

#include "nmea.h"
#include "ubx.h"
#ifdef WITH_MAVLINK
#include "mavlink.h"
#endif

#include "ogn.h"

// #include "ctrl.h"
// #include "knob.h"

#include "lowpass2.h"

// #define DEBUG_PRINT

// ----------------------------------------------------------------------------

// void Debug_Print(uint8_t Byte) { while(!UART1_TxEmpty()) taskYIELD(); UART1_TxChar(Byte); }

static NMEA_RxMsg  NMEA;                 // NMEA sentences catcher
#ifdef WITH_GPS_UBX
static UBX_RxMsg   UBX;                  // UBX messages catcher
#endif
#ifdef WITH_MAVLINK
static MAV_RxMsg   MAV;                  // MAVlink message catcher
#endif

static GPS_Position Position[4];         // GPS position pipe
static uint8_t      PosIdx;              // Pipe index, increments with every GPS position received

static   TickType_t Burst_TickCount;     // [msec] TickCount when the data burst from GPS started

         uint32_t   GPS_TimeSinceLock;   // [sec] time since the GPS has a lock
         uint32_t   GPS_FatTime   = 0;    // [sec] UTC date/time in FAT format
          int32_t   GPS_Altitude  = 0;    // [0.1m] last valid altitude
          int32_t   GPS_Latitude  = 0;    //
          int32_t   GPS_Longitude = 0;    //
          int16_t   GPS_GeoidSepar= 0;    // [0.1m]
         uint16_t   GPS_LatCosine = 3000; //

         Status     GPS_Status;

static union
{ uint8_t Flags;
  struct
  { bool     Spare:1;  //
    bool    Active:1;  // has started
    bool     GxRMC:1;  // GPRMC or GNRMC registered
    bool     GxGGA:1;  // GPGGA or GNGGA registered
    bool     GxGSA:1;  // GPGSA or GNGSA registered
    bool  Complete:1;  // all GPS data is supplied and thus ready for processing
  } ;
} GPS_Burst;
                                                                                                   // for the autobaud on the GPS port
const int GPS_BurstTimeout = 200; // [ms]

static const uint8_t  BaudRates=7;                                                                 // number of possible baudrates choices
static       uint8_t  BaudRateIdx=0;                                                               // actual choice
static const uint32_t BaudRate[BaudRates] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400 } ;  // list of baudrate the autobaud scans through

uint32_t GPS_getBaudRate (void) { return BaudRate[BaudRateIdx]; }
uint32_t GPS_nextBaudRate(void) { BaudRateIdx++; if(BaudRateIdx>=BaudRates) BaudRateIdx=0; return GPS_getBaudRate(); }

const uint32_t GPS_TargetBaudRate = 57600; // BaudRate[4]; // [bps] must be one of the baud rates known by the autbaud
const uint8_t  GPS_dynModel       =     7; // for UBX GPS's: 6 = airborne with >1g, 7 = with >2g

// ----------------------------------------------------------------------------

int16_t GPS_AverageSpeed(void)                        // get average speed based on stored GPS positions
{ uint8_t Count=0;
  int16_t Speed=0;
  for(uint8_t Idx=0; Idx<4; Idx++)                    // loop over GPS positions
  { GPS_Position *Pos = Position+Idx;
    if( !Pos->GPS || !Pos->isValid() ) continue;      // skip invalid positions
    Speed += Pos->Speed +abs(Pos->ClimbRate); Count++;
  }
  if(Count==0) return -1;
  if(Count>1) Speed/=Count;
  return Speed; }                                     // [0.1m/s]

// ----------------------------------------------------------------------------

static void GPS_PPS_On(void)                          // called on rising edge of PPS
{ static TickType_t PrevTickCount=0;
  TickType_t TickCount = xTaskGetTickCount();         // [ms] TickCount now
  TickType_t Delta = TickCount-PrevTickCount;         // [ms] time difference to the previous PPS
  PrevTickCount = TickCount;                          // [ms]
  if(abs((int)Delta-1000)>10) return;                 // [ms] filter out difference away from 1.00sec
  TimeSync_HardPPS(TickCount);
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60);
  CONS_UART_Write('.');
  Format_UnsDec(CONS_UART_Write, TimeSync_msTime(),3);
  Format_String(CONS_UART_Write, " -> PPS\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  GPS_Status.PPS=1;
  LED_PCB_Flash(50);
  // uint8_t Sec=GPS_Sec; Sec++; if(Sec>=60) Sec=0; GPS_Sec=Sec;
  // GPS_UnixTime++;
#ifdef WITH_MAVLINK
  static MAV_SYSTEM_TIME MAV_Time;
  MAV_Time.time_unix_usec = (uint64_t)1000000*TimeSync_Time();
  MAV_Time.time_boot_ms   = TickCount;
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  MAV_RxMsg::Send(sizeof(MAV_Time), MAV_Seq++, MAV_SysID, MAV_COMP_ID_GPS, MAV_ID_SYSTEM_TIME, (const uint8_t *)&MAV_Time, CONS_UART_Write);
  xSemaphoreGive(CONS_Mutex);
#endif
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

const uint8_t GPS_BurstDelay=70;                                           // [ms] time after the PPS when the data burst starts on the UART

static void GPS_BurstStart(void)                                           // when GPS starts sending the data on the serial port
{ Burst_TickCount=xTaskGetTickCount();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60);
  CONS_UART_Write('.');
  Format_UnsDec(CONS_UART_Write, TimeSync_msTime(),3);
  Format_String(CONS_UART_Write, " -> GPS...\n");
  xSemaphoreGive(CONS_Mutex);
#endif
#ifdef WITH_GPS_CONFIG
  static uint16_t QueryWait=0;
  if(GPS_Status.NMEA)                                                      // if there is communication with the GPS already
  { if(QueryWait)
    { QueryWait--; }
    else
    { if(!GPS_Status.ModeConfig)                                             // if GPS navigation mode is not done yet
      { // Format_String(CONS_UART_Write, "CFG_NAV5 query...\n");
#ifdef WITH_GPS_UBX
        UBX_RxMsg::SendPoll(0x06, 0x24, GPS_UART_Write);                     // send the query for the navigation mode setting
#endif
      }
      if(!GPS_Status.BaudConfig)                                             // if GPS baud config is not done yet
      { // Format_String(CONS_UART_Write, "CFG_PRT query...\n");
#ifdef WITH_GPS_UBX
        UBX_RxMsg::SendPoll(0x06, 0x00, GPS_UART_Write);                     // send the query for the port config to have a template configuration packet
#endif
#ifdef WITH_GPS_MTK
        static char GPS_Cmd[32];
        strcpy(GPS_Cmd, "$PMTK251,");                                        // MTK command to change the baud rate
        uint8_t Len = strlen(GPS_Cmd);
        Len += Format_UnsDec(GPS_Cmd+Len, GPS_TargetBaudRate);
        Len += NMEA_AppendCheck(GPS_Cmd, Len);
        GPS_Cmd[Len]=0;
        // Serial.println(GPS_Cmd);
        Format_String(GPS_UART_Write, GPS_Cmd, Len);
        GPS_UART_Write('\r'); GPS_UART_Write('\n');
#endif
#ifdef WITH_GPS_SRF
        strcpy(GPS_Cmd, "$PSRF100,1,");                                        // SiRF command to change the baud rate
        Len = strlen(GPS_Cmd);
        Len += Format_UnsDec(GPS_Cmd+Len, GPS_TargetBaudRate);
        strcpy(GPS_Cmd+Len, ",8,1,0");
        Len = strlen(GPS_Cmd);
        Len += NMEA_AppendCheck(GPS_Cmd, Len);
        GPS_Cmd[Len]=0;
        // Serial.println(GPS_Cmd);
        Format_String(GPS_UART_Write, GPS_Cmd, Len);
        GPS_UART_Write('\r'); GPS_UART_Write('\n');
#endif
      }
      QueryWait=300;
    }
  }
  else { QueryWait=0; }
#endif // WITH_GPS_CONFIG
}

static void GPS_BurstComplete(void)                                        // when GPS has sent the essential data for position fix
{
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60);
  CONS_UART_Write('.');
  Format_UnsDec(CONS_UART_Write, TimeSync_msTime(),3);
  Format_String(CONS_UART_Write, " -> ...complete\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  if(Position[PosIdx].GPS)                                                 // GPS position data complete
  { Position[PosIdx].Ready=1;
    if(Position[PosIdx].isTimeValid())                                     // if time is valid already
    { if(Position[PosIdx].isDateValid())                                   // if date is valid as well
      { uint32_t UnixTime=Position[PosIdx].getUnixTime();
        GPS_FatTime=Position[PosIdx].getFatTime();
        TimeSync_SoftPPS(Burst_TickCount, UnixTime, GPS_BurstDelay);                //
      }
    }
    if(Position[PosIdx].isValid())                                         // position is complete and locked
    { Position[PosIdx].calcLatitudeCosine();
      GPS_TimeSinceLock++;
      GPS_Altitude=Position[PosIdx].Altitude;
      GPS_Latitude=Position[PosIdx].Latitude;
      GPS_Longitude=Position[PosIdx].Longitude;
      GPS_GeoidSepar=Position[PosIdx].GeoidSeparation;
      GPS_LatCosine=Position[PosIdx].LatitudeCosine;
      // GPS_FreqPlan=Position[PosIdx].getFreqPlan();
      if(GPS_TimeSinceLock==1)
      { GPS_LockStart(); }
      if(GPS_TimeSinceLock>1)
      { uint8_t PrevIdx=(PosIdx+3)&3;
        Position[PosIdx].calcDifferences(Position[PrevIdx]);
        LED_PCB_Flash(100); }
    }
    else                                                                  // complete but no valid lock
    { if(GPS_TimeSinceLock) { GPS_LockEnd(); GPS_TimeSinceLock=0; }
    }
#ifdef WITH_MAVLINK
    static MAV_GPS_RAW_INT MAV_Position;
    Position[PosIdx].Encode(MAV_Position);
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    MAV_RxMsg::Send(sizeof(MAV_Position), MAV_Seq++, MAV_SysID, MAV_COMP_ID_GPS, MAV_ID_GPS_RAW_INT, (const uint8_t *)&MAV_Position, CONS_UART_Write);
    xSemaphoreGive(CONS_Mutex);
#endif
  }
  else                                                                    // posiiton not complete, no GPS lock
  { if(GPS_TimeSinceLock) { GPS_LockEnd(); GPS_TimeSinceLock=0; }
  }
  uint8_t NextPosIdx = (PosIdx+1)&3;                                      // next position to be recorded
  Position[NextPosIdx].Clear();                                           // clear the next position
  int8_t Sec = Position[PosIdx].Sec;                                      //
  Sec++; if(Sec>=60) Sec=0;
  Position[NextPosIdx].Sec=Sec;                                           // set the correct time for the next position
  // Position[NextPosIdx].copyTime(Position[PosIdx]);                        // copy time from current position
  // Position[NextPosIdx].incrTime();                                        // increment time by 1 sec
  PosIdx=NextPosIdx;                                                      // advance the index
}

static void GPS_BurstEnd(void)                                             // when GPS stops sending the data on the serial port
{ }

GPS_Position *GPS_getPosition(void)                                       // return most recent GPS_Position which is complete
{ uint8_t PrevIdx=PosIdx;
  GPS_Position *PrevPos = Position+PrevIdx;
  if(PrevPos->GPS) return PrevPos;
  PrevIdx=(PrevIdx+3)&3;
  PrevPos = Position+PrevIdx;
  if(PrevPos->GPS) return PrevPos;
  return 0; }

GPS_Position *GPS_getPosition(int8_t Sec)                                // return the GPS_Position which corresponds to given Sec (may be incomplete and not valid)
{ for(uint8_t Idx=0; Idx<4; Idx++)
  { if(Sec==Position[Idx].Sec) return Position+Idx; }
  return 0; }

// ----------------------------------------------------------------------------

static void GPS_NMEA(void)                                                 // when GPS gets a correct NMEA sentence
{ GPS_Status.NMEA=1;
  GPS_Status.BaudConfig = (GPS_getBaudRate() == GPS_TargetBaudRate);
  LED_PCB_Flash(2);                                                        // Flash the LED for 2 ms
  Position[PosIdx].ReadNMEA(NMEA);                                         // read position elements from NMEA
  if(NMEA.isGxRMC()) GPS_Burst.GxRMC=1;
  if(NMEA.isGxGGA()) GPS_Burst.GxGGA=1;
  if(NMEA.isGxGSA()) GPS_Burst.GxGSA=1;
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60);
  CONS_UART_Write('.');
  Format_UnsDec(CONS_UART_Write, TimeSync_msTime(),3);
  Format_String(CONS_UART_Write, " -> ");
  Format_Bytes(CONS_UART_Write, NMEA.Data, 6);
  CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, GPS_Burst.Flags);
  CONS_UART_Write('\r'); CONS_UART_Write('\n');
  xSemaphoreGive(CONS_Mutex);
#endif
  if( NMEA.isP() || NMEA.isGxRMC() || NMEA.isGxGGA() || NMEA.isGxGSA() || NMEA.isGPTXT() )
  { static char CRNL[3] = "\r\n";
    // if(CONS_UART_Free()>=128)
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_Bytes(CONS_UART_Write, NMEA.Data, NMEA.Len);
      Format_Bytes(CONS_UART_Write, (const uint8_t *)CRNL, 2);
      xSemaphoreGive(CONS_Mutex); }
#ifdef WITH_SDLOG
    if(Log_Free()>=128)
    { xSemaphoreTake(Log_Mutex, portMAX_DELAY);
      Format_Bytes(Log_Write, NMEA.Data, NMEA.Len);
      Format_Bytes(Log_Write, (const uint8_t *)CRNL, 2);
      xSemaphoreGive(Log_Mutex); }
#endif
  }
}

static void DumpUBX(void)
{ Format_String(CONS_UART_Write, "UBX:");
  for(uint8_t Idx=0; Idx<20; Idx++)
  { CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, UBX.Byte[Idx]); }
  Format_String(CONS_UART_Write, "\n"); }

#ifdef WITH_GPS_UBX
static void GPS_UBX(void)                                                         // when GPS gets an UBX packet
{ GPS_Status.UBX=1;
  GPS_Status.BaudConfig = (GPS_getBaudRate() == GPS_TargetBaudRate);
  LED_PCB_Flash(2);
#ifdef WITH_GPS_UBX_PASS
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                                    // send ther UBX packet to the console
    UBX.Send(CONS_UART_Write);
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
    Format_String(CONS_UART_Write, "TaskGPS: CFG_PRT\n");
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
    Format_String(CONS_UART_Write, "TaskGPS: CFG_NAV5 ");
    Format_Hex(CONS_UART_Write, CFG->dynModel);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    if(CFG->dynModel==GPS_dynModel) GPS_Status.ModeConfig=1;                      // dynamic model = 6 => Airborne with >1g acceleration
    else
    { CFG->dynModel=GPS_dynModel; CFG->mask = 0x01;                               //
      UBX.RecalcCheck();                                                          // reclaculate the check sum
      UBX.Send(GPS_UART_Write);                                                   // send this UBX packet
    }
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
  }
#endif
#endif // WITH_GPS_CONFIG
}
#endif // WITH_GPS_UBX

#ifdef WITH_MAVLINK
static void GPS_MAV(void)                                                   // when GPS gets an MAV packet
{ GPS_Status.MAV=1;
  GPS_Status.BaudConfig = (GPS_getBaudRate() == GPS_TargetBaudRate);
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
  GPS_Status.Flags = 0;
#ifdef WITH_PPS_IRQ
  GPS_PPS_IRQ_Callback = PPS_IRQ;
#endif

  // PPS_TickCount=0;
  Burst_TickCount=0;

  vTaskDelay(5);

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
    Position[Idx].Clear();
  PosIdx=0;

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
    { if(!GPS_Burst.Active) GPS_BurstStart();                              // burst started
      GPS_Burst.Active=1;
      if( (!GPS_Burst.Complete) && (GPS_Burst.GxGGA && GPS_Burst.GxRMC && GPS_Burst.GxGSA) )
      { GPS_Burst.Complete=1; GPS_BurstComplete(); }
    }
    else if(LineIdle>=GPS_BurstTimeout)                                    // if GPS sends no more data for 10 time ticks
    { if(GPS_Burst.Active)                                                 // if still in burst
      { if(!GPS_Burst.Complete) GPS_BurstComplete();
        GPS_BurstEnd(); }                                                  // burst just ended
      else if(LineIdle>=1000)                                              // if idle for more than 1 sec
      { GPS_Status.Flags=0; }
      GPS_Burst.Flags=0;
    }

    if(NoValidData>=1200)                                                  // if no valid data from GPS for 1sec
    { GPS_Status.Flags=0; GPS_Burst.Flags=0;                                                 // assume GPS state is unknown
      uint32_t NewBaudRate = GPS_nextBaudRate();                           // switch to the next baud rate
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


