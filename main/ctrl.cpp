#include <stdio.h>

#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_system.h"
// #include "esp_sleep.h"

#include "hal.h"

#include "sens.h"
#include "rf.h"
#include "ctrl.h"
#include "proc.h"
#include "log.h"

#include "gps.h"
#include "ubx.h"
#include "timesync.h"
#include "format.h"

#include "disp_oled.h"
#include "disp_lcd.h"

// #include "ymodem.h"

// #define DEBUG_PRINT

static char Line[128];

FIFO<uint8_t, 8> KeyBuffer;

// ========================================================================================================================

void PrintTasks(void (*CONS_UART_Write)(char))
{ char Line[32];

  size_t FreeHeap = xPortGetFreeHeapSize();
  Format_String(CONS_UART_Write, "Task            Pr. Stack, ");
  Format_UnsDec(CONS_UART_Write, (uint32_t)FreeHeap, 4, 3);
  Format_String(CONS_UART_Write, "kB free\n");

  UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
  TaskStatus_t *pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
  if(pxTaskStatusArray==0) return;
  uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, NULL );
  for(UBaseType_t T=0; T<uxArraySize; T++)
  { TaskStatus_t *Task = pxTaskStatusArray+T;
    uint8_t Len=Format_String(Line, Task->pcTaskName, configMAX_TASK_NAME_LEN, 0);
    // for( ; Len<=configMAX_TASK_NAME_LEN; )
    //   Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, Task->uxCurrentPriority, 2); Line[Len++]=' ';
    // Line[Len++]='0'+Task->uxCurrentPriority; Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, Task->usStackHighWaterMark, 3);
    Line[Len++]='\n'; Line[Len]=0;
    Format_String(CONS_UART_Write, Line);
  }
  vPortFree( pxTaskStatusArray );
}

// ========================================================================================================================


// ========================================================================================================================

static NMEA_RxMsg NMEA;
#ifdef WITH_GPS_UBX_PASS
static UBX_RxMsg  UBX;
#endif

static void PrintParameters(void)                               // print parameters stored in Flash
{ Parameters.Print(Line);
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex);                                  // give back UART1 to other tasks
}

#ifdef WITH_CONFIG
static void ReadParameters(void)  // read parameters requested by the user in the NMEA sent.
{ if((!NMEA.hasCheck()) || NMEA.isChecked() )
  { PrintParameters();
    if(NMEA.Parms==0)                                                      // if no parameter given
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                           // print a help message
      // Format_String(CONS_UART_Write, "$POGNS,<aircraft-type>,<addr-type>,<address>,<RFM69(H)W>,<Tx-power[dBm]>,<freq.corr.[kHz]>,<console baudrate[bps]>,<RF temp. corr.[degC]>,<pressure corr.[Pa]>\n");
      Format_String(CONS_UART_Write, "$POGNS[,<Name>=<Value>]\n");
      xSemaphoreGive(CONS_Mutex);                                          //
      return; }
    Parameters.ReadPOGNS(NMEA);
    PrintParameters();
    esp_err_t Err = Parameters.WriteToNVS();                                                  // erase and write the parameters into the Flash
    // if(Parameters.ReadFromNVS()!=ESP_OK) Parameters.setDefault();
    // Parameters.WriteToFlash();                                             // erase and write the parameters into the Flash
    // if(Parameters.ReadFromFlash()<0) Parameters.setDefault();              // read the parameters back: if invalid, set defaults
                                                                              // page erase lasts 20ms tus about 20 system ticks are lost here
  }
  // PrintParameters();
}
#endif

#ifdef WITH_LOG
static void ListLogFile(void)
{ if(NMEA.Parms!=1) return;
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "ListLogFile() ");
  Format_String(CONS_UART_Write, (const char *)NMEA.ParmPtr(0), 0, 12);
  Format_String(CONS_UART_Write, " ");
  Format_UnsDec(CONS_UART_Write, (uint32_t)NMEA.ParmLen(0));
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  uint32_t FileTime = SPIFFSlog_ReadShortFileTime((const char *)NMEA.ParmPtr(0), NMEA.ParmLen(0));
  if(FileTime==0) return;
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "ListLogFile() ");
  Format_Hex(CONS_UART_Write, FileTime);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  SPIFFSlog_ListFile(FileTime);
}
#endif

static void ProcessNMEA(void)     // process a valid NMEA that got to the console
{
#ifdef WITH_CONFIG
  if(NMEA.isPOGNS()) ReadParameters();
#endif
#ifdef WITH_LOG
  if(NMEA.isPOGNL()) ListLogFile();
#endif
}

static uint8_t Verbose=0;
static uint32_t VerboseSuspendTime=0;
const  uint32_t VerboseSuspendTimeout=60;

static void CheckCtrlV(void)
{ if(VerboseSuspendTime==0) return;
  uint32_t Time = TimeSync_Time()-VerboseSuspendTime;
  if(Time>VerboseSuspendTimeout) { Parameters.Verbose=Verbose; VerboseSuspendTime=0; }
}

static void ProcessCtrlV(void)
{ if(VerboseSuspendTime) { Parameters.Verbose=Verbose; VerboseSuspendTime=0; return; }
  Verbose = Parameters.Verbose; VerboseSuspendTime=TimeSync_Time(); Parameters.Verbose=0;
}

static void ProcessCtrlC(void)                                  // print system state to the console
{ xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Parameters.Print(Line);
  Format_String(CONS_UART_Write, Line);
  Format_String(CONS_UART_Write, "GPS: ");
  Format_UnsDec(CONS_UART_Write, GPS_getBaudRate(), 1);
  Format_String(CONS_UART_Write, "bps");
  CONS_UART_Write(',');
  Format_UnsDec(CONS_UART_Write, GPS_PosPeriod, 3, 2);
  CONS_UART_Write('s');
  if(GPS_Status.PPS)         Format_String(CONS_UART_Write, ",PPS");
  if(GPS_Status.NMEA)        Format_String(CONS_UART_Write, ",NMEA");
  if(GPS_Status.UBX)         Format_String(CONS_UART_Write, ",UBX");
  if(GPS_Status.MAV)         Format_String(CONS_UART_Write, ",MAV");
  if(GPS_Status.BaudConfig)  Format_String(CONS_UART_Write, ",BaudOK");
  if(GPS_Status.ModeConfig)  Format_String(CONS_UART_Write, ",ModeOK");
  CONS_UART_Write('\r'); CONS_UART_Write('\n');
  PrintTasks(CONS_UART_Write);                               // print the FreeRTOS tasks

#ifdef WITH_SPIFFS
  char FullName[32];
  strcpy(FullName, "/spiffs/");
  struct stat Stat;
  uint32_t Files=0;                                          // count/list files in SPIFFS
  DIR *Dir=opendir(FullName);                                // open SPIFFS top directory
  if(Dir)
  { for( ; ; )                                               // loop over files
    { struct dirent *Ent = readdir(Dir); if(!Ent) break;     // get the next file of the directory
      if(Ent->d_type != DT_REG) continue;                    // if not a regular file (directory, link, ...): then skip
      char *Name = Ent->d_name;
      strcpy(FullName+8, Name);
      if(stat(FullName, &Stat)<0) continue;
      Format_String(CONS_UART_Write, FullName);
      CONS_UART_Write(' ');
      Format_UnsDec(CONS_UART_Write, (uint32_t)Stat.st_size);
      // if(Stat.st_size==0) { unlink(FullName); }           // remove files with zero length
      Format_String(CONS_UART_Write, "\n");
      Files++; }                                             // count the (regular) files
    closedir(Dir); }
  Format_String(CONS_UART_Write, "SPIFFS: ");
  size_t Total, Used;
  if(SPIFFS_Info(Total, Used)==0)                            // get the SPIFFS usage summary
  { Format_UnsDec(CONS_UART_Write, Used/1024);
    Format_String(CONS_UART_Write, "kB used, ");
    Format_UnsDec(CONS_UART_Write, Total/1024);
    Format_String(CONS_UART_Write, "kB total, "); }
  Format_UnsDec(CONS_UART_Write, Files);
  Format_String(CONS_UART_Write, " files\n");
#endif // WITH_SPIFFS

  Parameters.Write(CONS_UART_Write);                         // write the parameters to the console
  // Parameters.WriteFile(stdout);                                   // write the parameters to the stdout

#ifdef WITH_SD
  Format_String(CONS_UART_Write, "SD card:");
  if(SD_isMounted())
  { Format_UnsDec(CONS_UART_Write, (uint32_t)SD_getSectors());
    CONS_UART_Write('x');
    Format_UnsDec(CONS_UART_Write, (uint32_t)SD_getSectorSize()*5/512, 2, 1);
    Format_String(CONS_UART_Write, "KB"); }
  else
  { Format_String(CONS_UART_Write, " not mounted"); }
  Format_String(CONS_UART_Write, "\n");
#endif

  xSemaphoreGive(CONS_Mutex);
}

static void ProcessCtrlL(void)                                    // print system state to the console
{
#ifdef WITH_SPIFFS
  SPIFFSlog_ListFiles();
#endif
}

void SleepIn(void)
{ xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "Sleep-in\n");
  xSemaphoreGive(CONS_Mutex);

#ifdef WITH_GPS_UBX
#ifdef WITH_GPA_ENA
  GPS_DISABLE();
#endif
  UBX_RXM_PMREQ PMREQ;
  PMREQ.duration = 0;
  PMREQ.flags = 0x01;
  UBX_RxMsg::Send(0x02, 0x41, GPS_UART_Write, (uint8_t *)(&PMREQ), sizeof(PMREQ));
#endif

#ifdef WITH_GPS_MTK
#ifdef WITH_GPA_ENA
  GPS_DISABLE();
  Format_String(GPS_UART_Write, "$PMTK225,4*2F\r\n"); // backup mode: 7uA
#else
  Format_String(GPS_UART_Write, "$PMTK161,0*28\r\n"); // standby mode: 1mA
#endif
#endif

#ifdef WITH_OLED
  OLED_DisplayON(0);
#endif

#ifdef WITH_U8G2_OLED
  u8g2_SetPowerSave(&U8G2_OLED, 1);
#endif

#if defined(WITH_ST7789) || defined(WITH_ILI9341)
  LCD_SetBacklightLevel(0);
#endif

  PowerMode=0;
  for(int Idx=0; Idx<1500; Idx++)
  { LED_TimerCheck(1);
    vTaskDelay(1); }

}

void SleepOut(void)
{
#ifdef WITH_GPS_ENABLE
  GPS_DISABLE();
#endif
  PowerMode=2;
#if defined(WITH_ST7789) || defined(WITH_ILI9341)
  LCD_SetBacklightLevel(6);
#endif
#ifdef WITH_U8G2_OLED
  u8g2_SetPowerSave(&U8G2_OLED, 0);
#endif
#ifdef WITH_OLED
  OLED_DisplayON(1);
#endif

#ifdef WITH_GPS_UBX
#ifdef WITH_GPS_ENABLE
  GPS_ENABLE();
#endif
  Format_String(GPS_UART_Write, "\n");
#endif

#ifdef WITH_GPS_MTK
#ifdef WITH_GPS_ENABLE
  GPS_ENABLE();
#else
  Format_String(GPS_UART_Write, "$PMTK161,1*29\r\n");
#endif
#endif
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "Sleep-out\n");
  xSemaphoreGive(CONS_Mutex); }

#ifdef WITH_SLEEP
static TickType_t LowBatt_Time=0;

static void LowBatt_Watch(void)                                     // check battery voltage
{ uint16_t BattVolt = BatteryVoltage>>8;                            // [mV]
  if(BattVolt>=3250 || BattVolt<=700 ) { LowBatt_Time=0; return; }
   int16_t BattRate = (BatteryVoltageRate+128)>>8;                  // [mv/sec]
  if(BattRate>0) { LowBatt_Time=0; return; }
  TickType_t Now=xTaskGetTickCount();
  if(LowBatt_Time==0) { LowBatt_Time=Now; return; }
  Now-=LowBatt_Time;
  if(Now>=30000)                                                    // if low battery and voltage dropping persists for 30sec
  { SleepIn();                                                      //
    Sleep();                                                        // enter sleep
    SleepOut();                                                     // wake up
    LowBatt_Time=0; }                                               // reset time counter
}
#endif

static void ProcessInput(void)
{ for( ; ; )
  { uint8_t Byte; int Err=CONS_UART_Read(Byte); if(Err<=0) break; // get byte from console, if none: exit the loop
#ifndef WITH_GPS_UBX_PASS
    if(Byte==0x03) ProcessCtrlC();                                // if Ctrl-C received: print parameters
    if(Byte==0x0C) ProcessCtrlL();                                // if Ctrl-L received: list log files
    if(Byte==0x16) ProcessCtrlV();                                // if Ctrl-L received: suspend (verbose) printout
    if(Byte==0x18) esp_restart() ;                                // if Ctrl-X received then restart
    // if(Byte==0x19) Shutdown();                                    // if Ctrl-Y receiver: shutdown
#endif
    NMEA.ProcessByte(Byte);                                       // pass the byte through the NMEA processor
    if(NMEA.isComplete())                                         // if complete NMEA:
    {
#ifdef WITH_GPS_NMEA_PASS
      if(NMEA.isChecked())
        NMEA.Send(GPS_UART_Write);
#endif
      ProcessNMEA();                                              // interpret the NMEA
      NMEA.Clear(); }                                             // clear the NMEA processor for the next sentence
#ifdef WITH_GPS_UBX_PASS
    UBX.ProcessByte(Byte);
    if(UBX.isComplete())
    { UBX.Send(GPS_UART_Write);                                   // is there a need for a Mutex on the GPS UART ?
      UBX.Clear(); }
#endif
  }
}

// ========================================================================================================================

extern "C"
void vTaskCTRL(void* pvParameters)
{
  uint32_t PrevTime=0;
  GPS_Position *PrevGPS=0;
  for( ; ; )                                          //
  { ProcessInput();                                   // process console input
#ifdef WITH_SLEEP
#if defined(WITH_FollowMe) || defined(WITH_TBEAM)
    LowBatt_Watch();
#endif
#endif
    vTaskDelay(1);                                    //

#ifdef WITH_AXP
    bool PowerOffRequest = AXP.readLongPressIRQ() /* || AXP.readShortPressIRQ() */ ;
    if(PowerOffRequest)
    { PowerMode=0;
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "Power-Off Request\n");
      xSemaphoreGive(CONS_Mutex);
      // vTaskDelay(1000);
      AXP.setLED(4);
#ifdef WITH_OLED
      OLED_DisplayON(0);
#endif
#ifdef WITH_U8G2_OLED
      u8g2_SetPowerSave(&U8G2_OLED, 1);
#endif
#if defined(WITH_ST7789) || defined(WITH_ILI9341)
      LCD_SetBacklightLevel(0);
#endif
      AXP.setPowerOutput(AXP.OUT_LDO2,  0); // turn off RFM power
      AXP.setPowerOutput(AXP.OUT_LDO3,  0); // turn off GPS power
      AXP.setPowerOutput(AXP.OUT_DCDC1, 0);
      // AXP.setPowerOutput(AXP.OUT_DCDC2, 0);
      // AXP.setPowerOutput(AXP.OUT_DCDC3, 0);
      // AXP.setPowerOutput(AXP.OUT_EXTEN, 0);
      AXP.ShutDown();
      vTaskDelay(1000);
// #define PIN_AXP_IRQ GPIO_NUM_35
//     esp_sleep_enable_ext0_wakeup(PIN_AXP_IRQ, 0); // 1 = High, 0 = Low
//     esp_deep_sleep_start();
    }
    bool ShortPress = AXP.readShortPressIRQ();
    if(ShortPress)
    { KeyBuffer.Write(0x04);
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "AXP short-press\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      AXP.clearIRQ(); }
#endif

    LED_TimerCheck(1);                                // update the LED flashes
#ifdef WITH_BEEPER
    Play_TimerCheck();                                // update the LED flashes
#endif

    int32_t PressRelease=Button_TimerCheck();         // 0 = no change
// #ifdef DEBUG_PRINT
    if(PressRelease!=0)
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "Button: ");
      Format_SignDec(CONS_UART_Write, PressRelease);
      Format_String(CONS_UART_Write, "ms\n");
      xSemaphoreGive(CONS_Mutex); }
// #endif
    if(PressRelease>0)
    { if(PressRelease<=700)                                            // short button push: switch pages
      { KeyBuffer.Write(0x01); }
      else if(PressRelease<=3000)                                      // longer button push: some page action
      { KeyBuffer.Write(0x41); }
      else                                                             // very long push
      { }
    }

    uint32_t Time=TimeSync_Time();
    bool TimeChange = Time!=PrevTime;
    uint32_t Sec = (Time-1)%60;
    GPS_Position *GPS = GPS_getPosition(Sec);
    bool GPSchange  = GPS!=PrevGPS;
    if( (!TimeChange) && (!GPSchange) ) continue;
    PrevTime=Time; PrevGPS=GPS;

    if(TimeChange) CheckCtrlV();

#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    if(TimeChange)
    { Format_String(CONS_UART_Write, "TimeChange: ");
      // Format_HHMMSS(CONS_UART_Write, Sec);
      Format_HHMMSS(CONS_UART_Write, Time);
      Format_String(CONS_UART_Write, "\n"); }
    if(GPSchange && GPS)
    { Format_String(CONS_UART_Write, "GPSchange: ");
      GPS->PrintLine(Line);
      Format_String(CONS_UART_Write, Line); }
    xSemaphoreGive(CONS_Mutex);
#endif

#ifdef WITH_BQ
#ifdef DEBUG_PRINT
    if(TimeChange)
    { uint16_t Batt = BatterySense(2);
      // uint8_t ID     = BQ.readID();
      // uint8_t Status = BQ.readStatus();
      // uint8_t Fault  = BQ.readFault();
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "BQ24295:");
      for(uint8_t Reg=0; Reg<=10; Reg++)
      { uint8_t Val=BQ.readReg(Reg); CONS_UART_Write(' '); Format_Hex(CONS_UART_Write, Val); }
      // Format_Hex(CONS_UART_Write, ID);
      // CONS_UART_Write('/');
      // Format_Hex(CONS_UART_Write, Status);
      // CONS_UART_Write('/');
      // Format_Hex(CONS_UART_Write, Fault);
      Format_String(CONS_UART_Write, " Batt=");
      Format_UnsDec(CONS_UART_Write, Batt, 4, 3);
      Format_String(CONS_UART_Write, "V\n");
      xSemaphoreGive(CONS_Mutex); }
#endif
#endif

#ifdef WITH_AXP
#ifdef DEBUG_PRINT
    if(TimeChange)
    {  uint16_t Batt=AXP.readBatteryVoltage();       // [mV]
       uint16_t InpCurr=AXP.readBatteryInpCurrent(); // [mA]
       uint16_t OutCurr=AXP.readBatteryOutCurrent(); // [mA]
       uint16_t Vbus=AXP.readVbusVoltage();          // [mV]
       uint16_t VbusCurr=AXP.readVbusCurrent();      // [mA]
        int16_t Temp=AXP.readTemperature();          // [0.1degC]
       uint32_t InpCharge=AXP.readBatteryInpCharge();
       uint32_t OutCharge=AXP.readBatteryOutCharge();
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "AXP192: Batt=");
      Format_UnsDec(CONS_UART_Write, Batt, 4, 3);
      Format_String(CONS_UART_Write, "V, ");
      Format_UnsDec(CONS_UART_Write, InpCurr, 4, 3);
      Format_String(CONS_UART_Write, "/");
      Format_UnsDec(CONS_UART_Write, OutCurr, 4, 3);
      Format_String(CONS_UART_Write, "A, USB: ");
      Format_UnsDec(CONS_UART_Write, Vbus, 4, 3);
      Format_String(CONS_UART_Write, "V, ");
      Format_UnsDec(CONS_UART_Write, VbusCurr, 4, 3);
      Format_String(CONS_UART_Write, "A, Charge=");
      Format_UnsDec(CONS_UART_Write, ((InpCharge<<12)+562)/1125, 2, 1);
      Format_String(CONS_UART_Write, "-");
      Format_UnsDec(CONS_UART_Write, ((OutCharge<<12)+562)/1125, 2, 1);
      Format_String(CONS_UART_Write, "mAh, Temp=");
      Format_SignDec(CONS_UART_Write, Temp, 2, 1);
      Format_String(CONS_UART_Write, "degC\n");
      xSemaphoreGive(CONS_Mutex); }
#endif
#endif

#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    if(TimeChange)
    { Format_String(CONS_UART_Write, "TimeChange: ");
      Format_HHMMSS(CONS_UART_Write, Time);
      Format_String(CONS_UART_Write, "\n"); }
    if(GPSchange && GPS)
    { Format_String(CONS_UART_Write, "GPSchange: ");
      GPS->PrintLine(Line);
      Format_String(CONS_UART_Write, Line); }
    xSemaphoreGive(CONS_Mutex);
#endif

#ifdef DEBUG_PRINT                                      // in debug mode print the parameters and state every 60sec
    if((Time%60)!=0) continue;
    ProcessCtrlC();
#endif

  }
}

// ========================================================================================================================
