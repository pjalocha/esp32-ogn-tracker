#include <stdio.h>

#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_system.h"

#include "hal.h"

#include "rf.h"
#include "ctrl.h"
#include "log.h"

#include "gps.h"
#include "ubx.h"
#include "timesync.h"
#include "format.h"

// #define DEBUG_PRINT

static char Line[128];

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

#ifdef WITH_OLED
int OLED_DisplayStatus(uint32_t Time, uint8_t LineIdx=0)
{ Format_String(Line   , "OGN Tx/Rx      ");
  Format_HHMMSS(Line+10, Time);
  OLED_PutLine(LineIdx++, Line);
  Parameters.Print(Line);
  OLED_PutLine(LineIdx++, Line);
  return 0; }

int OLED_DisplayPosition(GPS_Position *GPS=0, uint8_t LineIdx=2)
{ if(GPS && GPS->isValid())
  { Line[0]=' ';
    Format_SignDec(Line+1,  GPS->Latitude /60, 6, 4); Line[9]=' ';
    Format_UnsDec (Line+10, GPS->Altitude /10, 5, 0); Line[15]='m';
    OLED_PutLine(LineIdx  , Line);
    Format_SignDec(Line,    GPS->Longitude/60, 7, 4);
    Format_SignDec(Line+10, GPS->ClimbRate,    4, 1);
    OLED_PutLine(LineIdx+1, Line);
    Format_UnsDec (Line   , GPS->Speed, 4, 1); Format_String(Line+5, "m/s  ");
    Format_UnsDec (Line+10, GPS->Heading, 4, 1); Line[15]='^';
    OLED_PutLine(LineIdx+2, Line);
    Format_String(Line, "0D/00sat DOP00.0");
    Line[0]+=GPS->FixMode; Format_UnsDec(Line+3, GPS->Satellites, 2);
    Format_UnsDec(Line+12, (uint16_t)GPS->HDOP, 3, 1);
    OLED_PutLine(LineIdx+3, Line);
  }
  else { OLED_PutLine(LineIdx, 0); OLED_PutLine(LineIdx+1, 0); OLED_PutLine(LineIdx+2, 0); OLED_PutLine(LineIdx+3, 0); }
  if(GPS && GPS->isDateValid())
  { Format_UnsDec (Line   , (uint16_t)GPS->Day,   2, 0); Line[2]='.';
    Format_UnsDec (Line+ 3, (uint16_t)GPS->Month, 2, 0); Line[5]='.';
    Format_UnsDec (Line+ 6, (uint16_t)GPS->Year , 2, 0); Line[8]=' '; Line[9]=' '; }
  else Format_String(Line, "          ");
  if(GPS && GPS->isTimeValid())
  { Format_UnsDec (Line+10, (uint16_t)GPS->Hour,  2, 0);
    Format_UnsDec (Line+12, (uint16_t)GPS->Min,   2, 0);
    Format_UnsDec (Line+14, (uint16_t)GPS->Sec,   2, 0);
  } else Line[10]=0;
  OLED_PutLine(LineIdx+4, Line);
  Line[0]=0;
  if(GPS && GPS->hasBaro)
  { Format_String(Line   , "0000.0hPa 00000m");
    Format_UnsDec(Line   , GPS->Pressure/40, 5, 1);
    Format_UnsDec(Line+10, GPS->StdAltitude/10, 5, 0); }
  OLED_PutLine(LineIdx+5, Line);
  return 0; }
#endif

#ifdef WITH_U8G2

void OLED_PutLine(u8g2_t *OLED, uint8_t LineIdx, const char *Line)
{ if(Line==0) return;
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "OLED_PutLine( ,");
  Format_UnsDec(CONS_UART_Write, (uint16_t)LineIdx);
  CONS_UART_Write(',');
  Format_String(CONS_UART_Write, Line);
  Format_String(CONS_UART_Write, ")\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  // u8g2_SetFont(OLED, u8g2_font_5x8_tr);
  u8g2_SetFont(OLED, u8g2_font_amstrad_cpc_extended_8r);
  u8g2_DrawStr(OLED, 0, (LineIdx+1)*8, Line);
}

void OLED_DrawStatus(u8g2_t *OLED, uint32_t Time, uint8_t LineIdx=0)
{ Format_String(Line   , "OGN Tx/Rx      ");
  Format_HHMMSS(Line+10, Time); Line[16]=0;
  OLED_PutLine(OLED, LineIdx++, Line);
  Parameters.Print(Line); Line[16]=0;
  OLED_PutLine(OLED, LineIdx++, Line); }

void OLED_DrawPosition(u8g2_t *OLED, GPS_Position *GPS=0, uint8_t LineIdx=2)
{ if(GPS && GPS->isValid())
  { Line[0]=' ';
    Format_SignDec(Line+1,  GPS->Latitude /60, 6, 4); Line[9]=' ';
    Format_UnsDec (Line+10, GPS->Altitude /10, 5, 0); Line[15]='m';
    OLED_PutLine(OLED, LineIdx  , Line);
    Format_SignDec(Line,    GPS->Longitude/60, 7, 4);
    Format_SignDec(Line+10, GPS->ClimbRate,    4, 1);
    OLED_PutLine(OLED, LineIdx+1, Line);
    Format_UnsDec (Line   , GPS->Speed, 4, 1); Format_String(Line+5, "m/s  ");
    Format_UnsDec (Line+10, GPS->Heading, 4, 1); Line[15]='^';
    OLED_PutLine(OLED, LineIdx+2, Line);
    Format_String(Line, "0D/00sat DOP00.0");
    Line[0]+=GPS->FixMode; Format_UnsDec(Line+3, GPS->Satellites, 2);
    Format_UnsDec(Line+12, (uint16_t)GPS->HDOP, 3, 1);
    OLED_PutLine(OLED, LineIdx+3, Line);
  }
  // else { OLED_PutLine(OLED, LineIdx, 0); OLED_PutLine(OLED, LineIdx+1, 0); OLED_PutLine(LineIdx+2, 0); OLED_PutLine(LineIdx+3, 0); }
  if(GPS && GPS->isDateValid())
  { Format_UnsDec (Line   , (uint16_t)GPS->Day,   2, 0); Line[2]='.';
    Format_UnsDec (Line+ 3, (uint16_t)GPS->Month, 2, 0); Line[5]='.';
    Format_UnsDec (Line+ 6, (uint16_t)GPS->Year , 2, 0); Line[8]=' '; Line[9]=' '; }
  else Format_String(Line, "          ");
  if(GPS && GPS->isTimeValid())
  { Format_UnsDec (Line+10, (uint16_t)GPS->Hour,  2, 0);
    Format_UnsDec (Line+12, (uint16_t)GPS->Min,   2, 0);
    Format_UnsDec (Line+14, (uint16_t)GPS->Sec,   2, 0);
  } else Line[10]=0;
  OLED_PutLine(OLED, LineIdx+4, Line);
  Line[0]=0;
  if(GPS && GPS->hasBaro)
  { Format_String(Line   , "0000.0hPa 00000m");
    Format_UnsDec(Line   , GPS->Pressure/40, 5, 1);
    Format_UnsDec(Line+10, GPS->StdAltitude/10, 5, 0); }
  OLED_PutLine(OLED, LineIdx+5, Line);
}

void OLED_DrawGPS(u8g2_t *OLED, GPS_Position *GPS=0)  // GPS time, position, altitude
{ // u8g2_SetFont(OLED, u8g2_font_ncenB14_tr);
  u8g2_SetFont(OLED, u8g2_font_7x13_tf);              // 5 lines, 12 pixels/line
  uint8_t Len=0;
  Len+=Format_String(Line+Len, "GPS ");
  if(GPS && GPS->isValid())
  { Line[Len++]='0'+GPS->FixMode; Line[Len++]='D'; Line[Len++]='/';
    Len+=Format_UnsDec(Line+Len, GPS->Satellites, 1);
    Len+=Format_String(Line+Len, "sat DOP");
    Len+=Format_UnsDec(Line+Len, (uint16_t)GPS->HDOP, 2, 1); }
  else
  { Len+=Format_String(Line+Len, " wait for lock"); }
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 12, Line);
  if(GPS && GPS->isDateValid())
  { Format_UnsDec (Line   , (uint16_t)GPS->Day,   2, 0); Line[2]='.';
    Format_UnsDec (Line+ 3, (uint16_t)GPS->Month, 2, 0); Line[5]='.';
    Format_UnsDec (Line+ 6, (uint16_t)GPS->Year , 2, 0); Line[8]=' ';
  } else Format_String(Line, "  .  .   ");
  if(GPS && GPS->isTimeValid())
  { Format_UnsDec (Line+ 9, (uint16_t)GPS->Hour,  2, 0); Line[11]=':';
    Format_UnsDec (Line+12, (uint16_t)GPS->Min,   2, 0); Line[14]=':';
    Format_UnsDec (Line+15, (uint16_t)GPS->Sec,   2, 0);
  } else Format_String(Line+9, "  :  :  ");
  Line[17]=0;
  u8g2_DrawStr(OLED, 0, 24, Line);
  Len=0;
  Len+=Format_String(Line+Len, "Lat:  ");
  if(GPS && GPS->isValid())
  { Len+=Format_SignDec(Line+Len,  GPS->Latitude /6, 7, 5);
    Line[Len++]=0xB0; }
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 36, Line);
  Len=0;
  Len+=Format_String(Line+Len, "Lon: ");
  if(GPS && GPS->isValid())
  { Len+=Format_SignDec(Line+Len,  GPS->Longitude /6, 8, 5);
    Line[Len++]=0xB0; }
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 48, Line);
  Len=0;
  Len+=Format_String(Line+Len, "Alt: ");
  if(GPS && GPS->isValid())
  { Len+=Format_SignDec(Line+Len,  GPS->Altitude, 4, 1);
    Line[Len++]='m'; }
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 60, Line);
}

void OLED_DrawRF(u8g2_t *OLED)                      // RF
{ u8g2_SetFont(OLED, u8g2_font_7x13_tf);            // 5 lines. 12 pixels/line
  uint8_t Len=0;
#ifdef WITH_RFM69
  Len+=Format_String(Line+Len, "RFM69");            // Type of RF chip used
  if(isTxTypeHW()) Line[Len++]='H';
  Line[Len++]='W';
#endif
#ifdef WITH_RFM95
  Len+=Format_String(Line+Len, "RFM95");
#endif
#ifdef WITH_SX1272
  Len+=Format_String(Line+Len, "SX1272");
#endif
  Line[Len++]=':';
  Len+=Format_SignDec(Line+Len, (int16_t)Parameters.getTxPower());        // Tx power
  Len+=Format_String(Line+Len, "dBm");
  Line[Len++]=' ';
  Len+=Format_SignDec(Line+Len, (int32_t)Parameters.RFchipFreqCorr, 2, 1); // frequency correction
  Len+=Format_String(Line+Len, "ppm");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 12, Line);
  Len=0;
  Len+=Format_String(Line+Len, "Rx:");                                     //
  Len+=Format_SignDec(Line+Len, -5*RX_AverRSSI, 2, 1);                     // noise level seen by the receiver
  Len+=Format_String(Line+Len, "dBm ");
  Len+=Format_UnsDec(Line+Len, RX_OGN_Count64);                            // received packet/min
  Len+=Format_String(Line+Len, "/min");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 24, Line);
  Len=0;
  Len+=Format_SignDec(Line+Len, (int16_t)RF_Temp);                         // RF chip internal temperature (not calibrated)
  Line[Len++]=0xB0;
  Line[Len++]='C';
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 36, Line);
  // u8g2_DrawStr(OLED, 0, 48, RF_FreqPlan.getPlanName());
  Len=0;
  Len+=Format_String(Line+Len, RF_FreqPlan.getPlanName());                 // name of the frequency plan
  Line[Len++]=' ';
  Len+=Format_UnsDec(Line+Len, (uint16_t)(RF_FreqPlan.getCenterFreq()/100000), 3, 1); // center frequency
  Len+=Format_String(Line+Len, "MHz");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 48, Line);

}

void OLED_DrawBaro(u8g2_t *OLED, GPS_Position *GPS=0)
{ u8g2_SetFont(OLED, u8g2_font_7x13_tf);              // 5 lines, 12 pixels/line
  uint8_t Len=0;
#ifdef WITH_BMP180
  Len+=Format_String(Line+Len, "BMP180 ");
#endif
#ifdef WITH_BMP280
  Len+=Format_String(Line+Len, "BMP280 ");
#endif
#ifdef WITH_BME280
  Len+=Format_String(Line+Len, "BME280 ");
#endif
#ifdef WITH_MS5607
  Len+=Format_String(Line+Len, "MS5607 ");
#endif
  Len+=Format_UnsDec(Line+Len, GPS->Pressure/4, 5, 2);
  Len+=Format_String(Line+Len, "Pa");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 12, Line);
  Len=0;
  Len+=Format_UnsDec(Line+Len, GPS->StdAltitude, 5, 1);
  Len+=Format_String(Line+Len, "m ");
  Len+=Format_SignDec(Line+Len, GPS->ClimbRate, 2, 1);
  Len+=Format_String(Line+Len, "m/s");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 24, Line);
  Len=0;
  Len+=Format_SignDec(Line+Len, GPS->Temperature, 2, 1);
  Line[Len++]=0xB0;
  Line[Len++]='C';
  Line[Len++]=' ';
  Len+=Format_SignDec(Line+Len, GPS->Humidity, 2, 1);
  Line[Len++]='%';
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 36, Line);
}

void OLED_DrawSystem(u8g2_t *OLED)
{
  u8g2_SetFont(OLED, u8g2_font_7x13_tf);              // 5 lines, 12 pixels/line
  uint8_t Len=0;
  Len+=Format_String(Line+Len, "GPS ");
#ifdef WITH_GPS_UBX
  Len+=Format_String(Line+Len, "UBX ");
#endif
#ifdef WITH_GPS_MTK
  Len+=Format_String(Line+Len, "MTK ");
#endif
#ifdef WITH_GPS_SRF
  Len+=Format_String(Line+Len, "SRF ");
#endif
  Len+=Format_UnsDec(Line+Len, GPS_getBaudRate(), 1);
  Len+=Format_String(Line+Len, "bps");
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 12, Line);

#ifdef WITH_SD
  Len=0;
  Len += Format_String(Line+Len, "SD ");
  if(SD_isMounted())
  { Len+=Format_UnsDec(Line+Len, (uint32_t)SD_getSectors());
    Line[Len++]='x';
    Len+=Format_UnsDec(Line+Len, (uint32_t)SD_getSectorSize()*5/512, 2, 1);
    Len+=Format_String(Line+Len, "KB"); }
  else
  { Len+=Format_String(Line+Len, "none"); }
  Line[Len]=0;
  u8g2_DrawStr(OLED, 0, 60, Line);

#endif

}

void OLED_DrawConfig(u8g2_t *OLED)
{ u8g2_SetFont(OLED, u8g2_font_timR18_tr);
  Parameters.Print(Line); Line[10]=0;
  u8g2_DrawStr(OLED, 0, 20, Line);
}

#endif

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

static void ProcessInput(void)
{ for( ; ; )
  { uint8_t Byte; int Err=CONS_UART_Read(Byte); if(Err<=0) break; // get byte from console, if none: exit the loop
#ifndef WITH_GPS_UBX_PASS
    if(Byte==0x03) ProcessCtrlC();                                // if Ctrl-C received
    if(Byte==0x0C) ProcessCtrlL();                                // if Ctrl-C received
    if(Byte==0x18) esp_restart() ;                                // if Ctrl-X received then restart
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

const uint8_t OLED_Pages = 6;
static uint8_t OLED_Page = 1;

extern "C"
void vTaskCTRL(void* pvParameters)
{ uint32_t PrevTime=0;
  GPS_Position *PrevGPS=0;
  for( ; ; )
  { ProcessInput();                                   // process console input

    vTaskDelay(1);                                    //

    LED_TimerCheck(1);                                // update the LED flashes
#ifdef WITH_BEEPER
    Play_TimerCheck(1);                               // read the button(s)
#endif
    bool PageChange=0;
    int32_t PressRelease=Button_TimerCheck();
    if(PressRelease!=0)
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "PressRelease = ");
      Format_SignDec(CONS_UART_Write, PressRelease);
      Format_String(CONS_UART_Write, "ms\n");
      xSemaphoreGive(CONS_Mutex); }
    if(PressRelease>0)
    { if(PressRelease<=300)                                            // short button push: switch pages
      { OLED_Page++; if(OLED_Page>=OLED_Pages) OLED_Page=0;
        PageChange=1; }
      else if(PressRelease<=2000)                                      // long button push: some page action
      { }
    }

    uint32_t Time=TimeSync_Time();
    GPS_Position *GPS = GPS_getPosition();
    bool TimeChange = Time!=PrevTime;
    bool GPSchange  = GPS!=PrevGPS;
    if( (!TimeChange) && (!GPSchange) ) continue;
    PrevTime=Time; PrevGPS=GPS;

#ifdef WITH_OLED
    if(Button_SleepRequest)
    { OLED_DisplayON(0); }
    else
    { esp_err_t StatErr=ESP_OK;
      esp_err_t PosErr=ESP_OK;
      if(TimeChange)
      { StatErr = OLED_DisplayStatus(Time, 0); }
      if(GPSchange)
      { PosErr = OLED_DisplayPosition(GPS, 2); }
    }
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    if(TimeChange)
    { Format_String(CONS_UART_Write, "TimeChange: ");
      Format_SignDec(CONS_UART_Write, StatErr);
      Format_String(CONS_UART_Write, "\n"); }
    if(GPSchange)
    { Format_String(CONS_UART_Write, "GPSchange: ");
      Format_SignDec(CONS_UART_Write, PosErr);
      Format_String(CONS_UART_Write, "\n"); }
    xSemaphoreGive(CONS_Mutex);
#endif
#endif // WITH_OLED

#ifdef WITH_U8G2
    if(Button_SleepRequest)
    { u8g2_SetPowerSave(&U8G2_OLED, 0); }
    else if(TimeChange || PageChange)
    { u8g2_ClearBuffer(&U8G2_OLED);
      switch(OLED_Page)
      { case 1: OLED_DrawGPS   (&U8G2_OLED, GPS); break;
        case 2: OLED_DrawRF    (&U8G2_OLED); break;
        case 3: OLED_DrawBaro  (&U8G2_OLED, GPS); break;
        case 4: OLED_DrawConfig(&U8G2_OLED); break;
        case 5: OLED_DrawSystem(&U8G2_OLED); break;
        default:
        { OLED_DrawStatus(&U8G2_OLED, Time, 0);
          OLED_DrawPosition(&U8G2_OLED, GPS, 2); }
      }
      u8g2_SendBuffer(&U8G2_OLED);
    }
#endif

#ifdef DEBUG_PRINT                                      // in debug mode print the parameters and state every 60sec
    if((Time%60)!=0) continue;
    ProcessCtrlC();
#endif

  }
}

// ========================================================================================================================
