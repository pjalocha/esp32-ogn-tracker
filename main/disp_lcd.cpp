#include <stdio.h>

#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

// #include "esp_system.h"
// #include "esp_sleep.h"

#include "hal.h"

#include "sens.h"
#include "rf.h"
#include "ctrl.h"
#include "proc.h"
// #include "log.h"

#include "gps.h"
// #include "ubx.h"
// #include "timesync.h"
#include "format.h"

// #include "ymodem.h"

// #define DEBUG_PRINT

// static char Line[128];

// ========================================================================================================================

#if defined(WITH_ST7789) || defined(WITH_ILI9341)

#include "st7789.h"
#include "lcd_battery.h"

// Embedded images
// #ifdef WITH_M5_JACEK
// extern const uint8_t OGN_logo_jpg[]   asm("_binary_OGN_logo_320x240_jpg_start");
// extern const uint8_t OGN_logo_end[]   asm("_binary_OGN_logo_320x240_jpg_end");
// const int OGN_logo_size = OGN_logo_end-OGN_logo_jpg;
// #else
extern const uint8_t OGN_logo_jpg[]   asm("_binary_OGN_logo_240x240_jpg_start");
extern const uint8_t OGN_logo_end[]   asm("_binary_OGN_logo_240x240_jpg_end");
const int OGN_logo_size = OGN_logo_end-OGN_logo_jpg;
// #endif

// extern const uint8_t Club_logo_jpg[]   asm("_binary_AP_logo_240x240_jpg_start");
// extern const uint8_t Club_logo_end[]   asm("_binary_AP_logo_240x240_jpg_end");
// const int Club_logo_size = Club_logo_end-Club_logo_jpg;

#ifdef FOR_CKL
extern const uint8_t Club_logo_jpg[]   asm("_binary_CKL_logo_240x240_jpg_start");
extern const uint8_t Club_logo_end[]   asm("_binary_CKL_logo_240x240_jpg_end");
const int Club_logo_size = Club_logo_end-Club_logo_jpg;
#endif

// uint8_t LCD_Backlight = 8*16;

static void LCD_UpdateTime(uint32_t Time, GPS_Position *GPS, bool Redraw=0)
{ static char     Msg[2][10] = { { 0 }, { 0 } };
  static uint16_t Back[2] = { 0, 0 };
  static bool     Idx = 0;
  Back[Idx] = RGB565_LIGHTRED;
  if(GPS && GPS->isTimeValid())
  { if(GPS->isDateValid()) { Time=GPS->getUnixTime(); Back[Idx]=RGB565_GREEN; }
                      else { Time=GPS->getDayTime();  Back[Idx]=RGB565_GREENYELLOW; }
    if(GPS->FracSec>=50) Time++; }
  uint8_t Len=Format_HHcMMcSS(Msg[Idx], Time); Msg[Idx][Len]=0;
  bool Redo = Redraw || Back[Idx]!=Back[Idx^1];
  if(Redo) LCD_DrawString(Msg[Idx], LCD_WIDTH-LCD_StringWidth(Msg[Idx])-4, LCD_HEIGHT-LCD_FontHeight(), RGB565_BLACK, Back[Idx]);
      else LCD_UpdateString(Msg[Idx], Msg[Idx^1], LCD_WIDTH-LCD_StringWidth(Msg[Idx])-4, LCD_HEIGHT-LCD_FontHeight(), RGB565_BLACK, Back[Idx]);
  Idx^=1; }

static LCD_BattSymb BattSymb;

static uint8_t BattCapacity(uint16_t mVolt)
{ if(mVolt>=4100) return 100;                                 // 4.1V or above => full capacity
  if(mVolt<=3600) return   0;                                 // 3.6V or below => zero capacity
  return (mVolt-3600+2)/5; }                                  // linear dependence (simplified)

static void LCD_UpdateBattery(bool Redraw=0)
{ // static char     Volt[2][8] = { { 0 }, { 0 } };
  // static uint16_t Back[2] = { 0, 0 };
  static uint16_t BattCol[2] = { 0, 0 };
  static bool     Idx = 0;
  uint16_t mVolt = (BatteryVoltage+128)>>8;
  // uint16_t Voltage = (mVolt+5)/10;                            // [0.01V]
  uint8_t Capacity = BattCapacity(mVolt);                     // [mv] => [%]
#ifdef WITH_AXP
  static char     Curr[2][8] = { { 0 }, { 0 } };
  uint16_t InpCurr=AXP.readBatteryInpCurrent(); // [mA]
  uint16_t OutCurr=AXP.readBatteryOutCurrent(); // [mA]
   int16_t Current = InpCurr-OutCurr;
  uint8_t Charging = Current>0;
#endif
/*
  Back[Idx] = RGB565_LIGHTRED;
  if(Voltage>=350) Back[Idx] = RGB565_YELLOW;
  if(Voltage>=365) Back[Idx] = RGB565_GREENYELLOW;
  if(Voltage>=375) Back[Idx] = RGB565_GREEN;
  if(Voltage>=410) Back[Idx] = RGB565_CYAN;
  if(Voltage>=420) Back[Idx] = RGB565_LIGHTBLUE;
  if(Voltage>=425) Back[Idx] = RGB565_MAGENTA;
  Format_UnsDec(Volt[Idx], Voltage, 3, 2); Volt[Idx][4]='V'; Volt[Idx][5]=0;
  bool Redo = Redraw || Back[Idx]!=Back[Idx^1];
  if(Redo) LCD_DrawString(Volt[Idx], 4, LCD_HEIGHT-LCD_FontHeight(), RGB565_BLACK, Back[Idx]);
      else LCD_UpdateString(Volt[Idx], Volt[Idx^1], 4, LCD_HEIGHT-LCD_FontHeight(), RGB565_BLACK, Back[Idx]);
*/
  uint8_t BattLev=(Capacity+10)/20;
#ifdef WITH_AXP
  static uint8_t DispLev = 0;
  if(Charging) { DispLev++; if(DispLev>5) DispLev = BattLev?BattLev-1:0; }
         else  { DispLev = BattLev; }
#else
  uint8_t &DispLev = BattLev;
#endif

  // static uint8_t Level = 0;
  const uint16_t LevelCol[6] = { RGB565_RED, RGB565_RED, RGB565_ORANGE, RGB565_YELLOW, RGB565_GREENYELLOW, RGB565_GREEN };
  // const uint16_t LevelCol[6] = { RGB565_RED, RGB565_RED, RGB565_DARKORANGE, RGB565_DARKYELLOW, RGB565_DARKGREENYELLOW, RGB565_DARKGREEN };
  BattSymb.setLevel(DispLev);
#ifdef PRINT_DEBUG
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "LCD_UpdateBattery() ");
  Format_Hex(CONS_UART_Write, BattSymb.CellMap);
  CONS_UART_Write('/');
  Format_Hex(CONS_UART_Write, BattSymb.Flags);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  BattCol[Idx]=LevelCol[BattLev];
  if(Redraw || BattCol[Idx]!=BattCol[Idx^1])
  { BattSymb.Xpos=0; BattSymb.Ypos=LCD_HEIGHT-BattSymb.Width;
    BattSymb.FrameCol=RGB565_DARKGREY; BattSymb.FillCol=RGB565_LIGHTGREY; BattSymb.CellCol=BattCol[Idx];
    if(BattSymb.CellCol==RGB565_RED) BattSymb.FrameCol=RGB565_RED;
    BattSymb.Draw(); }
  else
   BattSymb.Update();
  // Level+=1; if(Level>5) Level=0;

#ifndef FOR_CKL
#ifdef WITH_AXP
  strcpy(Curr[Idx], "    mA ");
  Format_SignDec(Curr[Idx], Current, 3);
  bool Redo = Redraw || Curr[Idx][0]!=Curr[Idx^1][0];
  if(Redo)
    LCD_DrawString(Curr[Idx], 0, LCD_HEIGHT-LCD_FontHeight()-LCD_FontHeight(tft_Dejavu18), RGB565_BLACK, RGB565_WHITE, tft_Dejavu18);
  else
    LCD_UpdateString(Curr[Idx], Curr[Idx^1], 0, LCD_HEIGHT-LCD_FontHeight()-LCD_FontHeight(tft_Dejavu18), RGB565_BLACK, RGB565_WHITE, tft_Dejavu18);
#endif
#endif
  Idx^=1; }

static void LCD_UpdateRX(bool Redraw=0)
{ static char     Count[2][12] = { { 0 }, { 0 } };
  static char     Noise[2][12] = { { 0 }, { 0 } };
  static bool     Idx = 0;
  uint8_t Len=0;
  Len+=Format_String(Count[Idx]+Len, "  Rx:");                                   //
  Len+=Format_UnsDec(Count[Idx]+Len, RX_OGN_Count64);                            // received packet/min
  Len+=Format_String(Count[Idx]+Len, "/min");
  Count[Idx][Len]=0;
  if(Redraw) LCD_DrawString(Count[Idx], LCD_WIDTH-LCD_StringWidth(Count[Idx], tft_Dejavu18), 4, RGB565_BLACK, RGB565_WHITE, tft_Dejavu18);
        else LCD_UpdateString(Count[Idx], Noise[Idx^1], LCD_WIDTH-LCD_StringWidth(Count[Idx], tft_Dejavu18), 4, RGB565_BLACK, RGB565_WHITE, tft_Dejavu18);
  Len=0;
  Len+=Format_String(Noise[Idx]+Len, " ");
  Len+=Format_SignDec(Noise[Idx]+Len, -5*TRX.averRSSI, 4, 1);                    // noise level seen by the receiver
  Len+=Format_String(Noise[Idx]+Len, "dBm");
  Noise[Idx][Len]=0;
  if(Redraw) LCD_DrawString(Noise[Idx], LCD_WIDTH-LCD_StringWidth(Noise[Idx], tft_Dejavu18), 4+LCD_FontHeight(tft_Dejavu18), RGB565_BLACK, RGB565_WHITE, tft_Dejavu18);
        else LCD_UpdateString(Noise[Idx], Noise[Idx^1], LCD_WIDTH-LCD_StringWidth(Noise[Idx], tft_Dejavu18), 4+LCD_FontHeight(tft_Dejavu18), RGB565_BLACK, RGB565_WHITE, tft_Dejavu18);
  Idx^=1; }

static void LCD_UpdateGPS(GPS_Position *GPS, bool Redraw=0)
{ static char     Sat[2][8] = { { 0 }, { 0 } };
  static uint16_t Back[2] = { 0, 0 };
  // static char     SNR[2][8] = { { 0 }, { 0 } };
  static char     Alt[2][8] = { { 0 }, { 0 } };
  static bool     Idx = 0;
  Back[Idx] = RGB565_LIGHTRED;
  if(GPS)
  { uint16_t Sats = 0;
    if(GPS->isValid()) Sats = GPS->Satellites;
         if(Sats>=5) Back[Idx] = RGB565_GREEN;
    else if(Sats>=4) Back[Idx] = RGB565_GREENYELLOW;
    else if(Sats>=3) Back[Idx] = RGB565_YELLOW;
    if(GPS->Sec&3) { Format_UnsDec(Sat[Idx], Sats, 2); Format_String(Sat[Idx]+2, "sat"); }
              else { Format_UnsDec(Sat[Idx], (GPS_SatSNR+2)/4, 2); Format_String(Sat[Idx]+2, "dB "); }
    Sat[Idx][5]=0; }
  else // no data from GPS ?
  { Format_String(Sat[Idx], "GPS ?"); Sat[Idx][5]=0; }
  bool Redo = Redraw || Back[Idx]!=Back[Idx^1];
  if(Redo) LCD_DrawString(Sat[Idx], 0, 0, RGB565_BLACK, Back[Idx]);
      else LCD_UpdateString(Sat[Idx], Sat[Idx^1], 0, 0, RGB565_BLACK, Back[Idx]);
  if(GPS && GPS->isValid())
  { int32_t Altitude=GPS->Altitude; if(Altitude<0) Altitude=0; Altitude=(Altitude+5)/10;
    uint8_t Len=Format_UnsDec(Alt[Idx], (uint32_t)Altitude);
    Len+=Format_String(Alt[Idx]+Len, "m   ");
    Alt[Idx][Len]=0;
#ifdef FOR_CKL
    if(Redraw) LCD_DrawString(Alt[Idx], LCD_WIDTH-6*16, 0, RGB565_BLACK, RGB565_WHITE);
          else LCD_UpdateString(Alt[Idx], Alt[Idx^1], LCD_WIDTH-6*16, 0, RGB565_BLACK, RGB565_WHITE);
#else
    if(Redraw) LCD_DrawString(Alt[Idx], 0, LCD_FontHeight(), RGB565_BLACK, RGB565_WHITE);
          else LCD_UpdateString(Alt[Idx], Alt[Idx^1], 0, LCD_FontHeight(), RGB565_BLACK, RGB565_WHITE);
#endif
  }
  else
  { }
  Idx^=1; }

static void LCD_UpdatePosition(GPS_Position *GPS, bool Redraw=0)
{ static bool Idx = 0;
  int PosY = 0;

  static char Lock[2][20] = { { 0 }, { 0 } };
  if(GPS)
  { strcpy(Lock[Idx], "0/00sat/00dB/00.0 ");
    Lock[Idx][0] = '0'+GPS->FixQuality;
    Format_UnsDec(Lock[Idx]+2, GPS->Satellites, 2);
    Format_UnsDec(Lock[Idx]+8, (GPS_SatSNR+2)/4, 2);
    Format_UnsDec(Lock[Idx]+13, GPS->HDOP, 3, 1); }
  else
  { strcpy(Lock[Idx], "No data from GPS "); }
  if(Redraw) LCD_DrawString(Lock[Idx], 4, PosY, RGB565_BLACK, RGB565_WHITE);
        else LCD_UpdateString(Lock[Idx], Lock[Idx^1], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();

  static char Time[2][18] = { { 0 }, { 0 } };
  static char Lat [2][18] = { { 0 }, { 0 } };
  static char Lon [2][18] = { { 0 }, { 0 } };
  static char Alt [2][18] = { { 0 }, { 0 } };
  static char Vrt [2][18] = { { 0 }, { 0 } };
  static char Spd [2][18] = { { 0 }, { 0 } };
  static char Trk [2][18] = { { 0 }, { 0 } };
  static char Trn [2][18] = { { 0 }, { 0 } };
  strcpy(Time[Idx], "00.00.0000 000000");
  strcpy(Lat[Idx], "Lat:   __._____  ");
  strcpy(Lon[Idx], "Lon:  ___._____  ");
  strcpy(Alt[Idx], "Alt:   ____._m   ");
  strcpy(Vrt[Idx], "Vrt:  __._ m/s   ");
  strcpy(Spd[Idx], "Spd:  __._ m/s   ");
  strcpy(Trk[Idx], "Trk: ___._ deg   ");
  strcpy(Trn[Idx], "Trn:  __._ deg/s ");
  if(GPS)
  { if(GPS->isTimeValid())
    { Format_UnsDec (Time[Idx]+11, (uint16_t)GPS->Hour,  2, 0);
      Format_UnsDec (Time[Idx]+13, (uint16_t)GPS->Min,   2, 0);
      Format_UnsDec (Time[Idx]+15, (uint16_t)GPS->Sec,   2, 0); }
    if(GPS->isDateValid())
    { Format_UnsDec (Time[Idx]   , (uint16_t)GPS->Day,   2, 0);
      Format_UnsDec (Time[Idx]+ 3, (uint16_t)GPS->Month, 2, 0);
      Format_UnsDec (Time[Idx]+ 6, (uint16_t)GPS->Year , 4, 0); }
    Format_SignDec(Lat[Idx]+6, GPS->Latitude /6, 7, 5);
    Format_SignDec(Lon[Idx]+5, GPS->Longitude/6, 8, 5);
    Format_SignDec(Alt[Idx]+6, GPS->Altitude,  5, 1);
    Format_UnsDec (Spd[Idx]+5, GPS->Speed,     3, 1);
    Format_SignDec(Vrt[Idx]+5, GPS->ClimbRate, 3, 1);
    Format_UnsDec (Trk[Idx]+5, GPS->Heading,   4, 1);
    Format_SignDec(Trn[Idx]+5, GPS->TurnRate,  3, 1); }
  if(Redraw)
    LCD_DrawString(Time[Idx], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  else
    LCD_UpdateString(Time[Idx], Time[Idx^1], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();
  if(Redraw || Lat[Idx][6]!=Lat[Idx^1][6])
    LCD_DrawString(Lat[Idx], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  else
    LCD_UpdateString(Lat[Idx], Lat[Idx^1], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();
  if(Redraw || Lon[Idx][5]!=Lon[Idx^1][5])
    LCD_DrawString(Lon[Idx], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  else
    LCD_UpdateString(Lon[Idx], Lon[Idx^1], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();
  if(Redraw || Alt[Idx][6]!=Alt[Idx^1][6])
    LCD_DrawString(Alt[Idx], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  else
    LCD_UpdateString(Alt[Idx], Alt[Idx^1], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();
  if(Redraw || Vrt[Idx][5]!=Vrt[Idx^1][5])
    LCD_DrawString(Vrt[Idx], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  else
    LCD_UpdateString(Vrt[Idx], Vrt[Idx^1], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();
  if(Redraw)
    LCD_DrawString(Spd[Idx], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  else
    LCD_UpdateString(Spd[Idx], Spd[Idx^1], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();
  if(Redraw)
    LCD_DrawString(Trk[Idx], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  else
    LCD_UpdateString(Trk[Idx], Trk[Idx^1], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();
  if(Redraw || Trn[Idx][5]!=Trn[Idx^1][5])
    LCD_DrawString(Trn[Idx], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  else
    LCD_UpdateString(Trn[Idx], Trn[Idx^1], 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();

  Idx^=1; }

static void LCD_UpdateSys(bool Redraw=0)
{ if(!Redraw) return;
  int PosY = 2;
  char Line[24];
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
  LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();

  Len=0;
#ifdef WITH_RFM69
  Len+=Format_String(Line+Len, "RFM69 v");            // Type of RF chip used
  if(Parameters.isTxTypeHW()) Line[Len++]='H';
  Line[Len++]='W';
#endif
#ifdef WITH_RFM95
  Len+=Format_String(Line+Len, "RFM95 v");
#endif
#ifdef WITH_SX1272
  Len+=Format_String(Line+Len, "SX1272 v");
#endif
  Len+=Format_Hex(Line+Len, TRX.chipVer);
  // Line[Len++]=' ';
  // Len+=Format_SignDec(Line+Len, (int16_t)TRX.chipTemp);
  // Line[Len++]=0xB0;
  // Line[Len++]='C';
  Line[Len]=0;
  LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();

  Len=0;
#ifdef WITH_BMP180
  Len+=Format_String(Line+Len, "BMP180 0x");
  Len+=Format_Hex(Line+Len, Baro.ADDR);
#endif
#ifdef WITH_BMP280
  Len+=Format_String(Line+Len, "BMP280 0x");
  Len+=Format_Hex(Line+Len, Baro.ADDR);
#endif
#ifdef WITH_BME280
  Len+=Format_String(Line+Len, "BME280 0x");
  Len+=Format_Hex(Line+Len, Baro.ADDR);
#endif
#ifdef WITH_MS5607
  Len+=Format_String(Line+Len, "MS5607 0x");
  Len+=Format_Hex(Line+Len, Baro.ADDR);
#endif
#ifdef WITH_MS5611
  Len+=Format_String(Line+Len, "MS5611 0x");
  Len+=Format_Hex(Line+Len, Baro.ADDR);
#endif
  Line[Len]=0;
  LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();

  Len=0;
#ifdef WITH_ST7789
  Len+=Format_String(Line+Len, "ST7789 240x240");
#endif
#ifdef WITH_ILI9341
  Len+=Format_String(Line+Len, "ILI9341 320x240");
#endif
  Line[Len]=0;
  LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();

#ifdef WITH_SPIFFS
  Len=0;
  Len+=Format_String(Line+Len, "SPIFFS ");
  size_t Total, Used;
  if(SPIFFS_Info(Total, Used)==0)                            // get the SPIFFS usage summary
  { Len+=Format_UnsDec(Line+Len, (Total-Used)/1024);
    Len+=Format_String(Line+Len, "/");
    Len+=Format_UnsDec(Line+Len, Total/1024);
    Len+=Format_String(Line+Len, "kB"); }
  Line[Len]=0;
  LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();
#endif

#ifdef WITH_SD
  Len=0;
  Len+=Format_String(Line+Len, "SD ");
  if(SD_isMounted())
  { Len+=Format_UnsDec(Line+Len, (uint32_t)SD_getSectors());
    Line[Len++]='x';
    Len+=Format_UnsDec(Line+Len, (uint32_t)SD_getSectorSize()*5/512, 2, 1);
    Len+=Format_String(Line+Len, "KB"); }
  else
  { Len+=Format_String(Line+Len, "none"); }
  Line[Len]=0;
  LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();
#endif


}

static void LCD_UpdateRF(bool Redraw=0)
{ static bool Idx = 0;
  if(!Redraw) return;

  int PosY = 2;
  char Line[24];
  uint8_t Len=0;
#ifdef WITH_RFM69
  Len+=Format_String(Line+Len, "RFM69");            // Type of RF chip used
  if(Parameters.isTxTypeHW()) Line[Len++]='H';
  Line[Len++]='W';
#endif
#ifdef WITH_RFM95
  Len+=Format_String(Line+Len, "RFM95");
#endif
#ifdef WITH_SX1272
  Len+=Format_String(Line+Len, "SX1272");
#endif
  Line[Len++]=':'; Line[Len++]=' ';
  Len+=Format_SignDec(Line+Len, (int16_t)Parameters.getTxPower());        // Tx power
  Len+=Format_String(Line+Len, "dBm");
  Line[Len]=0;
  LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();

  Len=0;
  Len+=Format_UnsDec(Line+Len, (uint16_t)(RF_FreqPlan.getCenterFreq()/100000), 3, 1); // center frequency
  Len+=Format_String(Line+Len, "MHz");
  Line[Len++]=' ';
  Len+=Format_String(Line+Len, RF_FreqPlan.getPlanName());                 // name of the frequency plan
  Line[Len]=0;
  LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();

  Len=0;
  Len+=Format_SignDec(Line+Len, (int32_t)Parameters.RFchipFreqCorr, 2, 1); // frequency correction
  Len+=Format_String(Line+Len, "ppm");
  Line[Len]=0;
  LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();


  Idx^=1; }

static void LCD_UpdatePower(bool Redraw=0)
{ static bool Idx = 0;

  static char USB[2][20] = { { 0 }, { 0 } };

  Idx^=1; }

static void LCD_UpdateParameters(bool Redraw=0)
{ if(!Redraw) return;
  char Line[32];
  int PosY = 2;
  uint8_t Len=Format_String(Line, "ID=");
  Line[Len++]=HexDigit(Parameters.AcftType); Line[Len++]=':';
  Line[Len++]=HexDigit(Parameters.AddrType); Line[Len++]=':';
  Len+=Format_Hex(Line+Len, Parameters.Address, 6);
  Line[Len]=0;
  LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
  PosY+=LCD_FontHeight();

  for(uint8_t Idx=0; Idx<Parameters.InfoParmNum; Idx++)
  { if(PosY>(LCD_HEIGHT-2*LCD_FontHeight())) break;
    const char *Value = Parameters.InfoParmValue(Idx); if(Value[0]==0) continue;
    uint8_t Len=Format_String(Line, OGN_Packet::InfoParmName(Idx));
    Line[Len++]='=';
    Len+=Format_String(Line+Len, Value);
    Line[Len]=0;
    LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
    PosY+=LCD_FontHeight(); }

}

static void LCD_UpdateRelayList(bool Redraw=0)
{ static uint8_t PrevLines=0;
  if(Redraw) PrevLines=0;
  char Line[24];
  int PosY = 2;
  uint8_t Lines=0;
  for( uint8_t Idx=0; Idx<RelayQueueSize; Idx++)
  { if(PosY>(LCD_HEIGHT-2*LCD_FontHeight())) break;
    OGN_RxPacket<OGN_Packet> *Packet = RelayQueue.Packet+Idx; if(Packet->Rank==0) continue;
    uint8_t Len=0;
    Line[Len++]='0'+Packet->Packet.Header.AddrType;
    Line[Len++]=':';
    Len+=Format_Hex(Line+Len, Packet->Packet.Header.Address, 6);
    Line[Len++]=' ';
    Len+=Format_Hex(Line+Len, Packet->Rank);
    Line[Len++]=' ';
    Line[Len++]=':';
    Len+=Format_UnsDec(Line+Len, Packet->Packet.Position.Time, 2);
    Line[Len]=0;
    LCD_DrawString(Line, 4, PosY, RGB565_BLACK, RGB565_WHITE);
    PosY+=LCD_FontHeight(); Lines++; }
  for(uint8_t Line=Lines; Line<PrevLines; Line++)
  { if(PosY>(LCD_HEIGHT-2*LCD_FontHeight())) break;
    LCD_DrawBox(0, PosY, LCD_WIDTH, LCD_FontHeight(), RGB565_WHITE);
    PosY+=LCD_FontHeight(); }
  PrevLines=Lines; }

static void LCD_UpdateLookList(bool Redraw=0)
{ static uint8_t PrevLines=0;
  if(Redraw) PrevLines=0;
  char Line[24];
  int PosY = 2;
  uint8_t Lines=0;
  for( uint8_t Idx=0; Idx<Look.MaxTargets; Idx++)
  { if(PosY>(LCD_HEIGHT-2*LCD_FontHeight())) break;
    const LookOut_Target *Tgt = Look.Target+Idx; if(!Tgt->Alloc) continue;
    uint16_t Bkg = RGB565_GREEN;
    uint8_t Len=0;
    Len+=Format_Hex(Line+Len, Tgt->ID, 7);
    Line[Len++]=' ';
    if(Tgt->DistMargin) continue;
    uint8_t Warn = Tgt->WarnLevel;
         if(Warn>=3) Bkg = RGB565_LIGHTRED;
    else if(Warn==2) Bkg = RGB565_ORANGE;
    else if(Warn==1) Bkg = RGB565_YELLOW;
    Len+=Format_UnsDec(Line+Len, Tgt->TimeMargin>>1, 2);
    Line[Len++]='s'; Line[Len++]=' ';
    // Len+=Format_UnsDec(Line+Len, ((Tgt->MissDist>>1)+50)/100, 2, 1);
    Len+=Format_UnsDec(Line+Len, ((Tgt->HorDist>>1)+50)/100, 2, 1);
    Line[Len++]='k'; Line[Len++]='m'; Line[Len++]=' '; Line[Len]=0;
    LCD_DrawString(Line, 4, PosY, RGB565_BLACK, Bkg);
    PosY+=LCD_FontHeight(); Lines++; }
  for( uint8_t Idx=0; Idx<Look.MaxTargets; Idx++)
  { if(PosY>(LCD_HEIGHT-2*LCD_FontHeight())) break;
    const LookOut_Target *Tgt = Look.Target+Idx; if(!Tgt->Alloc) continue;
    uint16_t Bkg = RGB565_GREEN;
    uint8_t Len=0;
    Len+=Format_Hex(Line+Len, Tgt->ID, 7);
    Line[Len++]=' ';
    if(Tgt->DistMargin==0) continue;
    Line[Len++]=' '; Line[Len++]=' '; Line[Len++]=' '; Line[Len++]=' '; Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, ((Tgt->HorDist>>1)+50)/100, 2, 1);
    Line[Len++]='k'; Line[Len++]='m'; Line[Len++]=' '; Line[Len]=0;
    LCD_DrawString(Line, 4, PosY, RGB565_BLACK, Bkg);
    PosY+=LCD_FontHeight(); Lines++; }
  for(uint8_t Line=Lines; Line<PrevLines; Line++)
  { if(PosY>(LCD_HEIGHT-2*LCD_FontHeight())) break;
    LCD_DrawBox(0, PosY, LCD_WIDTH, LCD_FontHeight(), RGB565_WHITE);
    PosY+=LCD_FontHeight(); }
  PrevLines=Lines; }

static void LCD_DrawID(void)
{ const uint8_t *Font = tft_Dejavu18;
  char ID[16];
  uint8_t Len=0;
  ID[Len++]=HexDigit(Parameters.AcftType); ID[Len++]=':';
  ID[Len++]=HexDigit(Parameters.AddrType); ID[Len++]=':';
  Len+=Format_Hex(ID+Len, Parameters.Address, 6);
  ID[Len]=0;
  int PosX = LCD_WIDTH -LCD_StringWidth(ID, Font) -4;
  int PosY = LCD_HEIGHT -LCD_FontHeight(Font) -LCD_FontHeight();
  LCD_DrawTranspString(ID, PosX, PosY, RGB565_BLUE, Font);

}

// ------------------------------------------------------------------------------------------------------------------------

void LCD_LogoPage_Draw(void)                                                                 // Logo page: initial background draw
{ if(LCD_WIDTH!=240) LCD_ClearDisplay(RGB565_WHITE);
#ifdef FOR_CKL
  LCD_DrawJPEG(Club_logo_jpg, Club_logo_size, (LCD_WIDTH-240)/2, 0);
#else
// #ifdef WITH_M5_JACEK
//   LCD_DrawJPEG( OGN_logo_jpg,  OGN_logo_size, (LCD_WIDTH-320)/2, 0);
// #else
  LCD_DrawJPEG( OGN_logo_jpg,  OGN_logo_size, (LCD_WIDTH-240)/2, 0);
// #endif
  LCD_DrawID();
#endif
}

void LCD_LogoPage_Draw(uint32_t Time, GPS_Position *GPS)                                     // Logo page: initial draw with Time/GPS data
{ LCD_LogoPage_Draw();
  LCD_UpdateTime(Time, GPS, 1); LCD_UpdateBattery(1);
#ifndef FOR_CKL
  LCD_UpdateRX(1);
#endif
  LCD_UpdateGPS(GPS, 1); }

void LCD_LogoPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange)  // Logo page: update the data in the corners
{ if(TimeChange)
  { LCD_UpdateTime(Time, GPS);
    LCD_UpdateBattery();
#ifndef FOR_CKL
    LCD_UpdateRX();
#endif
  }
  if(GPSchange) { LCD_UpdateGPS(GPS); }
}

void LCD_GPSpage_Draw(void)
{ LCD_ClearDisplay(RGB565_WHITE); }

void LCD_GPSpage_Draw(uint32_t Time, GPS_Position *GPS)
{ LCD_GPSpage_Draw();
  LCD_UpdateTime(Time, GPS, 1); LCD_UpdateBattery(1);
  LCD_UpdatePosition(GPS, 1); }

void LCD_GPSpage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange)
{ if(TimeChange) { LCD_UpdateTime(Time, GPS); LCD_UpdateBattery(); }
  if(GPSchange) LCD_UpdatePosition(GPS); }


void LCD_RFpage_Draw(void)
{ LCD_ClearDisplay(RGB565_WHITE); }

void LCD_RFpage_Draw(uint32_t Time, GPS_Position *GPS)
{ LCD_RFpage_Draw();
  LCD_UpdateTime(Time, GPS, 1); LCD_UpdateBattery(1);
  LCD_UpdateRF(1); }

void LCD_RFpage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange)
{ if(TimeChange) { LCD_UpdateTime(Time, GPS); LCD_UpdateBattery(); }
  if(TimeChange) LCD_UpdateRF(); }


void LCD_BattPage_Draw(void)
{ LCD_ClearDisplay(RGB565_WHITE); }

void LCD_BattPage_Draw(uint32_t Time, GPS_Position *GPS)
{ LCD_BattPage_Draw();
  LCD_UpdateTime(Time, GPS, 1); LCD_UpdateBattery(1);
}

void LCD_BattPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange)
{ if(TimeChange) { LCD_UpdateTime(Time, GPS); LCD_UpdateBattery(); }
}


void LCD_ParmPage_Draw(void)
{ LCD_ClearDisplay(RGB565_WHITE); }

void LCD_ParmPage_Draw(uint32_t Time, GPS_Position *GPS)
{ LCD_ParmPage_Draw();
  LCD_UpdateTime(Time, GPS, 1); LCD_UpdateBattery(1);
  LCD_UpdateParameters(1); }

void LCD_ParmPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange)
{ if(TimeChange) { LCD_UpdateTime(Time, GPS); LCD_UpdateBattery(); }
  LCD_UpdateParameters(); }


void LCD_BaroPage_Draw(void)
{ LCD_ClearDisplay(RGB565_WHITE); }

void LCD_BaroPage_Draw(uint32_t Time, GPS_Position *GPS)
{ LCD_BaroPage_Draw();
  LCD_UpdateTime(Time, GPS, 1); LCD_UpdateBattery(1);
}

void LCD_BaroPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange)
{ if(TimeChange) { LCD_UpdateTime(Time, GPS); LCD_UpdateBattery(); }
}


void LCD_SysPage_Draw(void)
{ LCD_ClearDisplay(RGB565_WHITE); }

void LCD_SysPage_Draw(uint32_t Time, GPS_Position *GPS)
{ LCD_SysPage_Draw();
  LCD_UpdateTime(Time, GPS, 1); LCD_UpdateBattery(1);
  LCD_UpdateSys(1); }

void LCD_SysPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange)
{ if(TimeChange) { LCD_UpdateTime(Time, GPS); LCD_UpdateBattery(); }
  if(TimeChange) LCD_UpdateSys(); }


void LCD_LookPage_Draw(void)
{ LCD_ClearDisplay(RGB565_WHITE); }

void LCD_LookPage_Draw(uint32_t Time, GPS_Position *GPS)
{ LCD_LookPage_Draw();
  LCD_UpdateTime(Time, GPS, 1); LCD_UpdateBattery(1);
  LCD_UpdateLookList(1); }

void LCD_LookPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange)
{ if(TimeChange) { LCD_UpdateTime(Time, GPS); LCD_UpdateBattery(); }
  if(GPSchange) LCD_UpdateLookList(); }


void LCD_RelayPage_Draw(void)
{ LCD_ClearDisplay(RGB565_WHITE); }

void LCD_RelayPage_Draw(uint32_t Time, GPS_Position *GPS)
{ LCD_RelayPage_Draw();
  LCD_UpdateTime(Time, GPS, 1); LCD_UpdateBattery(1);
  LCD_UpdateRelayList(1); }

void LCD_RelayPage_Update(uint32_t Time, GPS_Position *GPS, bool TimeChange, bool GPSchange)
{ if(TimeChange) { LCD_UpdateTime(Time, GPS); LCD_UpdateBattery(); }
  if(GPSchange) LCD_UpdateRelayList(); }

// ------------------------------------------------------------------------------------------------------------------------

#endif

// ========================================================================================================================
