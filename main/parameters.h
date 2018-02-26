#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include <stdio.h>
#include <string.h>

#include "hal.h"

#ifdef WITH_ESP32
#include "nvs.h"
#endif

#ifdef WITH_STM32
#include "stm32f10x_flash.h"
#include "flashsize.h"
#endif

#include "nmea.h"
#include "format.h"

// Parameters stored in Flash
class FlashParameters
{ public:
   union
   { uint32_t  AcftID;       // identification: Private:AcftType:AddrType:Address - must be different for every tracker
     struct
     { uint32_t Address:24;  // address (ID)
       uint8_t  AddrType:2;
       uint8_t  AcftType:4;
       bool      NoTrack:1;
       bool      Stealth:1;
     } ;
   } ;

    int16_t  RFchipFreqCorr; // [10Hz] frequency correction for crystal frequency offset
    int8_t   RFchipTxPower;  // [dBm] highest bit set => HW module (up to +20dBm Tx power)
    int8_t   RFchipTempCorr; // [degC] correction to the temperature measured in the RF chip

   uint32_t  CONbaud;        // [bps] Console baud rate

    int16_t  PressCorr;      // [0.25Pa] pressure correction for the baro
   union
   { uint8_t Flags;
     struct
     { bool SaveToFlash:1;   // Save parameters from the config file to Flash
       bool       hasBT:1;   // has BT interface on the console
       bool       BT_ON:1;   // BT on after power up
     } ;
   } ;                       //
    int8_t  TimeCorr;        // [sec] it appears for ArduPilot you need to correct time by 3 seconds

   int16_t  GeoidSepar;      // [0.1m] Geoid-Separation, apparently ArduPilot MAVlink does not give this value (although present in the format)
  uint16_t  SoftPPSdelay;    // [ms]

   static const uint8_t InfoParmLen = 16; // [char] max. size of an infp-parameter
   static const uint8_t InfoParmNum =  6; // [int]  number of info-parameters
         char *InfoParmValue(uint8_t Idx)       { return Idx<InfoParmNum ? Pilot + Idx*InfoParmLen:0; }
   const char *InfoParmName(uint8_t Idx) const { static const char *Name[InfoParmNum] = { "Pilot", "Manuf", "Model", "Reg", "Base", "ICE" } ;
                                                  return Idx<InfoParmNum ? Name[Idx]:0; }
     char   Pilot[InfoParmLen];
     char   Manuf[InfoParmLen];
     char   Model[InfoParmLen];
     char     Reg[InfoParmLen];
     char    Base[InfoParmLen];
     char     ICE[InfoParmLen];

   // char BTname[8];
   // char  BTpin[4];
   // char Pilot[16];
   // char Copilot[16]
   // char Category[16]

   // static const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);

  public:
   int8_t getTxPower(void) const { int8_t Pwr=RFchipTxPower&0x7F; if(Pwr&0x40) Pwr|=0x80; return Pwr; }
   void   setTxPower(int8_t Pwr) { RFchipTxPower = (RFchipTxPower&0x80) | (Pwr&0x7F); }

   void   setTxTypeHW(void)       {        RFchipTxPower|=0x80; }
   void   clrTxTypeHW(void)       {        RFchipTxPower&=0x7F; }
   uint8_t isTxTypeHW(void) const { return RFchipTxPower& 0x80; } // if this RFM69HW (Tx power up to +20dBm) ?

   static const uint32_t CheckInit = 0x89ABCDEF;

 public:
  // void setDefault(void) { setDefaults(UniqueID[0] ^ UniqueID[1] ^ UniqueID[2]); }

  void setDefault(void)
  { setDefault(getUniqueAddress()); }

  void setDefault(uint32_t UniqueAddr)
  { AcftID = 0x07000000 | (UniqueAddr&0x00FFFFFF);
    RFchipFreqCorr =         0; // [10Hz]
#ifdef WITH_RFM69W
    RFchipTxPower  =        13; // [dBm] for RFM69W
#else
    RFchipTxPower  = 0x80 | 14; // [dBm] for RFM69HW
#endif
    RFchipTempCorr =         0; // [degC]
    CONbaud        =    115200; // [bps]
    PressCorr      =         0; // [0.25Pa]
    TimeCorr       =         0; // [sec]
    GeoidSepar     =       470; // [0.1m]

    Pilot[0]    = 0;
    Manuf[0]    = 0;
    Model[0]    = 0;
    Reg[0]      = 0;
    Base[0]     = 0;
    ICE[0]      = 0;
  }

#ifdef WITH_ESP32
  esp_err_t WriteToNVS(const char *Name="Parameters", const char *NameSpace="TRACKER")
  { nvs_handle Handle;
    esp_err_t Err = nvs_open(NameSpace, NVS_READWRITE, &Handle);
    if(Err!=ESP_OK) return Err;
    Err = nvs_set_blob(Handle, Name, this, sizeof(FlashParameters));
    if(Err==ESP_OK) Err = nvs_commit(Handle);
    nvs_close(Handle);
    return Err; }

  esp_err_t ReadFromNVS(const char *Name="Parameters", const char *NameSpace="TRACKER")
  { nvs_handle Handle;
    esp_err_t Err = nvs_open(NameSpace, NVS_READWRITE, &Handle);
    if(Err!=ESP_OK) return Err;
    size_t Size=0;
    Err = nvs_get_blob(Handle, Name,    0, &Size);                  // get the Size of the blob in the Flash
    if( (Err==ESP_OK) && (Size<=sizeof(FlashParameters)) )
      Err = nvs_get_blob(Handle, Name, this, &Size);                // read the Blob from the Flash
    nvs_close(Handle);
    return Err; }
#endif // WITH_ESP32

#ifdef WITH_STM32
  uint32_t static CheckSum(const uint32_t *Word, uint32_t Words)                      // calculate check-sum of pointed data
  { uint32_t Check=CheckInit;
    for(uint32_t Idx=0; Idx<Words; Idx++)
    { Check+=Word[Idx]; }
    return Check; }

  uint32_t CheckSum(void) const                                                       // calc. check-sum of this class data
  { return CheckSum((uint32_t *)this, sizeof(FlashParameters)/sizeof(uint32_t) ); }

  static uint32_t *DefaultFlashAddr(void) { return FlashStart+((uint32_t)(getFlashSizeKB()-1)<<8); }

  int8_t ReadFromFlash(uint32_t *Addr=0)                                               // read parameters from Flash
  { if(Addr==0) Addr = DefaultFlashAddr();
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    uint32_t Check=CheckSum(Addr, Words);                                              // check-sum of Flash data
    if(Check!=Addr[Words]) return -1;                                                  // agree with the check-sum in Flash ?
    uint32_t *Dst = (uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)                                              // read data from Flash
    { Dst[Idx] = Addr[Idx]; }
    return 1; }                                                                        // return: correct

  int8_t CompareToFlash(uint32_t *Addr=0)
  { if(Addr==0) Addr = DefaultFlashAddr();                                             // address in the Flash
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    uint32_t Check=CheckSum(Addr, Words);                                              // check-sum of Flash data
    if(Check!=Addr[Words]) return 0;                                                   // agree with the check-sum in Flash ?
    uint32_t *Dst = (uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)                                              // read data from Flash
    { if(Dst[Idx]!=Addr[Idx]) return 0; }
    return 1; }                                                                        // return: correct

  int8_t WriteToFlash(uint32_t *Addr=0) const                                          // write parameters to Flash
  { if(Addr==0) Addr = DefaultFlashAddr();
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    FLASH_Unlock();                                                                    // unlock Flash
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage((uint32_t)Addr);                                                   // erase Flash page
    uint32_t *Data=(uint32_t *)this;                                                   // take data of this object
    for(uint32_t Idx=0; Idx<Words; Idx++)                                              // word by word
    { FLASH_ProgramWord((uint32_t)Addr, Data[Idx]); Addr++; } // !=FLASH_COMPLETE ?    // write to Flash
    FLASH_ProgramWord((uint32_t)Addr, CheckSum(Data, Words) );                         // write the check-sum
    FLASH_Lock();                                                                      // re-lock Flash
    if(CheckSum(Addr, Words)!=Addr[Words]) return -1;                                  // verify check-sum in Flash
    return 0; }
#endif // WITH_STM32

  uint8_t Print(char *Line)       // print parameters on a single line, suitable for console output
  { uint8_t Len=0;
    Line[Len++]=HexDigit(AcftType); Line[Len++]=':';
    Line[Len++]=HexDigit(AddrType); Line[Len++]=':';
    Len+=Format_Hex(Line+Len, Address, 6);
    uint32_t DefaultAddr=getUniqueAddress();
    if(Address!=DefaultAddr)
    { Line[Len++]='/'; Len+=Format_Hex(Line+Len, DefaultAddr, 6); }
#ifdef WITH_RFM69
    Len+=Format_String(Line+Len, " RFM69");
    if(isTxTypeHW()) Line[Len++]='H';
    Line[Len++]='W';
#endif
#ifdef WITH_RFM95
    Len+=Format_String(Line+Len, " RFM95");
#endif
    Line[Len++]='/';
    Len+=Format_SignDec(Line+Len, (int16_t)getTxPower());
    Len+=Format_String(Line+Len, "dBm");
    Line[Len++]=' '; Len+=Format_SignDec(Line+Len, (int32_t)RFchipFreqCorr/10, 2, 1); Len+=Format_String(Line+Len, "kHz");
    Len+=Format_String(Line+Len, " CON:");
    Len+=Format_UnsDec(Line+Len, CONbaud);
    Len+=Format_String(Line+Len, "bps\n");
    Line[Len]=0;
    return Len; }

  int ReadPOGNS(NMEA_RxMsg &NMEA)
  { int Count=0;
    for(uint8_t Idx=0; ; Idx++)
    { char *Parm = (char *)NMEA.ParmPtr(Idx); if(Parm==0) break;
      if(ReadLine(Parm)) Count++; }
    return Count; }
/*
  int ReadPOGNS(NMEA_RxMsg &NMEA)
  { const char *Parm; int8_t Val;
    Parm = (const char *)NMEA.ParmPtr(0);                                  // [0..15] aircraft-type: 1=glider, 2=tow plane, 3=helicopter, ...
    if(Parm)
    { Val=Read_Hex1(Parm[0]);
      if( (Val>=0) && (Val<16) ) AcftType=Val; }
    Parm = (const char *)NMEA.ParmPtr(1);                                  // [0..3] addr-type: 1=ICAO, 2=FLARM, 3=OGN
    if(Parm)
    { Val=Read_Hex1(Parm[0]);
      if( (Val>=0) && (Val<4) ) AddrType=Val; }
    Parm = (const char *)NMEA.ParmPtr(2);                                  // [HHHHHH] Address (ID): 6 hex digits, 24-bit
    uint32_t Addr;
    int8_t Len=Read_Hex(Addr, Parm);
    if( (Len==6) && (Addr<0x01000000) ) Address=Addr;
    Parm = (const char *)NMEA.ParmPtr(3);                                  // [0..1] RFM69HW (up to +20dBm) or W (up to +13dBm)
    if(Parm)
    { Val=Read_Dec1(Parm[0]);
           if(Val==0) clrTxTypeHW();
      else if(Val==1) setTxTypeHW(); }
    Parm = (const char *)NMEA.ParmPtr(4);                                  // [dBm] Tx power
    int32_t TxPwr;
    Len=Read_SignDec(TxPwr, Parm);
    if( (Len>0) && (TxPwr>=(-10)) && (TxPwr<=20) ) setTxPower(TxPwr);
    Parm = (const char *)NMEA.ParmPtr(5);                                  // [kHz] Tx/Rx frequency correction
    int32_t FreqCorr;
    Len=Read_Float1(FreqCorr, Parm);
    if( (Len>0) && (FreqCorr>=(-1000)) && (FreqCorr<=1000) ) RFchipFreqCorr = 10*FreqCorr;
    Parm = (const char *)NMEA.ParmPtr(6);                                  // [bps] Console baud rate
    uint32_t BaudRate;
    Len=Read_UnsDec(BaudRate, Parm);
    if( (Len>0) && (BaudRate<=230400) ) CONbaud = BaudRate;
    return 0; }
*/
  static bool isStringChar (char ch)        // characters accepted as part of the string values
  { if( (ch>='0') && (ch<='9') ) return 1;  // numbers
    if( (ch>='A') && (ch<='Z') ) return 1;  // uppercase letters
    if( (ch>='a') && (ch<='z') ) return 1;  // lowercase letters
    if(strchr(".@-+_/#", ch)) return 1;     // any of the listed special characters
    return 0; }

  static int8_t Read_String(char *Value, const char *Inp, uint8_t MaxLen)
  { const char *Val = SkipBlanks(Inp);
    uint8_t Idx;
    for(Idx=0; Idx<MaxLen; Idx++)
    { char ch=(*Val); if(ch==0) break;
      if(!isStringChar(ch)) break;
      Value[Idx] = ch;
      Val++; }
    for( ; Idx<MaxLen; Idx++)
      Value[Idx]=0;
    return Val-Inp; }

  static const char *SkipBlanks(const char *Inp)
  { for( ; ; )
    { char ch=(*Inp); if(ch==0) break;
      if(ch>' ') break;
      Inp++; }
    return Inp; }

  bool ReadParam(const char *Name, const char *Value)                           // interprete "Name = Value" line
  { if(strcmp(Name, "Address")==0)
    { uint32_t Addr=0; if(Read_Int(Addr, Value)<=0) return 0;
      Address=Addr; return 1; }
    if(strcmp(Name, "AddrType")==0)
    { uint32_t Type=0; if(Read_Int(Type, Value)<=0) return 0;
      AddrType=Type; return 1; }
    if(strcmp(Name, "AcftType")==0)
    { uint32_t Type=0; if(Read_Int(Type, Value)<=0) return 0;
      AcftType=Type; return 1; }
    if(strcmp(Name, "Console")==0)
    { uint32_t Baud=0; if(Read_Int(Baud, Value)<=0) return 0;
      CONbaud=Baud; return 1; }
    if(strcmp(Name, "TxPower")==0)
    { int32_t TxPower=0; if(Read_Int(TxPower, Value)<=0) return 0;
      setTxPower(TxPower); return 1; }
    if(strcmp(Name, "FreqCorr")==0)
    { int32_t Corr=0; if(Read_Float1(Corr, Value)<=0) return 0;
      RFchipFreqCorr=10*Corr; return 1; }
    if(strcmp(Name, "PressCorr")==0)
    { int32_t Corr=0; if(Read_Float1(Corr, Value)<=0) return 0;
      PressCorr=4*Corr/10; return 1; }
    if(strcmp(Name, "TimeCorr")==0)
    { int32_t Corr=0; if(Read_Int(Corr, Value)<=0) return 0;
      TimeCorr=Corr; return 1; }
    if(strcmp(Name, "GeoidSepar")==0)
    { return Read_Float1(GeoidSepar, Value)<=0; }
    for(uint8_t Idx=0; Idx<InfoParmNum; Idx++)
    { if(strcmp(Name, InfoParmName(Idx))==0)
        return Read_String(InfoParmValue(Idx), Value, 16)<=0; }
    return 0; }

  bool ReadLine(char *Line)                                                     // read a parameter line
  { char *Name = (char *)SkipBlanks(Line); if((*Name)==0) return 0;             // skip blanks and get to parameter name
    Line = Name;
    for( ; ; )                                                                  // step through the characters
    { char ch=(*Line);                                                          // next character
      if(ch<=' ') break;                                                        // break at blank
      if(ch=='=') break;                                                        // or equal sign
      Line++; }
    char *NameEnd=Line;                                                         // remember where the parameter name ends
    Line = (char *)SkipBlanks(Line); if((*Line)!='=') return 0;                 // next should be the equal sign
    char *Value = (char *)SkipBlanks(Line+1); if((*Value)<=' ') return 0;
    *NameEnd=0;                                                                 // put NULL character just after the parameter name
    return ReadParam(Name, Value); }

  int ReadFile(FILE *File)
  { char Line[80];                                                              // line buffer
    size_t Lines=0;                                                             // count interpreted lines
    for( ; ; )                                                                  // loop over lines
    { if(fgets(Line, 80, File)==0) break;                                       // break on EOF or other trouble reading the file
      if(strchr(Line, '\n')==0) break;                                          // if no NL then break, line was too long
      if(ReadLine(Line)) Lines++; }                                             // interprete the line, count if positive
    return Lines; }                                                             // return number of interpreted lines

  int ReadFile(const char *Name = "/spiffs/TRACKER.CFG")
  { FILE *File=fopen(Name, "rt"); if(File==0) return 0;
    int Lines=ReadFile(File);
    fclose(File); return Lines; }

  int Write_Hex(char *Line, const char *Name, uint32_t Value, uint8_t Digits)
  { uint8_t Len=Format_String(Line, Name, 12, 0);
    Len+=Format_String(Line+Len, " = 0x");
    Len+=Format_Hex(Line+Len, Value, Digits);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_UnsDec(char *Line, const char *Name, uint32_t Value)
  { uint8_t Len=Format_String(Line, Name, 12, 0);
    Len+=Format_String(Line+Len, " = ");
    Len+=Format_UnsDec(Line+Len, Value);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_SignDec(char *Line, const char *Name, int32_t Value)
  { uint8_t Len=Format_String(Line, Name, 12, 0);
    Len+=Format_String(Line+Len, " = ");
    Len+=Format_SignDec(Line+Len, Value);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_Float1(char *Line, const char *Name, int32_t Value)
  { uint8_t Len=Format_String(Line, Name, 12, 0);
    Len+=Format_String(Line+Len, " = ");
    Len+=Format_SignDec(Line+Len, Value, 2, 1);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_String(char *Line, const char *Name, char *Value, uint8_t MaxLen=16)
  { uint8_t Len=Format_String(Line, Name, 12, 0);
    Len+=Format_String(Line+Len, " = ");
    Len+=Format_String(Line+Len, Value, 0, MaxLen);
    Line[Len]=0; return Len; }

  int WriteFile(FILE *File)
  { char Line[80];
    Write_Hex    (Line, "Address"   ,          Address ,       6); strcat(Line, " # [24-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Hex    (Line, "AddrType"  ,          AddrType,       1); strcat(Line, " #  [2-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Hex    (Line, "AcftType"  ,          AcftType,       1); strcat(Line, " #  [4-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_UnsDec (Line, "Console"   ,          CONbaud          ); strcat(Line, " #  [  bps]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_SignDec(Line, "TxPower"   ,          getTxPower()     ); strcat(Line, " #  [  dBm]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Float1 (Line, "FreqCorr"  , (int32_t)RFchipFreqCorr/10); strcat(Line, " #  [  kHz]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_SignDec(Line, "TempCorr"  , (int32_t)RFchipTempCorr   ); strcat(Line, " #  [ degC]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Float1 (Line, "PressCorr" , (int32_t)PressCorr*10/4   ); strcat(Line, " #  [   Pa]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_SignDec(Line, "TimeCorr"  , (int32_t)TimeCorr         ); strcat(Line, " #  [    s]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Float1 (Line, "GeoidSepar",          GeoidSepar       ); strcat(Line, " #  [    m]\n"); if(fputs(Line, File)==EOF) return EOF;
    for(uint8_t Idx=0; Idx<InfoParmNum; Idx++)
    { Write_String (Line, InfoParmName(Idx), InfoParmValue(Idx)); strcat(Line, " #  [char]\n"); if(fputs(Line, File)==EOF) return EOF; }
    return 10+InfoParmNum; }

  int WriteFile(const char *Name = "/spiffs/TRACKER.CFG")
  { FILE *File=fopen(Name, "wt"); if(File==0) return 0;
    int Lines=WriteFile(File);
    fclose(File); return Lines; }

  void Write(void (*Output)(char))
  { char Line[80];
    Write_Hex    (Line, "Address"   ,          Address ,       6); strcat(Line, " # [24-bit]\n"); Format_String(Output, Line);
    Write_Hex    (Line, "AddrType"  ,          AddrType,       1); strcat(Line, " #  [2-bit]\n"); Format_String(Output, Line);
    Write_Hex    (Line, "AcftType"  ,          AcftType,       1); strcat(Line, " #  [4-bit]\n"); Format_String(Output, Line);
    Write_UnsDec (Line, "Console"   ,          CONbaud          ); strcat(Line, " #  [  bps]\n"); Format_String(Output, Line);
    Write_SignDec(Line, "TxPower"   ,          getTxPower()     ); strcat(Line, " #  [  dBm]\n"); Format_String(Output, Line);
    Write_Float1 (Line, "FreqCorr"  , (int32_t)RFchipFreqCorr/10); strcat(Line, " #  [  kHz]\n"); Format_String(Output, Line);
    Write_SignDec(Line, "TempCorr"  , (int32_t)RFchipTempCorr   ); strcat(Line, " #  [ degC]\n"); Format_String(Output, Line);
    Write_Float1 (Line, "PressCorr" , (int32_t)PressCorr*10/4   ); strcat(Line, " #  [   Pa]\n"); Format_String(Output, Line);
    Write_SignDec(Line, "TimeCorr"  , (int32_t)TimeCorr         ); strcat(Line, " #  [    s]\n"); Format_String(Output, Line);
    Write_Float1 (Line, "GeoidSepar",          GeoidSepar       ); strcat(Line, " #  [    m]\n"); Format_String(Output, Line);
    for(uint8_t Idx=0; Idx<InfoParmNum; Idx++)
    { Write_String (Line, InfoParmName(Idx), InfoParmValue(Idx)); strcat(Line, " #  [char]\n"); Format_String(Output, Line); }
  }

} ;

#endif // __PARAMETERS_H__

