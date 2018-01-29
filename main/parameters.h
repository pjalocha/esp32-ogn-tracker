#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include <stdio.h>
#include <string.h>

#include "nvs.h"
// #include "flashsize.h"
// #include "uniqueid.h"

// #include "stm32f10x_flash.h"

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
   uint16_t  PressCorr;      // [0.25Pa] pressure correction for the baro
   union
   { uint16_t Flags;
     struct
     { bool SaveToFlash:1;   // Save parameters from the config file to Flash
       bool       hasBT:1;   // has BT interface on the console
       bool       BT_ON:1;   // BT on after power up
     } ;
   } ;                       //
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

  void setDefault(uint32_t UniqueID)
  { AcftID = 0x07000000 | (UniqueID&0x00FFFFFF);
    RFchipFreqCorr =         0; // [10Hz]
#ifdef WITH_RFM69W
    RFchipTxPower  =        13; // [dBm] for RFM69W
#else
    RFchipTxPower  = 0x80 | 14; // [dBm] for RFM69HW
#endif
    RFchipTempCorr =         0; // [degC]
    PressCorr      =         0; // [0.25Pa]
    CONbaud        =    115200; // [bps]
  }

  uint32_t static CheckSum(const uint32_t *Word, uint32_t Words)                      // calculate check-sum of pointed data
  { uint32_t Check=CheckInit;
    for(uint32_t Idx=0; Idx<Words; Words++)
    { Check+=Word[Idx]; }
    return Check; }

  uint32_t CheckSum(void) const                                                       // calc. check-sum of this class data
  { return CheckSum((uint32_t *)this, sizeof(FlashParameters)/sizeof(uint32_t) ); }

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
    Err = nvs_get_blob(Handle, Name,    0, &Size);
    if( (Err==ESP_OK) && (Size==sizeof(FlashParameters)) )
      Err = nvs_get_blob(Handle, Name, this, &Size);
    nvs_close(Handle);
    return Err; }

/*
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
*/
  uint8_t Print(char *Line)       // print parameters on a single line, suitable for console output
  { uint8_t Len=0;
    Line[Len++]=HexDigit(AcftType); Line[Len++]=':';
    Line[Len++]=HexDigit(AddrType); Line[Len++]=':';
    Len+=Format_Hex(Line+Len, Address, 6);
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

  static char *SkipBlanks(char *Inp)
  { for( ; ; )
    { char ch=(*Inp); if(ch==0) break;
      if(ch>' ') break;
      Inp++; }
    return Inp; }

  bool ReadLine(char *Line)
  { char *Name = SkipBlanks(Line); if((*Name)==0) return 0;
    Line = Name;
    for( ; ; )
    { char ch=(*Line);
      if(ch<=' ') break;
      if(ch=='=') break;
      Line++; }
    char *NameEnd=Line;
    Line=SkipBlanks(Line); if((*Line)!='=') return 0;
    char *Value = SkipBlanks(Line+1); if((*Value)<=' ') return 0;
    *NameEnd=0;
    return ReadParam(Name, Value); }

  bool ReadParam(const char *Name, const char *Value)
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
    return 0; }

  int ReadFile(FILE *File)
  { char Line[80];
    size_t Lines=0;
    for( ; ; )
    { if(fgets(Line, 80, File)==0) break;
      if(strchr(Line, '\n')==0) break;
      if(ReadLine(Line)) Lines++; }
    return Lines; }

  int ReadFile(const char *Name = "/spiffs/TRACKER.CFG")
  { FILE *File=fopen(Name, "rt"); if(File==0) return 0;
    int Lines=ReadFile(File);
    fclose(File); return Lines; }

  int Write_Hex(char *Line, const char *Name, uint32_t Value, uint8_t Digits)
  { uint8_t Len=Format_String(Line, Name);
    Len+=Format_String(Line+Len, "=0x");
    Len+=Format_Hex(Line+Len, Value, Digits);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_UnsDec(char *Line, const char *Name, uint32_t Value)
  { uint8_t Len=Format_String(Line, Name);
    Len+=Format_String(Line+Len, "=");
    Len+=Format_UnsDec(Line+Len, Value);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_SignDec(char *Line, const char *Name, int32_t Value)
  { uint8_t Len=Format_String(Line, Name);
    Len+=Format_String(Line+Len, "=");
    Len+=Format_SignDec(Line+Len, Value);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_Float1(char *Line, const char *Name, int32_t Value)
  { uint8_t Len=Format_String(Line, Name);
    Len+=Format_String(Line+Len, "=");
    Len+=Format_SignDec(Line+Len, Value, 2, 1);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int WriteFile(FILE *File)
  { char Line[80];
    Write_Hex    (Line, "Address" , Address , 6); strcat(Line, " # [24-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Hex    (Line, "AddrType", AddrType, 1); strcat(Line, " # [2-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Hex    (Line, "AcftType", AcftType, 1); strcat(Line, " # [4-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_UnsDec (Line, "Console",  CONbaud);     strcat(Line, " # [bps]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_SignDec(Line, "TxPower",  getTxPower()); strcat(Line, " # [ dBm]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Float1 (Line, "FreqCorr", (int32_t)RFchipFreqCorr/10); strcat(Line, " # [kHz]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_SignDec(Line, "TempCorr", (int32_t)RFchipTempCorr   ); strcat(Line, " # [degC]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Float1 (Line, "PressCorr",(int32_t)PressCorr*10/4   ); strcat(Line, " # [Pa]\n"); if(fputs(Line, File)==EOF) return EOF;
    return 8; }

  int WriteFile(const char *Name = "/spiffs/TRACKER.CFG")
  { FILE *File=fopen(Name, "wt"); if(File==0) return 0;
    int Lines=WriteFile(File);
    fclose(File); return Lines; }

} ;

#endif // __PARAMETERS_H__

