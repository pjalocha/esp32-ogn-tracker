#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include <stdio.h>
#include <string.h>

#if defined(WITH_STM32) || defined(WITH_ESP32)
#include "hal.h"
#endif

#include "ogn.h"

#ifdef WITH_ESP32
#include "nvs.h"
#endif

#ifdef WITH_SAMD21
#include "flashsize.h"
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
       uint8_t  AddrType:2;  // 0=RND, 1=ICAO, 2=FLR, 3=OGN
       uint8_t  AcftType:4;  // 1=glider, 2=towplane, 3=helicopter, etc.
       bool      NoTrack:1;  // unused
       bool      Stealth:1;  // unused
     } ;
   } ;

    int16_t  RFchipFreqCorr; // [0.1ppm] frequency correction for crystal frequency offset
    int8_t   RFchipTxPower;  // [dBm] highest bit set => HW module (up to +20dBm Tx power)
    int8_t   RFchipTempCorr; // [degC] correction to the temperature measured in the RF chip

   union
   { uint32_t Console;
     struct
     { uint32_t  CONbaud:24; // [bps] Console baud rate
       uint8_t   CONprot: 8; // [bit-mask] Console protocol mask: 0=minGPS, 1=allGPS, 2=Baro, 3=UBX, 4=OGN, 5=FLARM, 6=GDL90, 7=$PGAV5
     } ;
   } ;

    int16_t  PressCorr;      // [0.25Pa] pressure correction for the baro

   union
   { uint16_t Flags;
     struct
     { bool SaveToFlash:1;   // Save parameters from the config file to Flash
       bool       hasBT:1;   // has BT interface on the console
       bool       BT_ON:1;   // BT on after power up
       bool manGeoidSepar:1; // GeoidSepar is manually configured as the GPS or MAVlink are not able to deliver it
       bool     Encrypt:1;   // encrypt the position
       uint8_t  NavMode:3;   // GPS navigation mode/model
       uint8_t  NavRate:2;   // [Hz]
       uint8_t  Verbose:2;   //
        int8_t TimeCorr:4;   // [sec] it appears for ArduPilot you need to correct time by 3 seconds
     } ;
   } ;                      //

   int16_t  GeoidSepar;      // [0.1m] Geoid-Separation, apparently ArduPilot MAVlink does not give this value (although present in the format)
                             //  or it could be a problem of some GPSes
  uint8_t  PPSdelay;         // [ms] delay between the PPS and the data burst starts on the GPS UART (used when PPS failed or is not there)
  uint8_t  FreqPlan;         // force given frequency hopping plan

   static const uint8_t InfoParmLen = 16; // [char] max. size of an infp-parameter
   static const uint8_t InfoParmNum = 14; // [int]  number of info-parameters
         char *InfoParmValue(uint8_t Idx)      { return Idx<InfoParmNum ? Pilot + Idx*InfoParmLen:0; }
      uint8_t  InfoParmValueLen(uint8_t Idx)   { return strlen(InfoParmValue(Idx)); }
//    const char *InfoParmName(uint8_t Idx) const { static const char *Name[InfoParmNum] =
//                                                { "Pilot", "Manuf", "Model", "Type", "SN", "Reg", "ID", "Class",
//                                                  "Task" , "Base" , "ICE"  , "PilotID" } ;
//                                                   return Idx<InfoParmNum ? Name[Idx]:0; }
     char   Pilot[InfoParmLen];                // Pilot name
     char   Manuf[InfoParmLen];                // Manufacturer
     char   Model[InfoParmLen];                // Model
     char    Type[InfoParmLen];                // Type
     char      SN[InfoParmLen];                // Serial Number
     char     Reg[InfoParmLen];                // Registration
     char      ID[InfoParmLen];                // Competition ID
     char   Class[InfoParmLen];                // Competition class
     char    Task[InfoParmLen];                // Competition task
     char    Base[InfoParmLen];                // Base airfield
     char     ICE[InfoParmLen];                // In Case of Emergency
     char PilotID[InfoParmLen];                // Pilot ID based on his BT or WiFi MAC
     char    Hard[InfoParmLen];                // Hardware
     char    Soft[InfoParmLen];                // Software

   // char Copilot[16]
   // char Category[16]

#ifdef WITH_BT_SPP
   char BTname[16];
   // char  BTpin[16];
#endif

#ifdef WITH_WIFI
   static const uint8_t WIFInameLen = 32;
   static const uint8_t WIFIpassLen = 64;
   static const uint8_t WIFIsets    = 10;
   char *getWIFIname(uint8_t Idx) { return Idx<WIFIsets ? WIFIname[Idx]:0; }
   char *getWIFIpass(uint8_t Idx) { return Idx<WIFIsets ? WIFIpass[Idx]:0; }

   char WIFIname[WIFIsets][32];
   char WIFIpass[WIFIsets][64];
#endif

#ifdef WITH_ENCRYPT
  uint32_t EncryptKey[4];    // encryption key
#endif

  uint32_t CheckSum;

#ifdef WITH_WIFI
   const char *getWIFIpass(const char *NetName) const
   { for(uint8_t Idx=0; Idx<WIFIsets; Idx++)
     { if(strcmp(NetName, WIFIname[Idx])==0) return WIFIpass[Idx]; }
     return 0; }
#endif

   // static const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);

  public:
   int8_t getTxPower(void) const { int8_t Pwr=RFchipTxPower&0x7F; if(Pwr&0x40) Pwr|=0x80; return Pwr; }
   void   setTxPower(int8_t Pwr) { RFchipTxPower = (RFchipTxPower&0x80) | (Pwr&0x7F); }

   void   setTxTypeHW(void)       {        RFchipTxPower|=0x80; }
   void   clrTxTypeHW(void)       {        RFchipTxPower&=0x7F; }
   bool    isTxTypeHW(void) const { return RFchipTxPower& 0x80; } // if this RFM69HW (Tx power up to +20dBm) ?

   static const uint32_t CheckInit = 0x89ABCDEF;

   uint32_t static calcCheckSum(volatile uint32_t *Word, uint32_t Words)                      // calculate check-sum of pointed data
   { uint32_t Check=CheckInit;
     for(uint32_t Idx=0; Idx<Words; Idx++)
     { Check += Word[Idx]; }
     return Check; }

   uint32_t calcCheckSum(void) const                                                       // calc. check-sum of this class data
   { return calcCheckSum((volatile uint32_t *)this, sizeof(FlashParameters)/sizeof(uint32_t) ); }

   void setCheckSum(void) { CheckSum -= calcCheckSum(); }
   bool goodCheckSum(void) const { return calcCheckSum()==0; }

   uint8_t getAprsCall(char *Call)
   { const char *AddrTypeName[4] = { "RND", "ICA", "FLR", "OGN" };
     memcpy(Call, AddrTypeName[AddrType], 3);
     Format_Hex(Call+3, (uint8_t)(Address>>16));
     Format_Hex(Call+5, (uint16_t)Address);
     Call[9]=0; return 9; }

 public:

  void setDefault(void)
  { setDefault(getUniqueAddress()); }

  void setDefault(uint32_t UniqueAddr)
  { AcftID = ((uint32_t)DEFAULT_AcftType<<26) | 0x03000000 | (UniqueAddr&0x00FFFFFF);
    RFchipFreqCorr =         0; // [0.1ppm]
#ifdef WITH_RFM69W
    RFchipTxPower  =        13; // [dBm] for RFM69W
#else
    RFchipTxPower  = 0x80 | 14; // [dBm] for RFM69HW
#endif

    Flags          =         0;
#ifdef WITH_GPS_UBX
    NavMode        =         6; // Avionic mode 1g for UBX
#endif
#ifdef WITH_GPS_MTK
    NavMode        =         2; // Avionic mode for MTK
#endif
    NavRate        =         1; // [Hz]
    Verbose        =         1;

    RFchipTempCorr =         0; // [degC]
    CONbaud        =    DEFAULT_CONbaud; // [bps]
    CONprot        =      0xFF;
    PressCorr      =         0; // [0.25Pa]
    TimeCorr       =         0; // [sec]
    GeoidSepar     =    10*DEFAULT_GeoidSepar; // [0.1m]

    FreqPlan       =    DEFAULT_FreqPlan; // [0..5]
    PPSdelay       =    DEFAULT_PPSdelay; // [ms]
#ifdef WITH_ENCRYPT
    for(uint8_t Idx=0; Idx<4; Idx++) EncryptKey[Idx]=0;
#endif
    for(uint8_t Idx=0; Idx<InfoParmNum; Idx++)
      InfoParmValue(Idx)[0] = 0;
#ifdef WITH_BT_SPP
   getAprsCall(BTname);
// #ifdef WITH_TBEAM
//    strcpy(BTname, "TBEAM-OGN");
// #else
//    strcpy(BTname, "ESP32-OGN");
// #endif
   // strcpy(BTpin, "1234");
#endif
#ifdef WITH_WIFI
    for(uint8_t Idx=0; Idx<WIFIsets; Idx++)
    { WIFIname[Idx][0] = 0;
      WIFIpass[Idx][0] = 0; }
#endif
  }

// void WriteHeader(OGN_Packet &Packet)
// { Packet.HeaderWord=0;
//   Packet.Header.Address    = Parameters.Address;    // set address
//   Packet.Header.AddrType   = Parameters.AddrType;   // address-type
//   Packet.Header.Other=1;
//   Packet.calcAddrParity(); }                        // parity of (part of) the header

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

#ifdef WITH_SAMD21
  static uint32_t *DefaultFlashAddr(void) { return FlashStart+((uint32_t)(getFlashSizeKB()-1)<<8); } // the last KB

  int8_t ReadFromFlash(volatile uint32_t *Addr=0)                                      // read parameters from Flash
  { if(Addr==0) Addr = DefaultFlashAddr();                                             // default address: the last KB
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    if(calcCheckSum(Addr, Words)!=0) return -1;                                        // agree with the check-sum in Flash ?
    uint32_t *Dst = (uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)                                              // read data from Flash
    { Dst[Idx] = Addr[Idx]; }
    return 1; }                                                                        // return: correct

  bool CompareToFlash(volatile uint32_t *Addr=0)                                       // are the parameters identical to those in the flash ?
  { if(Addr==0) Addr = DefaultFlashAddr();                                             // address in the Flash
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    if(calcCheckSum(Addr, Words)!=0) return 0;                                         // agree with the check-sum in Flash ?
    uint32_t *Dst = (uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)                                              // read data from Flash
    { if(Dst[Idx] != Addr[Idx]) return 0; }
    return 1; }                                                                        // return: correct

  void ErasePage4x64(volatile uint32_t *Addr) const                                    // erase a 4x64 = 256-byte page
  { NVMCTRL->ADDR.reg = ((uint32_t)Addr)>>1;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    while (!NVMCTRL->INTFLAG.bit.READY) { }
  }

  void EraseSize(volatile uint32_t *Addr, uint32_t Size=1024) const                     // erase multiple pages for given size
  { for( ; ; )
    { if(Size==0) break;
      ErasePage4x64(Addr);                                                             //
      if(Size<256) break;
      Addr+=64; Size-=256; }
  }

  int WritePage64(volatile uint32_t *Addr, const uint32_t *Data, uint32_t Words) const
  { // NVMCTRL->ADDR.reg = ((uint32_t)Addr)>>1;
    NVMCTRL->CTRLB.bit.MANW = 1;                                          // disable Automatic Page Write
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC; // execute Page Buffer Clear
    while (NVMCTRL->INTFLAG.bit.READY == 0) { }
    uint32_t Idx=0;
    for(Idx=0; (Idx<16) && (Idx<Words); Idx++)                             // copy Data
    { Addr[Idx] = Data[Idx]; }
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;  // execute Write Page
    while (NVMCTRL->INTFLAG.bit.READY == 0) { }
    return Idx; }

  int WriteSize(volatile uint32_t *Addr, const uint32_t *Data, uint32_t Words)
  { for( ; ; )
    { int Len = WritePage64(Addr, Data, Words);
      Addr+=Len; Data+=Len; Words-=Len; if(Words==0) break; }
    return 0; }

  int8_t WriteToFlash(volatile uint32_t *Addr=0)                          // write parameters to Flash
  { if(Addr==0) Addr = DefaultFlashAddr();
    setCheckSum();
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    EraseSize(Addr, Words);
    WriteSize(Addr, (const uint32_t *)this, Words);
    if(calcCheckSum(Addr, Words)!=0) return -1;                                  // verify check-sum in Flash
    return 0; }
#endif // WITH_SAMD21

#ifdef WITH_STM32
/*
  uint32_t static CheckSum(const uint32_t *Word, uint32_t Words)                      // calculate check-sum of pointed data
  { uint32_t Check=CheckInit;
    for(uint32_t Idx=0; Idx<Words; Idx++)
    { Check+=Word[Idx]; }
    return Check; }

  uint32_t CheckSum(void) const                                                       // calc. check-sum of this class data
  { return CheckSum((uint32_t *)this, sizeof(FlashParameters)/sizeof(uint32_t) ); }
*/
  static uint32_t *DefaultFlashAddr(void) { return FlashStart+((uint32_t)(getFlashSizeKB()-1)<<8); }

  int8_t ReadFromFlash(volatile uint32_t *Addr=0)                                      // read parameters from Flash
  { if(Addr==0) Addr = DefaultFlashAddr();                                             // default address: the last KB
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    if(calcCheckSum(Addr, Words)!=0) return -1;                                        // agree with the check-sum in Flash ?
    uint32_t *Dst = (uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)                                              // read data from Flash
    { Dst[Idx] = Addr[Idx]; }
    return 1; }                                                                        // return: correct

  bool CompareToFlash(volatile uint32_t *Addr=0)                                       // are the parameters identical to those in the flash ?
  { if(Addr==0) Addr = DefaultFlashAddr();                                             // address in the Flash
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    if(calcCheckSum(Addr, Words)!=0) return 0;                                         // agree with the check-sum in Flash ?
    uint32_t *Dst = (uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)                                              // read data from Flash
    { if(Dst[Idx] != Addr[Idx]) return 0; }
    return 1; }                                                                        // return: correct
/*
  int8_t ReadFromFlash(uint32_t *Addr=0)                                               // read parameters from Flash
  { if(Addr==0) Addr = DefaultFlashAddr();
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    uint32_t Check=CheckSum(Addr, Words);                                              // check-sum of Flash data
    if(Check!=Addr[Words]) return -1;                                                  // agree with the check-sum in Flash ?
    uint32_t *Dst = (uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)                                              // read data from Flash
    { Dst[Idx] = Addr[Idx]; }
    return 1; }                                                                        // return: correct

  bool CompareToFlash(uint32_t *Addr=0)
  { if(Addr==0) Addr = DefaultFlashAddr();                                             // address in the Flash
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    uint32_t Check=CheckSum(Addr, Words);                                              // check-sum of Flash data
    if(Check!=Addr[Words]) return 0;                                                   // agree with the check-sum in Flash ?
    uint32_t *Dst = (uint32_t *)this;
    for(uint32_t Idx=0; Idx<Words; Idx++)                                              // read data from Flash
    { if(Dst[Idx]!=Addr[Idx]) return 0; }
    return 1; }                                                                        // return: correct
*/

  int8_t WriteToFlash(volatile uint32_t *Addr=0)                                       // write parameters to Flash
  { if(Addr==0) Addr = DefaultFlashAddr();
    setCheckSum();
    const uint32_t Words=sizeof(FlashParameters)/sizeof(uint32_t);
    FLASH_Unlock();                                                                    // unlock Flash
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage((uint32_t)Addr);                                                   // erase Flash page
    uint32_t *Data=(uint32_t *)this;                                                   // take data of this object
    for(uint32_t Idx=0; Idx<Words; Idx++)                                              // word by word
    { FLASH_ProgramWord((uint32_t)Addr, Data[Idx]); Addr++; } // !=FLASH_COMPLETE ?    // write to Flash
    // FLASH_ProgramWord((uint32_t)Addr, CheckSum(Data, Words) );                         // write the check-sum
    FLASH_Lock();                                                                      // re-lock Flash
    if(calcCheckSum(Addr, Words)!=0) return -1;                           // verify check-sum in Flash
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
#ifdef WITH_SX1272
    Len+=Format_String(Line+Len, " SX1272");
#endif
    Line[Len++]='/';
    Len+=Format_SignDec(Line+Len, (int16_t)getTxPower());
    Len+=Format_String(Line+Len, "dBm");
    Line[Len++]=' '; Len+=Format_SignDec(Line+Len, (int32_t)RFchipFreqCorr, 2, 1); Len+=Format_String(Line+Len, "ppm");
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
    if(strcmp(Name, "CONbaud")==0)
    { uint32_t Baud=0; if(Read_Int(Baud, Value)<=0) return 0;
      CONbaud=Baud; return 1; }
    if(strcmp(Name, "CONprot")==0)
    { uint32_t Prot=0; if(Read_Int(Prot, Value)<=0) return 0;
      CONprot=Prot; return 1; }
    if(strcmp(Name, "TxHW")==0)
    { int32_t HW=1; if(Read_Int(HW, Value)<=0) return 0;
      if(HW) setTxTypeHW(); else clrTxTypeHW(); }
    if(strcmp(Name, "TxPower")==0)
    { int32_t TxPower=0; if(Read_Int(TxPower, Value)<=0) return 0;
      setTxPower(TxPower); return 1; }
    if(strcmp(Name, "PPSdelay")==0)
    { uint32_t Delay=0; if(Read_Int(Delay, Value)<=0) return 0;
      if(Delay>0xFF) Delay=0xFF; PPSdelay=Delay; return 1; }
    if(strcmp(Name, "FreqPlan")==0)
    { uint32_t Plan=0; if(Read_Int(Plan, Value)<=0) return 0;
      if(Plan>5) Plan=5; FreqPlan=Plan; return 1; }
    if(strcmp(Name, "FreqCorr")==0)
    { int32_t Corr=0; if(Read_Float1(Corr, Value)<=0) return 0;
      RFchipFreqCorr=Corr; return 1; }
    if(strcmp(Name, "PressCorr")==0)
    { int32_t Corr=0; if(Read_Float1(Corr, Value)<=0) return 0;
      PressCorr=4*Corr/10; return 1; }
    if(strcmp(Name, "TimeCorr")==0)
    { int32_t Corr=0; if(Read_Int(Corr, Value)<=0) return 0;
      TimeCorr=Corr; return 1; }
    if(strcmp(Name, "GeoidSepar")==0)
    { return Read_Float1(GeoidSepar, Value)<=0; }
    if(strcmp(Name, "manGeoidSepar")==0)
    { int32_t Man=0; if(Read_Int(Man, Value)<=0) return 0;
      manGeoidSepar=Man; return 1; }
    if(strcmp(Name, "NavMode")==0)
    { int32_t Mode=0; if(Read_Int(Mode, Value)<=0) return 0;
      NavMode=Mode; return 1; }
    if(strcmp(Name, "NavRate")==0)
    { int32_t Mode=0; if(Read_Int(Mode, Value)<=0) return 0;
      if(Mode<1) Mode=1; NavRate=Mode; return 1; }
    if(strcmp(Name, "Verbose")==0)
    { int32_t Mode=0; if(Read_Int(Mode, Value)<=0) return 0;
      Verbose=Mode; return 1; }
#ifdef WITH_ENCRYPT
    if(strcmp(Name, "Encrypt")==0)
    { int32_t Encr=0; if(Read_Int(Encr, Value)<=0) return 0;
      Encrypt=Encr; return 1; }
    if(strcmp(Name, "EncryptKey")==0)
    { for( uint8_t Idx=0; Idx<4; Idx++)
      { uint32_t Key;
        uint8_t Len=Read_Hex(Key, Value);
        if(Len!=8) break;
        EncryptKey[Idx]=Key;
        Value+=Len;
        if((*Value)!=':') break;
        Value++; }
      return 1; }
#endif
#ifdef WITH_BT_PWR
    if(strcmp(Name, "Bluetooth")==0)
    { int32_t bton=0; if(Read_Int(bton, Value)<=0) return 0;
      // if (bton==2) //WAR: disable usart1 in order to be able to configure BT over 2nd USB
      // { USART1_Disable();
      //   bton=1; }
      BT_ON=bton; return 1; }
#endif
    for(uint8_t Idx=0; Idx<InfoParmNum; Idx++)
    { if(strcmp(Name, OGN_Packet::InfoParmName(Idx))==0)
        return Read_String(InfoParmValue(Idx), Value, 16)<=0; }
#ifdef WITH_BT_SPP
    if(strcmp(Name, "BTname")==0) return Read_String(BTname, Value, 16)<=0;
#endif
#ifdef WITH_WIFI
    if(strcmp(Name, "WIFIname")==0) return Read_String(WIFIname[0], Value, WIFInameLen)<=0;
    if(strcmp(Name, "WIFIpass")==0) return Read_String(WIFIpass[0], Value, WIFIpassLen)<=0;
    if( (memcmp(Name, "WIFIname", 8)==0) && (strlen(Name)==9) )
    { int Idx=Name[8]-'0'; if( (Idx>=0) && (Idx<WIFIsets) ) return Read_String(WIFIname[Idx], Value, WIFInameLen)<=0; }
    if( (memcmp(Name, "WIFIpass", 8)==0) && (strlen(Name)==9) )
    { int Idx=Name[8]-'0'; if( (Idx>=0) && (Idx<WIFIsets) ) return Read_String(WIFIpass[Idx], Value, WIFIpassLen)<=0; }
#endif
    if(strcmp(Name, "SaveToFlash")==0)
    { int32_t Save=0; if(Read_Int(Save, Value)<=0) return 0;
      SaveToFlash=Save; return 1; }
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

  int ReadFromFile(FILE *File)
  { char Line[80];                                                              // line buffer
    size_t Lines=0;                                                             // count interpreted lines
    for( ; ; )                                                                  // loop over lines
    { if(fgets(Line, 80, File)==0) break;                                       // break on EOF or other trouble reading the file
      if(strchr(Line, '\n')==0) break;                                          // if no NL then break, line was too long
      if(ReadLine(Line)) Lines++; }                                             // interprete the line, count if positive
    return Lines; }                                                             // return number of interpreted lines

  int ReadFromFile(const char *Name = "/spiffs/TRACKER.CFG")
  { FILE *File=fopen(Name, "rt"); if(File==0) return 0;
    int Lines=ReadFromFile(File);
    fclose(File); return Lines; }

  int Write_Hex(char *Line, const char *Name, uint32_t Value, uint8_t Digits)
  { uint8_t Len=Format_String(Line, Name, 14, 0);
    Len+=Format_String(Line+Len, " = 0x");
    Len+=Format_Hex(Line+Len, Value, Digits);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_UnsDec(char *Line, const char *Name, uint32_t Value)
  { uint8_t Len=Format_String(Line, Name, 14, 0);
    Len+=Format_String(Line+Len, " = ");
    Len+=Format_UnsDec(Line+Len, Value);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_SignDec(char *Line, const char *Name, int32_t Value)
  { uint8_t Len=Format_String(Line, Name, 14, 0);
    Len+=Format_String(Line+Len, " = ");
    Len+=Format_SignDec(Line+Len, Value);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_Float1(char *Line, const char *Name, int32_t Value)
  { uint8_t Len=Format_String(Line, Name, 14, 0);
    Len+=Format_String(Line+Len, " = ");
    Len+=Format_SignDec(Line+Len, Value, 2, 1);
    Len+=Format_String(Line+Len, ";");
    Line[Len]=0; return Len; }

  int Write_String(char *Line, const char *Name, char *Value, uint8_t MaxLen=16)
  { uint8_t Len=Format_String(Line, Name, 14, 0);
    Len+=Format_String(Line+Len, " = ");
    Len+=Format_String(Line+Len, Value, 0, MaxLen);
    Line[Len]=0; return Len; }

  int WriteToFile(FILE *File)
  { char Line[80];
    Write_Hex    (Line, "Address"   ,          Address ,       6); strcat(Line, " # [24-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Hex    (Line, "AddrType"  ,          AddrType,       1); strcat(Line, " #  [2-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Hex    (Line, "AcftType"  ,          AcftType,       1); strcat(Line, " #  [4-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_UnsDec (Line, "CONbaud"   ,          CONbaud          ); strcat(Line, " #  [  bps]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Hex    (Line, "CONprot"   ,          CONprot,        1); strcat(Line, " #  [ mask]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_SignDec(Line, "TxPower"   ,          getTxPower()     ); strcat(Line, " #  [  dBm]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_UnsDec (Line, "TxHW"      ,(uint32_t)isTxTypeHW()     ); strcat(Line, " #  [ bool]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_UnsDec (Line, "FreqPlan"  ,(uint32_t)FreqPlan         ); strcat(Line, " #  [ 0..5]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Float1 (Line, "FreqCorr"  , (int32_t)RFchipFreqCorr   ); strcat(Line, " #  [  ppm]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_SignDec(Line, "TempCorr"  , (int32_t)RFchipTempCorr   ); strcat(Line, " #  [ degC]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Float1 (Line, "PressCorr" , (int32_t)PressCorr*10/4   ); strcat(Line, " #  [   Pa]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_SignDec(Line, "TimeCorr"  , (int32_t)TimeCorr         ); strcat(Line, " #  [    s]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_Float1 (Line, "GeoidSepar",          GeoidSepar       ); strcat(Line, " #  [    m]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_UnsDec (Line, "manGeoidSepar" ,   manGeoidSepar       ); strcat(Line, " #  [  1|0]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_UnsDec (Line, "NavMode"  ,      (uint32_t)NavMode     ); strcat(Line, " #  [ 0..7]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_UnsDec (Line, "NavRate"  ,      (uint32_t)NavRate     ); strcat(Line, " #  [  1,2]\n"); if(fputs(Line, File)==EOF) return EOF;
#ifdef WITH_ENCRYPT
    Write_UnsDec (Line, "Encrypt"   ,          Encrypt          ); strcat(Line, " #  [  1|0]\n"); if(fputs(Line, File)==EOF) return EOF;
    // Write_Hex    (Line, "EncryptKey[0]",       EncryptKey[0] , 8); strcat(Line, " # [32-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    // Write_Hex    (Line, "EncryptKey[1]",       EncryptKey[1] , 8); strcat(Line, " # [32-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    // Write_Hex    (Line, "EncryptKey[2]",       EncryptKey[2] , 8); strcat(Line, " # [32-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
    // Write_Hex    (Line, "EncryptKey[3]",       EncryptKey[3] , 8); strcat(Line, " # [32-bit]\n"); if(fputs(Line, File)==EOF) return EOF;
#endif
    Write_UnsDec (Line, "Verbose"  ,      (uint32_t)Verbose     ); strcat(Line, " #  [ 0..3]\n"); if(fputs(Line, File)==EOF) return EOF;
    Write_UnsDec (Line, "PPSdelay"  ,(uint32_t)PPSdelay         ); strcat(Line, " #  [   ms]\n"); if(fputs(Line, File)==EOF) return EOF;
#ifdef WITH_BT_PWR
    Write_UnsDec (Line, "Bluetooth" ,          BT_ON            ); strcat(Line, " #  [  1|0]\n"); if(fputs(Line, File)==EOF) return EOF;
#endif
    for(uint8_t Idx=0; Idx<InfoParmNum; Idx++)
    { Write_String (Line, OGN_Packet::InfoParmName(Idx), InfoParmValue(Idx)); strcat(Line, "; #  [char]\n"); if(fputs(Line, File)==EOF) return EOF; }
#ifdef WITH_BT_SPP
    strcpy(Line, "BTname         = "); strcat(Line, BTname); strcat(Line, "; #  [char]\n"); if(fputs(Line, File)==EOF) return EOF;
#endif
#ifdef WITH_WIFI
    for(uint8_t Idx=0; Idx<WIFIsets; Idx++)
    { if(WIFIname[Idx][0]==0) continue;
      strcpy(Line, "WIFIname"); Line[8]='0'+Idx; Line[9]='='; strcpy(Line+10, WIFIname[Idx]); strcat(Line, "; #  [char]\n"); if(fputs(Line, File)==EOF) return EOF;
      strcpy(Line, "WIFIpass"); Line[8]='0'+Idx; Line[9]='='; strcpy(Line+10, WIFIpass[Idx]); strcat(Line, "; #  [char]\n"); if(fputs(Line, File)==EOF) return EOF; }
    // Write_String (Line, "WIFIname", WIFIname[0]); strcat(Line, " #  [char]\n"); if(fputs(Line, File)==EOF) return EOF;
    // Write_String (Line, "WIFIpass", WIFIpass[0]); strcat(Line, " #  [char]\n"); if(fputs(Line, File)==EOF) return EOF;
#endif
    return 10+InfoParmNum; }

  int WriteToFile(const char *Name = "/spiffs/TRACKER.CFG")
  { FILE *File=fopen(Name, "wt"); if(File==0) return 0;
    int Lines=WriteToFile(File);
    fclose(File); return Lines; }

  void Write(void (*Output)(char))
  { char Line[80];
    Write_Hex    (Line, "Address"   ,          Address ,       6); strcat(Line, " # [24-bit]\n"); Format_String(Output, Line);
    Write_Hex    (Line, "AddrType"  ,          AddrType,       1); strcat(Line, " #  [2-bit]\n"); Format_String(Output, Line);
    Write_Hex    (Line, "AcftType"  ,          AcftType,       1); strcat(Line, " #  [4-bit]\n"); Format_String(Output, Line);
    Write_UnsDec (Line, "CONbaud"   ,          CONbaud          ); strcat(Line, " #  [  bps]\n"); Format_String(Output, Line);
    Write_Hex    (Line, "CONprot"   ,          CONprot,        1); strcat(Line, " #  [ mask]\n"); Format_String(Output, Line);
    Write_SignDec(Line, "TxPower"   ,          getTxPower()     ); strcat(Line, " #  [  dBm]\n"); Format_String(Output, Line);
    Write_UnsDec (Line, "TxHW"      ,(uint32_t)isTxTypeHW()     ); strcat(Line, " #  [ bool]\n"); Format_String(Output, Line);
    Write_UnsDec (Line, "FreqPlan"  ,(uint32_t)FreqPlan         ); strcat(Line, " #  [ 0..5]\n"); Format_String(Output, Line);
    Write_Float1 (Line, "FreqCorr"  , (int32_t)RFchipFreqCorr   ); strcat(Line, " #  [  ppm]\n"); Format_String(Output, Line);
    Write_SignDec(Line, "TempCorr"  , (int32_t)RFchipTempCorr   ); strcat(Line, " #  [ degC]\n"); Format_String(Output, Line);
    Write_Float1 (Line, "PressCorr" , (int32_t)PressCorr*10/4   ); strcat(Line, " #  [   Pa]\n"); Format_String(Output, Line);
    Write_SignDec(Line, "TimeCorr"  , (int32_t)TimeCorr         ); strcat(Line, " #  [    s]\n"); Format_String(Output, Line);
    Write_Float1 (Line, "GeoidSepar",          GeoidSepar       ); strcat(Line, " #  [    m]\n"); Format_String(Output, Line);
    Write_UnsDec (Line, "manGeoidSepar" ,      manGeoidSepar    ); strcat(Line, " #  [  1|0]\n"); Format_String(Output, Line);
    Write_UnsDec (Line, "NavMode"  ,      (uint32_t)NavMode     ); strcat(Line, " #  [ 0..7]\n"); Format_String(Output, Line);
    Write_UnsDec (Line, "NavRate"  ,      (uint32_t)NavRate     ); strcat(Line, " #  [  1,2]\n"); Format_String(Output, Line);
#ifdef WITH_ENCRYPT
    Write_UnsDec (Line, "Encrypt"   ,          Encrypt          ); strcat(Line, " #  [  1|0]\n"); Format_String(Output, Line);
    // Write_Hex    (Line, "EncryptKey[0]",       EncryptKey[0] , 8); strcat(Line, " # [32-bit]\n"); Format_String(Output, Line);
    // Write_Hex    (Line, "EncryptKey[1]",       EncryptKey[1] , 8); strcat(Line, " # [32-bit]\n"); Format_String(Output, Line);
    // Write_Hex    (Line, "EncryptKey[2]",       EncryptKey[2] , 8); strcat(Line, " # [32-bit]\n"); Format_String(Output, Line);
    // Write_Hex    (Line, "EncryptKey[3]",       EncryptKey[3] , 8); strcat(Line, " # [32-bit]\n"); Format_String(Output, Line);
#endif
    Write_UnsDec (Line, "Verbose"  ,      (uint32_t)Verbose     ); strcat(Line, " #  [ 0..3]\n"); Format_String(Output, Line);
    Write_UnsDec (Line, "PPSdelay"  ,(uint32_t)PPSdelay         ); strcat(Line, " #  [   ms]\n"); Format_String(Output, Line);
#ifdef WITH_BT_PWR
    Write_UnsDec (Line, "Bluetooth" ,          BT_ON            ); strcat(Line, " #  [  1|0]\n"); Format_String(Output, Line);
#endif
#ifdef WITH_BT_SPP
    strcpy(Line, "BTname         = "); strcat(Line, BTname); strcat(Line, "; #  [char]\n"); Format_String(Output, Line);
#endif
    for(uint8_t Idx=0; Idx<InfoParmNum; Idx++)
    { Write_String (Line, OGN_Packet::InfoParmName(Idx), InfoParmValue(Idx)); strcat(Line, "; #  [char]\n"); Format_String(Output, Line); }
#ifdef WITH_WIFI
    for(uint8_t Idx=0; Idx<WIFIsets; Idx++)
    { if(WIFIname[Idx][0]==0) continue;
      strcpy(Line, "WIFIname"); Line[8]='0'+Idx; Line[9]='='; strcpy(Line+10, WIFIname[Idx]); strcat(Line, "; #  [char]\n"); Format_String(Output, Line);
      strcpy(Line, "WIFIpass"); Line[8]='0'+Idx; Line[9]='='; strcpy(Line+10, WIFIpass[Idx]); strcat(Line, "; #  [char]\n"); Format_String(Output, Line);; }
    // Write_String (Line, "WIFIname", WIFIname[0]); strcat(Line, " #  [char]\n"); Format_String(Output, Line);
    // Write_String (Line, "WIFIpass", WIFIpass[0]); strcat(Line, " #  [char]\n"); Format_String(Output, Line);
#endif
  }

} ;

#endif // __PARAMETERS_H__
