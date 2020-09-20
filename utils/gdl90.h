#ifndef __GDL90_H__
#define __GDL90_H__

#include <stdio.h>
#include <stdint.h>

#include "format.h"

// =================================================================================

uint16_t GDL90_CRC16(uint8_t Byte, uint16_t CRC);                        // pass a single byte through the CRC
uint16_t GDL90_CRC16(const uint8_t *Data, uint8_t Len, uint16_t CRC=0);  // pass a packet of bytes through the CRC

int GDL90_Send(void (*Output)(char), uint8_t ID, const uint8_t *Data, int Len);  // transmit GDL90 packet with proper framing and CRC

// =================================================================================

// https://www.faa.gov/nextgen/programs/adsb/archival/media/gdl90_public_icd_reva.pdf
// SkyRadar msg ID 101: https://github.com/etdey/gdl90/blob/master/Format%20of%20the%20SkyRadar%20receiver%20message%20ID%20101.pdf
// other msg ID: https://www.foreflight.com/connect/spec/

class GDL90_HEARTBEAT        // Heart-beat packet to be send at every UTC second
{ public:
   static const int Size=6;
   union
   { uint8_t Status1;
     struct
     { bool  Initialized: 1;
       bool  reserved   : 1;
       bool  RATCS      : 1;
       bool  LowBatt    : 1; // battery is LOW
       bool  AddrType   : 1; // are we transmitting ICAO (0) or self-assigned address (1)
       bool  IDENT      : 1;
       bool  MaintReq   : 1;
       bool  PosValid   : 1; // GPS position is valid
     } ;
   } ;
   union
   { uint8_t Status2;
     struct
     { bool  UTCvalid     : 1; // UTC timing is valid
       bool  reserved1    : 1;
       bool  reserved2    : 1;
       bool  reserved3    : 1;
       bool  reserved4    : 1;
       bool  CSA_Req      : 1; // CSA has been requested (Conflict Situation Awareness)
       bool  CSA_NotAvail : 1; // CSA is not available
       bool  TimeStampMSB : 1; // [0x10000sec] highest TimeStamp bit
     } ;
   } ;
   uint16_t TimeStamp;                // [sec] since 0000z cut to 16-bit
   uint8_t  MsgCount[2];              // [/sec] counts messages received during the previous second

  public:
   void Clear(void)
   { Status1=0; Status2=0; TimeStamp=0; MsgCount[0]=0; MsgCount[1]=0; }

   void setTimeStamp(uint32_t Time)
   { Time%=86400; TimeStamp=Time; TimeStampMSB=Time>>16; }

   uint32_t getTimeStamp(void) const
   { uint32_t Time=TimeStampMSB; Time = (Time<<16) | TimeStamp; return Time; }

   uint8_t getUplinkCount(void) const { return MsgCount[0]>>3; }                            // Uplink messages received
   void setUplinkCount(uint8_t Count) { MsgCount[0] = (MsgCount[0]&0x07) | Count<<3; }

   uint16_t getDownlinkCount(void) const { uint16_t Count = MsgCount[0]&0x03; return (Count<<8) | MsgCount[1]; } // Basic and Long messages received
   void setDownlinkCount(uint8_t Count) { MsgCount[0] = (MsgCount[0]&0xFC) | (Count>>8); MsgCount[1] = Count; }

   int Send(void (*Output)(char)) const { return GDL90_Send(Output, 0, (const uint8_t *)this, Size); }

} __attribute__((packed));

// class GSL90_CONFIG    // Initialization, ID=117
// { public:
//    uint8_t Data[19];
// } ;

class GDL90_GEOMALT     // Geometrical altitude: ID = 11 (GPS ref. to Ellipsoid)
{ public:
   static const int Size=4;
   uint8_t Data[Size];

  public:
   void setAltitude(int32_t Alt)  // [5 feet] GPS altitude (ref. to Ellipsoid)
   { if(Alt>0x7FFF) Alt=0x7FFF;
     else if(Alt<(-0x8000)) Alt=(-0x8000);
     Data[0]=Alt>>8; Data[1]=Alt&0x0FF; }
   void setWarning(bool Warn)     // [bool]
   { if(Warn) Data[2]|=0x80;
         else Data[2]&=0x7F; }
   void setFOM(uint16_t FOM)      // [m] vertical Figure of Merit (accuracy ?)
   { Data[2] = (Data[2]&0x80) | (FOM>>8);
     Data[3] = FOM&0xFF; }

} ;

class GDL90_REPORT  // Position report: Traffic: ID = 20, Ownship: ID = 10
{ public:
   static const int Size=27;
   union
   { uint8_t Data[Size]; // 27 bytes excluding the ID and framing/CRC
/*
     struct
     { uint8_t  AddrType  : 4;     //
       uint8_t  Alert     : 4;     // 1=alert
       uint32_t Address   :24;     // byte-reversed
       uint32_t Latitude  :24;     // byte-reversed
       uint32_t Longitude :24;     // byte-reversed
       uint16_t Altitude  :12;     // garbled
       uint8_t  Misc      : 4;     // garbled
       uint8_t  NACp      : 4;
       uint8_t  NIC       : 4;
       uint16_t Velocity  :12;     // garbled
       uint16_t Climb     :12;     // garbled
       uint8_t  Track     : 8;
       uint8_t  AcftCat   : 8;
       char     Call[8];
       uint8_t  Spare     : 4;
       uint8_t  Priority  : 4;
     } ;
*/
   } ; // __attribute__((packed));

  public:
   static uint32_t get3bytes(const uint8_t *Byte) { uint32_t Word=Byte[0]; Word=(Word<<8) | Byte[1]; Word=(Word<<8) | Byte[2]; return Word; } // 3-byte value
   static void     set3bytes(uint8_t *Byte, uint32_t Word) { Byte[0]=Word>>16; Byte[1]=Word>>8; Byte[2]=Word; }

   void Clear(void) { for(int Idx=0; Idx<27; Idx++) Data[Idx]=0; }                              // clear all data (Lat/Lon = invalid)

   uint8_t getAlertStatus(void) const { return Data[0]>>4; }                                    // 0 = no alert, 1 = alert
   void    setAlertStatus(uint8_t Status) { Data[0] = (Data[0]&0x0F) | (Status<<4); }

   uint8_t getAddrType(void) const { return Data[0]&0x0F; }                                     // 0=ICAO, 1=non-ICAO, 4=surface vehicle, 5=ground beacon
   void    setAddrType(uint8_t AddrType) { Data[0] = (Data[0]&0xF0) | (AddrType&0x0F); }

   uint32_t getAddress(void) const { return get3bytes(Data+1); }
   void    setAddress(uint32_t Addr) { set3bytes(Data+1, Addr); }
   int32_t getLatitude(void) const { return get3bytes(Data+4)<<8; }                             // [cyclic]
   void    setLatitude(int32_t Lat) { set3bytes(Data+4, (Lat>>8)&0xFFFFFF); }
   int32_t getLongitude(void) const { return get3bytes(Data+7)<<8; }                            // [cyclic]
   void    setLongitude(int32_t Lon) { set3bytes(Data+7, (Lon>>8)&0xFFFFFF); }

   static int32_t CordicOGN(int32_t Coord) { return ((int64_t)Coord*83399993+(1<<21))>>22; }    // [1/60000deg] => [cordic]

   void    setLatOGN(int32_t Lat) { setLatitude (CordicOGN(Lat)); }                             // [1/60000deg] 
   void    setLonOGN(int32_t Lon) { setLongitude(CordicOGN(Lon)); }                             // [1/60000deg]

   int32_t getAltitude(void) const { int32_t Alt=Data[10]; Alt=(Alt<<4) | (Data[11]>>4); return Alt*25-1000; } // [feet]
   void setAltitude(int32_t Alt)                                                                               // [feet]
   { Alt = (Alt+1000+12)/25;
     if(Alt<0) Alt=0; else if(Alt>0xFFF) Alt=0xFFF;
     Data[10] = Alt>>4; Alt&=0x00F; Data[11] = (Data[11]&0x0F) | (Alt<<4); }

   uint8_t getMiscInd(void) const { return Data[11]&0x0F; }   // Airborne | Extrapolated | TT: 00=not valid, 01=true track, 10=magnetic, 11=heading
   void    setMiscInd(uint8_t MiscInd) { Data[11] = (Data[11]&0xF0) | (MiscInd&0x0F); }

   uint8_t getNIC(void) const { return Data[12]>>4; }         // containment radius: 9=75m, 10=25m, 11=7.5m
   uint8_t getNACp(void) const { return Data[12]&0x0F; }      // est. pos. uncertainty: 9=30m, 10=10m, 11=3m
   void setAccuracy(uint8_t NIC, uint8_t NACp) { Data[12] = (NIC<<4) | (NACp&0x0F); }

   uint16_t getSpeed(void) const { uint16_t Speed=Data[13]; Speed=(Speed<<4) | (Data[14]>>4); return Speed; }  // [knot]
   void     setSpeed(uint16_t Speed) { if(Speed>0xFFE) Speed=0xFFE; Data[13] = Speed>>4; Data[14] = (Data[14]&0x0F) | (Speed<<4); } // [knot]
   void     clrSpeed(void) { Data[13] = 0xFF; Data[14] = (Data[14]&0x0F) | 0xF0; }                               // Speed = invalid

   int32_t getClimbRate(void) const
   { int16_t Climb=Data[14]&0x0F; Climb=(Climb<<8)|Data[15]; Climb<<=4; return (int32_t)Climb*4; }               // [fpm]
   void    setClimbRate(int32_t Climb)                                                                           // [fpm]
   { Climb = (Climb+32)>>6; if(Climb<(-510)) Climb=(-510); else if(Climb>510) Climb=510;                         // full 12-bit range is not being used
     Data[15] = Climb&0xFF; Data[14] = (Data[14]&0xF0) | ((Climb>>8)&0x0F); }
   void    clrClimbRate(void) { Data[15]=0x00; Data[14] = (Data[14]&0xF0) | 0x08; }                              // set vertical rate = not available

   uint8_t getHeading(void) const { return Data[16]; }                                                            // [cyclic]
   void    setHeading(uint8_t Heading) { Data[16]=Heading; }                                                      // [cyclic]

   uint8_t getAcftCat(void) const { return Data[17]; }      // 1=light, 2=small, 3=large, 4=high vortex, 5=heavy, 6=high-G, 7=rotor, 9=glider, 10=airship, 11=parachute, 12=ULM/para/hang, 14=UAV, 15=space, 17=surf, 18=service
   void    setAcftCat(uint8_t Cat) { Data[17]=Cat; }
   void    setAcftType(uint8_t AcftType)                    // set OGN-type aricrraft-type
   {                            // 0, 1, 2, 3,  4,  5,  6,  7,  8,  9, A, B, C, D, E, F
     const uint8_t OGNtype[16] = { 0, 9, 1, 7, 11,  1, 12, 12,  1,  3,15,10,10,14,18,19 } ; // conversion table from OGN aricraft-type
     setAcftCat(OGNtype[AcftType&0x0F]); }

   const char *getAcftCall(void) const { return (const char *)(Data+18); } // is not null-terminated
   void setAcftCall(const char *Call)
   { int Idx=0;
     for( ; Idx<8; Idx++)
     { char ch=Call[Idx]; if(ch==0) break;
       Data[18+Idx]=ch; }
     for( ; Idx<8; Idx++)
       Data[18+Idx]=' ';
   }
   void setAcftCall(uint32_t ID)
   { const char *AddrName[4] = { "RN", "IC", "FL", "OG" } ;
     char *Call = (char *)(Data+18);
     const char *Name = AddrName[(ID>>24)&3]; Call[0]=Name[0]; Call[1]=Name[1];
     Format_Hex(Call+2, (uint8_t)((ID>>16)&0xFF)); Format_Hex(Call+4, (uint16_t)(ID&0xFFFF)); }

   uint8_t getPriority(void) const { return Data[26]>>4; } // 1=general, 2=medical, 3=fuel, 4=comms, 5=interference, 6=downed
   void    setPriority(uint8_t Prior) { Data[26] = (Data[26]&0x0F) | (Prior<<4); }

  int Send(void (*Output)(char), uint8_t ID=10) const { return GDL90_Send(Output, ID, Data, Size); }

  void Print(void) const
  { printf("%X:%06X %02X/%8s %X/%X %dft %+dfpm [%+09.5f,%+010.5f] %03.0f/%dkt\n",
       getAddrType(), getAddress(), getAcftCat(), getAcftCall(),
       getNIC(), getNACp(), getAltitude(), getClimbRate(),
       (90.0/0x40000000)*getLatitude(), (90.0/0x40000000)*getLongitude(),
       (360.0/256)*getHeading(), getSpeed()); }

} ;

// =================================================================================

class GDL90_RxMsg // receiver for the MAV messages
{ public:
   static const uint8_t MaxBytes = 32; // max. number of bytes
   static const uint8_t SYNC   = 0x7E; // GDL90 sync byte
   static const uint8_t ESC    = 0x7D; // GDL90 escape byte

   uint8_t  Byte[MaxBytes];
   uint8_t  Len;

  public:
   void Clear(void) { Len=0; Byte[Len]=0; }

   void Print(void)
   { printf("GDL90[%d] ", Len);
     for(int Idx=0; Idx<Len; Idx++)
       printf("%02X", Byte[Idx]);
     printf("\n"); }

   uint8_t ProcessByte(uint8_t RxByte)                       // process a single byte: add to the message or reject
   { // printf("Process[%2d] 0x%02X\n", Len, RxByte);
     if(Len==0)                                              // if the very first byte
     { if(RxByte==SYNC) { Byte[Len]=RxByte; return 1; }      // is SYNC then store it
       else if(Byte[Len]==SYNC)
       { Byte[Len]=RxByte; if(RxByte==ESC) return 1;
         Len++; Byte[Len]=0; return 1; }
       else if(Byte[Len]==ESC)
       { RxByte^=0x20; Byte[Len++]=RxByte; Byte[Len]=0; return 1; }
       else return 0; }
     if(RxByte==SYNC)                                        // if not the very first byte and SYNC then the packet is possibly complete
     { if(Len<3 || Byte[Len]!=0) { Clear(); return 0; }
       uint16_t CRC=0;
       for(int Idx=0; Idx<(Len-2); Idx++)
       { CRC=GDL90_CRC16(Byte[Idx], CRC); }
       if( (CRC&0xFF)!=Byte[Len-2] || (CRC>>8)!=Byte[Len-1] ) { Clear(); return 0; }
       return 2; }
     if(Byte[Len]==ESC) { RxByte^=0x20; }                    // if after an ESC then xor with 0x20
     Byte[Len]=RxByte; if(RxByte==ESC) return 1;
     Len++;                                                  // advance
     if(Len>=MaxBytes) { Clear(); return 0; }
     Byte[Len]=0; return 1; }

} ;

// =================================================================================

#endif // __GDL90_H__
