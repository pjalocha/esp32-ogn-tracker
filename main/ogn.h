#ifndef __OGN_H__
#define __OGN_H__

#include <stdio.h>

#include <string.h>
#include <stdint.h>
#ifndef __AVR__
#include <time.h>
#endif

#include <math.h>

#include "intmath.h"

#include "bitcount.h"
#include "nmea.h"
#include "ubx.h"
#include "mavlink.h"

#include "ldpc.h"

#include "format.h"

#include "ognconv.h"

#include "ogn1.h"    // OGN v1
#include "ogn2.h"    // OGN v2
#include "adsl.h"    // ADS-L
#include "fanet.h"
#include "gdl90.h"

#include "atmosphere.h"

// ---------------------------------------------------------------------------------------------------------------------

template <class OGNx_Packet, class OGNy_Packet>
 static bool OGN_isSignif(const OGNx_Packet *Packet, const OGNy_Packet *PrevPacket)       // is significant: decide whether to store it or not
{ if(PrevPacket==0) return 1;
  int8_t TimeDelta = Packet->Position.Time - PrevPacket->Position.Time;
  if(TimeDelta<0) TimeDelta+=60;                                              // [sec] time since previous  packet
  if(TimeDelta>=20) return 1;                                                 // [sec]
  int16_t Climb = Packet->DecodeClimbRate();                                  // [0.1m/s]
  if(abs(Climb)>=100) return 1;                                               // if climb/decent rate more than 10m/s
  int32_t AltDelta=Packet->DecodeAltitude()-PrevPacket->DecodeAltitude();     // [m] altitude change
  if(abs(AltDelta)>=20)  return 1;                                            // if more than 50m altitude change
  int16_t PrevClimb = PrevPacket->DecodeClimbRate();                          // [0.1m/s]
  int32_t DistDeltaV = (int32_t)(Climb-PrevClimb)*TimeDelta;                  // [0.1m]
  if(abs(DistDeltaV)>=200) return 1;                                          // if climb doistance >= 20m
  int16_t Speed = Packet->DecodeSpeed();                                      // [0.1m/s]
  int16_t PrevSpeed = PrevPacket->DecodeSpeed();                              // [0.1m/s]
  int32_t DistDeltaH = (int32_t)(Speed-PrevSpeed)*TimeDelta;                  // [0.1m] speed change * time since last recorded packet
  if(abs(DistDeltaH)>=200) return 1;                                          // if extrapolation error more than 50m
  int16_t Turn = Packet->DecodeTurnRate();                                    // [0.1deg/s]
  int16_t CFaccel = ((int32_t)Turn*Speed*229+0x10000)>>17;                    // [0.1m/s^2] centrifugal acceleration in turn
  if(abs(CFaccel)>=25) return 1;                                              // CFaccel at or above 5m/s^2 (0.5g)
  int16_t PrevTurn = PrevPacket->DecodeTurnRate();                            // [0.1deg/s]
  int16_t PrevCFaccel = ((int32_t)PrevTurn*PrevSpeed*229+0x10000)>>17;        // [0.1m/s^2]
  int32_t DistDeltaR = abs(CFaccel-PrevCFaccel)*(int32_t)TimeDelta*TimeDelta/2; // [0.1m]
  if(abs(DistDeltaR)>=200) return 1;                                          // [0.1m]
  return 0; }


// ---------------------------------------------------------------------------------------------------------------------

template <class OGNx_Packet=OGN1_Packet>
 class OGN_TxPacket                                    // OGN packet with FEC code, like for transmission
{ public:
   static const int     Words =  7;
   static const int     Bytes = 26;

   OGNx_Packet Packet;    // OGN packet

   uint32_t FEC[2];       // Gallager code: 48 check bits for 160 user bits

  public:

   uint8_t Print(char *Out)
   { uint8_t Len=0;
     Out[Len++]=HexDigit(Packet.Position.AcftType); Out[Len++]=':';
     Out[Len++]='0'+Packet.Header.AddrType; Out[Len++]=':';
     uint32_t Addr = Packet.Header.Address;
     Len+=Format_Hex(Out+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(Out+Len, (uint16_t)Addr);
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint16_t)Packet.Position.Time, 2);
     Out[Len++]=' ';
     Len+=Format_Latitude(Out+Len, Packet.DecodeLatitude());
     Out[Len++]=' ';
     Len+=Format_Longitude(Out+Len, Packet.DecodeLongitude());
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint32_t)Packet.DecodeAltitude()); Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, Packet.DecodeSpeed(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, Packet.DecodeClimbRate(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]='\n'; Out[Len]=0;
     return Len; }

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX [%08lX %04lX] (%d)\n",
             (long int)Packet.HeaderWord, (long int)Packet.Data[0], (long int)Packet.Data[1],
             (long int)Packet.Data[2], (long int)Packet.Data[3], (long int)FEC[0],
             (long int)FEC[1], (int)checkFEC() ); }

   void DumpBytes(void) const
   { for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { printf(" %02X", Packet.Byte()[Idx]); }
     printf("\n"); }

   // void calcFEC(void)                            { LDPC_Encode(&Packet.HeaderWord, FEC); }       // calculate the 48-bit parity check
   // void calcFEC(const uint32_t ParityGen[48][5]) { LDPC_Encode(&PacketHeaderWord,  FEC, ParityGen); }
   void    calcFEC(void)                   { LDPC_Encode(Packet.Word()); }       // calculate the 48-bit parity check
   uint8_t checkFEC(void)    const  { return LDPC_Check(Packet.Word()); }        // returns number of parity checks that fail (0 => no errors, all fine)

   uint8_t  *Byte(void) const { return (uint8_t  *)&Packet.HeaderWord; } // packet as bytes
   uint32_t *Word(void) const { return (uint32_t *)&Packet.HeaderWord; } // packet as words

   void recvBytes(const uint8_t *SrcPacket) { memcpy(Byte(), SrcPacket, Bytes); } // load data bytes e.g. from a demodulator
/*
   uint8_t calcErrorPattern(uint8_t *ErrPatt, const uint8_t *OtherPacket) const
   { uint8_t ByteIdx=0; const uint32_t *WordPtr=Packet.Word();
     for(uint8_t WordIdx=0; WordIdx<Words; WordIdx++)
     { uint32_t Word=WordPtr[WordIdx];
       for(int Idx=0; Idx<4; Idx++)
       { if(ByteIdx>=Bytes) break;
         ErrPatt[ByteIdx]=Packet[ByteIdx]^Word; ByteIdx++;
         Word>>=8; }
     }
     return Bytes; }
*/
} ;

// ---------------------------------------------------------------------------------------------------------------------

template <class OGNx_Packet>
 class OGN_LogPacket                                       // OGN packet in an internal binary log file
{ public:
   static const int     Words =  6;
   static const int     Bytes = 24;

   OGNx_Packet Packet;
   uint16_t   Time;            // [16sec] truncated time
   union
   { uint8_t Flags;
     struct
     { uint8_t SNR : 6;        // [dB]
       uint8_t Prot: 1;
       uint8_t Rx  : 1;        // received or (own) transmitted ?
     } ;
   } ;
   uint8_t    Check;           // simple control sum

   void     setTime(uint32_t EstTime) { Time = EstTime>>4; }
   uint32_t getTime(uint32_t EstTime) const
   { EstTime>>=4;
     int16_t Diff = Time-EstTime;
     EstTime += Diff;
     return (EstTime<<4)+15; }

   uint8_t calcCheck(void) const
   { uint8_t Check=0x5A;
     uint8_t *Data = (uint8_t*)&Packet;
     for(uint8_t Idx=0; Idx<(Bytes-1); Idx++)
     { Check+=Data[Idx]; }
     return Check^0xA5; }
   void setCheck(void) { Check=calcCheck(); }
   bool isCorrect(void) const { return calcCheck()==Check; }
} ;

// ---------------------------------------------------------------------------------------------------------------------

template <class OGNx_Packet=OGN1_Packet>
 class OGN_RxPacket                                        // OGN packet with FEC code and some reception info
{ public:
   static const int     Words =  7;
   static const int     Bytes = 26;

   OGNx_Packet Packet;

   uint32_t FEC[2];       // Gallager code: 48 check bits for 160 user bits

   union
   { uint8_t State;       // state bits and small values
     struct
     { // bool Saved   :1;   // has been already saved in internal storage
       // bool Ready   :1;   // is ready for transmission
       // bool Sent    :1;   // has already been transmitted out
       bool Alloc   :1;   // allocated in a queue or list
       bool Correct :1;   // correctly received or corrected by FEC
       uint8_t RxErr:4;   // number of bit errors corrected upon reception
       uint8_t Warn :2;   // LookOut warning level
     } ;
   } ;

   uint8_t RxChan;        // RF channel where the packet was received
   uint8_t RxRSSI;        // [-0.5dBm]
   uint8_t Rank;          // rank: low altitude and weak signal => high rank

   int16_t LatDist;       // [m] distance along the latitude
   int16_t LonDist;       // [m] distance along the longitude
   // int16_t AltDist;       // [m]

  public:

   OGN_RxPacket() { Clear(); }
   void Clear(void) { Packet.Clear(); State=0; Rank=0; }

   uint8_t  *Byte(void) const { return (uint8_t  *)&Packet.HeaderWord; } // packet as bytes
   uint32_t *Word(void) const { return (uint32_t *)&Packet.HeaderWord; } // packet as words

   void recvBytes(const uint8_t *SrcPacket) { memcpy(Byte(), SrcPacket, Bytes); } // load data bytes e.g. from a demodulator

   uint8_t calcErrorPattern(uint8_t *ErrPatt, const uint8_t *OtherPacket) const
   { uint8_t ByteIdx=0; const uint32_t *WordPtr=Packet.Word();
     for(uint8_t WordIdx=0; WordIdx<Words; WordIdx++)
     { uint32_t Word=WordPtr[WordIdx];
       for(int Idx=0; Idx<4; Idx++)
       { if(ByteIdx>=Bytes) break;
         ErrPatt[ByteIdx]=OtherPacket[ByteIdx]^Word; ByteIdx++;
         Word>>=8; }
     }
     return Bytes; }

   // void calcFEC(void)                            { LDPC_Encode(&Packet.HeaderWord, FEC); }       // calculate the 48-bit parity check
   // void calcFEC(const uint32_t ParityGen[48][5]) { LDPC_Encode(&PacketHeaderWord,  FEC, ParityGen); }
   void    calcFEC(void)                   { LDPC_Encode(Packet.Word()); }       // calculate the 48-bit parity check
   uint8_t checkFEC(void)    const  { return LDPC_Check(Packet.Word()); }        // returns number of parity checks that fail (0 => no errors, all fine)

   int BitErr(OGN_RxPacket &RefPacket) const // return number of different data bits between this Packet and RefPacket
   { return Count1s(Packet.HeaderWord^RefPacket.Packet.HeaderWord)
           +Count1s(Packet.Data[0]^RefPacket.Packet.Data[0])
           +Count1s(Packet.Data[1]^RefPacket.Packet.Data[1])
           +Count1s(Packet.Data[2]^RefPacket.Packet.Data[2])
           +Count1s(Packet.Data[3]^RefPacket.Packet.Data[3])
           +Count1s(FEC[0]^RefPacket.FEC[0])
           +Count1s((FEC[1]^RefPacket.FEC[1])&0xFFFF); }

   void calcRelayRank(int32_t RxAltitude)                               // [m] altitude of reception
   { if(Packet.Header.Emergency) { Rank=0xFF; return; }                 // emergency packets always highest rank
     Rank=0;
     if(Packet.Header.NonPos)     return;                               // only relay position packets
     if(Packet.Position.Time>=60) return;                               // don't relay packets with unknown time - but maybe we should ?
     if(Packet.Header.Relay)      return;                               // no rank for relayed packets (only single relay)
     if(RxRSSI>128)                                                     // [-0.5dB] weaker signal => higher rank
       Rank += (RxRSSI-128)>>2;                                         // 1point/2dB less signal
     if(Packet.Header.Encrypted)  return;                               // for exncrypted packets we only take signal strength
     RxAltitude -= Packet.DecodeAltitude();                             // [m] receiver altitude - target altitude
     if(RxAltitude>0)                                                   // 
       Rank += RxAltitude>>6;                                           // 1points/64m of altitude below
     int16_t ClimbRate = Packet.DecodeClimbRate();                      // [0.1m/s] higher sink rate => higher rank
     if(ClimbRate<0)
       Rank += (-ClimbRate)>>3;                                         // 1point/0.8m/s of sink
   }

   uint8_t ReadPOGNT(const char *NMEA)
   { uint8_t Len=0;
     if(memcmp(NMEA, "$POGNT,", 7)!=0) return -1;
     Len+=7;

     if(NMEA[Len+2]!=',') return -1;
     int8_t Time=Read_Dec2(NMEA+Len);
     if( (Time<0) || (Time>=60) ) return -1;
     Packet.Position.Time=Time;
     Len+=3;

     if(NMEA[Len+1]!=',') return -1;
     int8_t AcftType=Read_Hex1(NMEA[Len]);
     if(AcftType<0) return -1;
     Packet.Position.AcftType=AcftType;
     Len+=2;

     if(NMEA[Len+1]!=',') return -1;
     int8_t AddrType=Read_Hex1(NMEA[Len]);
     if((AddrType<0) || (AddrType>=4) ) return -1;
     Packet.Header.AddrType=AddrType;
     Len+=2;

     uint32_t Addr;
     int8_t Ret=Read_Hex(Addr, NMEA+Len); if(Ret<=0) return -1;
     if(NMEA[Len+Ret]!=',') return -1;
     Packet.Header.Address=Addr;
     Len+=Ret+1;

     if(NMEA[Len+1]!=',') return -1;
     int8_t Relay=Read_Hex1(NMEA[Len]);
     if( (Relay<0) || (Relay>1) ) return -1;
     Packet.Header.Relay=Relay;
     Len+=2;

     if(NMEA[Len+2]!=',') return -1;
     int8_t FixQuality=Read_Hex1(NMEA[Len]);
     int8_t FixMode=Read_Hex1(NMEA[Len+1]);
     if( (FixQuality<0) || (FixQuality>=4) ) return -1;
     if( (FixMode<0) || (FixMode>=2) ) return -1;
     Packet.Position.FixQuality=FixQuality;
     Packet.Position.FixMode=FixMode;
     Len+=3;

     int32_t DOP=0;
     Ret=Read_Float1(DOP, NMEA+Len); if(Ret<0) return -1;
     if(NMEA[Len+Ret]!=',') return -1;
     if(DOP<10) DOP=10;
     Packet.EncodeDOP(DOP-10);
     Len+=Ret+1;

     if(NMEA[Len+10]!=',') return -1;
     int8_t Deg=Read_Dec2(NMEA+Len); if(Deg<0) return -1;
     int8_t Min=Read_Dec2(NMEA+Len+2); if(Min<0) return -1;
     if(NMEA[Len+4]!='.') return -1;
     int16_t Frac=Read_Dec4(NMEA+Len+5); if(Frac<0) return -1;
     char Sign=NMEA[Len+9];
     int32_t Lat = Deg*600000 + Min*10000 + Frac;
     if(Sign=='N') { } else if(Sign=='S') { Lat=(-Lat); } else return -1;
     Packet.EncodeLatitude(Lat);
     Len+=11;

     if(NMEA[Len+11]!=',') return -1;
     Deg=Read_Dec3(NMEA+Len); if(Deg<0) return -1;
     Min=Read_Dec2(NMEA+Len+3); if(Min<0) return -1;
     if(NMEA[Len+5]!='.') return -1;
     Frac=Read_Dec4(NMEA+Len+6); if(Frac<0) return -1;
     Sign=NMEA[Len+10];
     int32_t Lon = Deg*600000 + Min*10000 + Frac;
     if(Sign=='E') { } else if(Sign=='W') { Lon=(-Lon); } else return -1;
     Packet.EncodeLongitude(Lon);
     Len+=12;

     int32_t Alt=0;
     Ret=Read_SignDec(Alt, NMEA+Len); if(Ret<0) return -1;
     Packet.EncodeAltitude(Alt);
     if(NMEA[Len+Ret]!=',') return -1;
     Len+=Ret+1;

     int32_t AltDiff=0;
     Ret=Read_SignDec(AltDiff, NMEA+Len); if(Ret<0) return -1;
     // printf("Ret=%d, AltDiff=%d -> %s\n", Ret, AltDiff, NMEA+Len);
     if(Ret==0) Packet.clrBaro();
           else Packet.setBaroAltDiff(AltDiff);
     if(NMEA[Len+Ret]!=',') return -1;
     Len+=Ret+1;

     int32_t Climb=0;
     Ret=Read_Float1(Climb, NMEA+Len); if(Ret<0) return -1;
     // printf("Ret=%d, Climb=%d -> %s\n", Ret, Climb, NMEA+Len);
     Packet.EncodeClimbRate(Climb);
     if(NMEA[Len+Ret]!=',') return -1;
     Len+=Ret+1;

     int32_t Speed=0;
     Ret=Read_Float1(Speed, NMEA+Len); if(Ret<0) return -1;
     Packet.EncodeSpeed(Speed);
     if(NMEA[Len+Ret]!=',') return -1;
     Len+=Ret+1;

     int32_t Heading=0;
     Ret=Read_Float1(Heading, NMEA+Len); if(Ret<0) return -1;
     Packet.EncodeHeading(Heading);
     if(NMEA[Len+Ret]!=',') return -1;
     Len+=Ret+1;

     int32_t TurnRate=0;
     Ret=Read_Float1(TurnRate, NMEA+Len); if(Ret<0) return -1;
     Packet.EncodeTurnRate(TurnRate);
     if(NMEA[Len+Ret]!=',') return -1;
     Len+=Ret+1;

     int32_t RSSI=0;
     Ret=Read_SignDec(RSSI, NMEA+Len); if(Ret<0) return -1;
     RxRSSI=(-2*RSSI);
     if(NMEA[Len+Ret]!=',') return -1;
     Len+=Ret+1;

     int32_t Err=0;
     Ret=Read_SignDec(Err, NMEA+Len); if(Ret<0) return -1;
     RxErr=Err;
     if(NMEA[Len+Ret]!='*') return -1;
     Len+=Ret+1;

     return Len; }

   uint8_t WritePOGNT(char *NMEA)
   { uint8_t Len=0;
     Len+=Format_String(NMEA+Len, "$POGNT,");                             // sentence name
     if(Packet.Position.Time<60)
       Len+=Format_UnsDec(NMEA+Len, (uint16_t)Packet.Position.Time, 2);   // [sec] time
     NMEA[Len++]=',';
     NMEA[Len++]=HexDigit(Packet.Position.AcftType);                      // [0..F] aircraft-type: 1=glider, 2=tow plane, etc.
     NMEA[Len++]=',';
     NMEA[Len++]='0'+Packet.Header.AddrType;                              // [0..3] address-type: 1=ICAO, 2=FLARM, 3=OGN
     NMEA[Len++]=',';
     uint32_t Addr = Packet.Header.Address;                               // [24-bit] address
     Len+=Format_Hex(NMEA+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(NMEA+Len, (uint16_t)Addr);
     NMEA[Len++]=',';
     NMEA[Len++]='0'+Packet.Header.Relay;                                 // [0..3] counts retransmissions
     NMEA[Len++]=',';
     NMEA[Len++]='0'+Packet.Position.FixQuality;                          // [] fix quality
     NMEA[Len++]='0'+Packet.Position.FixMode;                             // [] fix mode
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, (uint16_t)(Packet.DecodeDOP()+10),2,1); // [] Dilution of Precision
     NMEA[Len++]=',';
     Len+=Format_Latitude(NMEA+Len, Packet.DecodeLatitude());        // [] Latitude
     NMEA[Len++]=',';
     Len+=Format_Longitude(NMEA+Len, Packet.DecodeLongitude());      // [] Longitude
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, (uint32_t)Packet.DecodeAltitude());     // [m] Altitude (by GPS)
     NMEA[Len++]=',';
     if(Packet.hasBaro())
       Len+=Format_SignDec(NMEA+Len, (int32_t)Packet.getBaroAltDiff());   // [m] Standard Pressure Altitude (by Baro)
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, Packet.DecodeClimbRate(), 2, 1);       // [m/s] climb/sink rate (by GPS or pressure sensor)
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, Packet.DecodeSpeed(), 2, 1);         // [m/s] ground speed (by GPS)
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, Packet.DecodeHeading(), 4, 1);          // [deg] heading (by GPS)
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, Packet.DecodeTurnRate(), 2, 1);         // [deg/s] turning rate (by GPS)
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, -(int16_t)RxRSSI/2);            // [dBm] received signal level
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, (uint16_t)RxErr);                // [bits] corrected transmisison errors
     Len+=NMEA_AppendCheckCRNL(NMEA, Len);
     NMEA[Len]=0;
     return Len; }

   // produce PFLAA sentence (relative position) from a reference point [RefLat, RefLon]
   uint8_t WritePFLAA(char *NMEA, uint8_t Status, int32_t RefLat, int32_t RefLon, int32_t RefAlt, uint16_t LatCos)
   { int32_t LatDist=0, LonDist=0;
     if(Packet.calcDistanceVector(LatDist, LonDist, RefLat, RefLon, LatCos)<0) return 0;     // return zero, when distance too large
     int32_t AltDist = Packet.DecodeAltitude()-RefAlt;
     return WritePFLAA(NMEA, Status, LatDist, LonDist, AltDist, Status); }                            // return number of formatted characters

   uint8_t WritePFLAA(char *NMEA, uint8_t Status, int32_t LatDist, int32_t LonDist, int32_t AltDist)
   { uint8_t Len=0;
     Len+=Format_String(NMEA+Len, "$PFLAA,");                             // sentence name and alarm-level (but no alarms for trackers)
     NMEA[Len++]='0'+Status;
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, LatDist);
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, LonDist);
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, AltDist);                              // [m] relative altitude
     NMEA[Len++]=',';
     uint8_t AddrType = Packet.Header.AddrType;
#ifdef WITH_SKYDEMON
     if(AddrType!=1) AddrType=2;                                          // SkyDemon only accepts 1 or 2
#endif
     NMEA[Len++]='0'+AddrType;                                            // address-type (3=OGN)
     NMEA[Len++]=',';
     uint32_t Addr = Packet.Header.Address;                               // [24-bit] address
     Len+=Format_Hex(NMEA+Len, (uint8_t)(Addr>>16));                      // XXXXXX 24-bit address: RND, ICAO, FLARM, OGN
     Len+=Format_Hex(NMEA+Len, (uint16_t)Addr);
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, Packet.DecodeHeading(), 4, 1);          // [deg] heading (by GPS)
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, Packet.DecodeTurnRate(), 2, 1);        // [deg/sec] turn rate
     NMEA[Len++]=',';
     Len+=Format_UnsDec(NMEA+Len, Packet.DecodeSpeed(), 2, 1);            // [approx. m/s] ground speed
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, Packet.DecodeClimbRate(), 2, 1);       // [m/s] climb/sink rate
     NMEA[Len++]=',';
     NMEA[Len++]=HexDigit(Packet.Position.AcftType);                      // [0..F] aircraft-type: 1=glider, 2=tow plane, etc.
     Len+=NMEA_AppendCheckCRNL(NMEA, Len);
     NMEA[Len]=0;
     return Len; }                                                        // return number of formatted characters

   void Print(void) const
   { printf("[%02d/%+6.1fdBm/%2d] ", RxChan, -0.5*RxRSSI, RxErr);
     Packet.Print(); }

   uint8_t Print(char *Out) const
   { uint8_t Len=0;
     Out[Len++]=HexDigit(Packet.Position.AcftType); Out[Len++]=':';
     Out[Len++]='0'+Packet.Header.AddrType; Out[Len++]=':';
     uint32_t Addr = Packet.Header.Address;
     Len+=Format_Hex(Out+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(Out+Len, (uint16_t)Addr);
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, -(int16_t)RxRSSI/2); Out[Len++]='d'; Out[Len++]='B'; Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint16_t)Packet.Position.Time, 2);
     Out[Len++]='s'; Out[Len++]=' ';
     Len+=Format_Latitude(Out+Len, Packet.DecodeLatitude());
     Out[Len++]=' ';
     Len+=Format_Longitude(Out+Len, Packet.DecodeLongitude());
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint32_t)Packet.DecodeAltitude()); Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, Packet.DecodeSpeed(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, Packet.DecodeClimbRate(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' ';
     Out[Len++]='r';
     Len+=Format_Hex(Out+Len, Rank);
     Out[Len++]='\n'; Out[Len]=0;
     return Len; }

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX [%08lX %04lX] (%d)\n",
             (long int)Packet.HeaderWord, (long int)Packet.Data[0], (long int)Packet.Data[1],
             (long int)Packet.Data[2], (long int)Packet.Data[3],
             (long int)FEC[0], (long int)FEC[1], (int)checkFEC() ); }

   void DumpBytes(void) const
   { for(uint8_t Idx=0; Idx<26; Idx++)
     { printf(" %02X", Packet.Byte()[Idx]); }
     printf(" (%d)\n", LDPC_Check(Packet.Byte())); }

} ;

#ifdef WITH_PPM

class OGN_PPM_Packet                                        // OGN packet with FEC code and some reception info
{ public:
   static const int     Words = 12;

   OGN1_Packet Packet;

   uint32_t FEC[7];       // Gallager code: 194 check bits for 160 user bits

  public:

   void    calcFEC(void)                   { LDPC_Encode_n354k160(Packet.Word()); }       // calculate the 48-bit parity check
   uint8_t checkFEC(void)    const  { return LDPC_Check_n354k160(Packet.Word()); }        // returns number of parity checks that fail (0 => no errors, all fine)

   uint32_t *Word(void) const { return Packet.Word(); }

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX [%08lX %08lX %08lX %08lX %08lX %08lX %01lX] (%d)\n",
             (long int)Packet.HeaderWord, (long int)Packet.Data[0], (long int)Packet.Data[1],
             (long int)Packet.Data[2], (long int)Packet.Data[3],
             (long int)FEC[0], (long int)FEC[1], (long int)FEC[2], (long int)FEC[2],
             (long int)FEC[4], (long int)FEC[5], (long int)FEC[6], (int)checkFEC() ); }

   static uint8_t Gray(uint8_t Binary) { return Binary ^ (Binary>>1); }

   static uint8_t Binary(uint8_t Gray)
   { Gray = Gray ^ (Gray >> 4);
     Gray = Gray ^ (Gray >> 2);
     Gray = Gray ^ (Gray >> 1);
     return Gray; }

   uint8_t getSymbol(uint16_t Idx)
   { if(Idx>=59) return 0xFF;
     uint32_t *Word = Packet.Word();
     uint8_t Symbol=0; uint8_t SymbMask=1;
     for(uint8_t Bit=0; Bit<6; Bit++, Idx+=59 )
     { uint8_t WordIdx=Idx>>5; uint8_t BitIdx=Idx&31;
       uint32_t Mask=1; Mask<<=BitIdx;
       if(Word[WordIdx]&Mask) Symbol|=SymbMask;
       SymbMask<<=1; }
     return Gray(Symbol); }

   void clear(void)
   { memset(Packet.Word(), 0, Words*4); }

   void setSymbol(uint16_t Idx, uint8_t Symbol)
   { if(Idx>=59) return;
     Symbol = Binary(Symbol);
     uint32_t *Word = Packet.Word();
     for(uint8_t Bit=0; Bit<6; Bit++, Idx+=59 )
     { if(Symbol&1)
       { uint8_t WordIdx=Idx>>5; uint8_t BitIdx=Idx&31;
         uint32_t Mask=1; Mask<<=BitIdx;
         Word[WordIdx]|=Mask; }
       Symbol>>=1; }
   }

} ;

#endif // WITH_PPM

// ---------------------------------------------------------------------------------------------------------------------

template<class OGNx_Packet, uint8_t Size=8>
 class OGN_PrioQueue
{ public:
   // static const uint8_t Size = 8;            // number of packets kept
   OGN_RxPacket<OGNx_Packet> Packet[Size];   // OGN packets
   uint16_t             Sum;                 // sum of all ranks
   uint8_t              Low, LowIdx;         // the lowest rank and the index of it

  public:
   void Clear(void)                                                           // clear (reset) the queue
   { for(uint8_t Idx=0; Idx<Size; Idx++)                                      // clear every packet
     { Packet[Idx].Clear(); }
     Sum=0; Low=0; LowIdx=0; }                                                // clear the rank sum, lowest rank

   OGN_RxPacket<OGNx_Packet> * operator [](uint8_t Idx) { return Packet+Idx; }

   uint8_t getNew(void)                                                       // get (index of) a free or lowest rank packet
   { Sum-=Packet[LowIdx].Rank; Packet[LowIdx].Rank=0; Low=0; return LowIdx; } // remove old packet from the rank sum

   uint8_t size(void)                                                        // count all slots with Alloc flag set
   { uint8_t Count=0;
     for(uint8_t Idx=0; Idx<Size; Idx++)
     { if(Packet[Idx].Alloc) Count++; }
     return Count; }

   OGN_RxPacket<OGNx_Packet> *addNew(uint8_t NewIdx)                          // add the new packet to the queue
   { OGN_RxPacket<OGNx_Packet> *Prev = 0;
     Packet[NewIdx].Alloc=1;                                                  // mark this clot as allocated
     uint32_t AddressAndType = Packet[NewIdx].Packet.getAddressAndType();     // get ID of this packet: ID is address-type and address (2+24 = 26 bits)
     for(uint8_t Idx=0; Idx<Size; Idx++)                                      // look for other packets with same ID
     { if(Idx==NewIdx) continue;                                              // avoid the new packet
       if(Packet[Idx].Packet.getAddressAndType() == AddressAndType)           // if another packet with same ID:
       { Prev=Packet+Idx; clean(Idx); }                                       // then remove it: set rank to zero
     }
     uint8_t Rank=Packet[NewIdx].Rank; Sum+=Rank;                             // add the new packet to the rank sum
     if(NewIdx==LowIdx) reCalc();
     else { if(Rank<Low) { Low=Rank; LowIdx=NewIdx; } }
     // if(NewIdx!=LowIdx)                                                       //
     // { if(Rank<=Low) { Low=Rank; LowIdx=NewIdx; } }
     // else reCalc();
     return Prev; }

   uint8_t getRand(uint32_t Rand) const                                       // get a position by random selection but probabilities prop. to ranks
   { if(Sum==0) return Rand%Size;                                             //
     uint16_t RankIdx = Rand%Sum;
     uint8_t Idx; uint16_t RankSum=0;
     for(Idx=0; Idx<Size; Idx++)
     { if(Packet[Idx].Alloc==0) continue;
       uint8_t Rank=Packet[Idx].Rank; if(Rank==0) continue;
       RankSum+=Rank; if(RankSum>RankIdx) return Idx; }
     return Rand%Size; }

   void reCalc(void)                                                           // find the lowest rank and calc. the sum of all ranks
   { Sum=Low=Packet[0].Rank; LowIdx=0;                                         // take minimum at the first slot
     for(uint8_t Idx=1; Idx<Size; Idx++)                                       // loop over all other slots
     { if(Packet[Idx].Alloc==0) { Low=0; LowIdx=Idx; continue; }
       uint8_t Rank=Packet[Idx].Rank;
       Sum+=Rank;                                                              // sum up the ranks
       if(Rank<Low) { Low=Rank; LowIdx=Idx; }                                  // update the minimum
     }
   }

   void cleanTime(uint8_t Time)                                                // clean up slots of given Time
   { for(int Idx=0; Idx<Size; Idx++)
     { if(Packet[Idx].Alloc==0) continue;
       uint8_t PktTime=Packet[Idx].Packet.Position.Time;
       if( PktTime==Time || PktTime>=60) clean(Idx);
     }
   }

   void clean(uint8_t Idx)                                                      // clean given slot, remove it from the sum
   { Sum-=Packet[Idx].Rank; Packet[Idx].Rank=0; Packet[Idx].Alloc=0; Low=0; LowIdx=Idx; }

   void decrRank(uint8_t Idx, uint8_t Decr=1)                                   // decrement rank of given slot
   { uint8_t Rank=Packet[Idx].Rank; if(Rank==0) return;                         // if zero already: do nothing
     if(Decr>Rank) Decr=Rank;                                                   // if to decrement by more than the rank already: reduce the decrement
     Rank-=Decr; Sum-=Decr;                                                     // decrement the rank and the sum of ranks
     if(Rank<Low) { Low=Rank; LowIdx=Idx; }                                     // if new minimum: update the minimum.
     Packet[Idx].Rank=Rank; }                                                   // update the rank of this slot

   uint8_t Print(char *Out)
   { uint8_t Len=0;
     for(uint8_t Idx=0; Idx<Size; Idx++)                                        // loop through the slots
     { if(Packet[Idx].Alloc==0) continue;
       uint8_t Rank=Packet[Idx].Rank;
       Out[Len++]=' '; Len+=Format_Hex(Out+Len, Rank);                          // print the slot Rank
       // if(Rank)                                                                 // if Rank is none-zero
       { Out[Len++]='/'; Len+=Format_Hex(Out+Len, Packet[Idx].Packet.getAddressAndType() );   // print address-type and address
         Out[Len++]=':';
         if(Packet[Idx].Packet.Header.Encrypted) Len+=Format_String(Out+Len, "ee");
         else Len+=Format_UnsDec(Out+Len, Packet[Idx].Packet.Position.Time, 2); // [sec] print time
       }
     }
     Out[Len++]=' '; Len+=Format_Hex(Out+Len, Sum);                             // sum of all Ranks
     Out[Len++]='/'; Len+=Format_Hex(Out+Len, LowIdx);                          // index of the lowest Rank or a free slot
     Out[Len++]='\n'; Out[Len]=0; return Len; }

} ;

class GPS_Time
{ public:
   int8_t  Year, Month, Day;    // Date (UTC) from GPS
   int8_t  Hour, Min, Sec;      // Time-of-day (UTC) from GPS
   int16_t mSec;                // [ms]

  public:

   void setDefaultDate() { Year=00; Month=1; Day=1; } // default Date is 01-JAN-2000
   void setDefaultTime() { Hour=0;  Min=0;   Sec=0; mSec=0; } // default Time is 00:00:00.00

   uint8_t FormatDate(char *Out, char Sep='.') const
   { uint8_t Len=0;
     Len+=Format_UnsDec(Out+Len, Day, 2);
     Out[Len++]=Sep;
     Len+=Format_UnsDec(Out+Len, Month, 2);
     Out[Len++]=Sep;
     Len+=Format_UnsDec(Out+Len, Year, 2);
     Out[Len]=0; return Len; }

   uint8_t FormatTime(char *Out, char Sep=':') const
   { uint8_t Len=0;
     Len+=Format_UnsDec(Out+Len, Hour, 2);
     Out[Len++]=Sep;
     Len+=Format_UnsDec(Out+Len, Min, 2);
     Out[Len++]=Sep;
     Len+=Format_UnsDec(Out+Len, Sec, 2);
     Out[Len++]='.';
     Len+=Format_UnsDec(Out+Len, mSec, 3);
     Out[Len]=0; return Len; }

   bool isTimeValid(void) const                      // is the GPS time-of-day valid
   { return (Hour>=0) && (Min>=0) && (Sec>=0); }     // all data must have been correctly read: negative means not correctly read)

   bool isDateValid(void) const                      // is the GPS date valid ?
   { return (Year>=0) && (Month>=0) && (Day>=0); }

   int8_t incrTimeFrac(int16_t msFrac)                 // [ms]
   { mSec+=msFrac;
     if(mSec>=1000) { mSec-=1000; return incrTime(); }
     else if(mSec<0) { mSec+=1000; return decrTime(); }
     return 0; }

   int8_t incrTime(void)                             // increment HH:MM:SS by one second
   { Sec++;  if(Sec<60) return 0;
     Sec=0;
     Min++;  if(Min<60) return 0;
     Min=0;
     Hour++; if(Hour<24) return 0;
     Hour=0;
     return 1; }                                     // return 1 if date needs to be incremented

   int8_t decrTime(void)                             // decrement HH:MM:SS by one second
   { if(Sec>0) { Sec--; return 0; }
     Sec=60;
     if(Min>60) { Min--; return 0; }
     Min=60;
     if(Hour>0) { Hour--; return 0; }
     Hour=24;
     return -1; }                                     // return 1 if date needs to be decremented

   int32_t calcTimeDiff(const GPS_Time &RefTime) const
   { int32_t TimeDiff = msDayTime() - RefTime.msDayTime();                                  // [ms]
          if(TimeDiff<(-(int32_t)mSecsPerDay/2)) TimeDiff+=mSecsPerDay;                              // wrap-around one day
     else if(TimeDiff>= (int32_t)mSecsPerDay/2 ) TimeDiff-=mSecsPerDay;
     return TimeDiff; }                                                                     // [ms]

   uint8_t MonthDays(void)                           // number of days per month
   { const uint16_t Table = 0x0AD5;                  // 1010 1101 0101 0=30days, 1=31days
     // const uint8_t Table[12] = { 31,28,31,30, 31,30,31,31, 30,31,30,31 };
     if( (Month<1) || (Month>12) ) return 0;
     if( Month==2) return 28+isLeapYear();
     return 30 + ((Table>>(Month-1))&1); }

   void incrDate(int8_t Days=1)                     // increment YY:MM:DD
   { uint8_t DaysPerMonth = MonthDays();
     Day+=Days; if(Day<=DaysPerMonth) return;
     Day-=DaysPerMonth; Month++; if(Month<=12) return;
     Month=1; Year++; }

   void decrDate(void)                             // decrement YY:MM:DD
   { if(Day>1) { Day--; return; }
     if(Month>1) { Month--; Day=MonthDays(); return; }
     Year--; Month=12; Day=MonthDays(); return; }

   void incrTimeDate(void) { if(incrTime()) incrDate(); }
   void decrTimeDate(void) { if(decrTime()) decrDate(); }

   void copyTime(GPS_Time &RefTime)                // copy HH:MM:SS.SSS from another record
   { mSec    = RefTime.mSec;
     Sec     = RefTime.Sec;
     Min     = RefTime.Min;
     Hour    = RefTime.Hour; }

   void copyDate(GPS_Time &RefTime)                // copy YY:MM:DD from another record
   { Day     = RefTime.Day;
     Month   = RefTime.Month;
     Year    = RefTime.Year; }

   void copyTimeDate(GPS_Time &RefTime) { copyTime(RefTime); copyDate(RefTime); }

   uint32_t msDayTime(void) const                                                          // [ms]
   { return getDayTime()*1000+mSec; }

   uint32_t getDayTime(void) const                                                         // [sec] time within the day
   { return Times60((uint32_t)(Times60((uint16_t)Hour) + Min)) + Sec; } // this appears to save about 100 bytes of code on STM32
     // return (uint32_t)Hour*SecsPerHour + (uint16_t)Min*SecsPerMin + Sec; }              // compared to this line

   uint32_t getUnixTime(void) const                         // return the Unix timestamp (tested 2000-2037)
   { uint16_t Days = DaysSinceYear2000() + DaysSimce1jan();
     return Times60(Times60(Times24((uint32_t)(Days+10957)))) + getDayTime(); }

   uint16_t getFatDate(void) const                          // return date in FAT format
   { return ((uint16_t)(Year+20)<<9) | ((uint16_t)Month<<5) | Day; }

   uint16_t getFatTime(void) const                          // return time in FAT format
   { return ((uint16_t)Hour<<11) | ((uint16_t)Min<<5) | (Sec>>1); }

   uint32_t getFatDateTime(void) const                      // return date+time in FAT format
   { return ((uint32_t)getFatDate()<<16) | getFatTime(); }

   void setUnixTime(uint32_t Time)                          // works except for the 1.1.2000
   { uint32_t Days = Time/SecsPerDay;                       // [day] since 1970
     uint32_t DayTime = Time - Days*SecsPerDay;             // [sec] time-of-day
     Hour  = DayTime/SecsPerHour; DayTime -= (uint32_t)Hour*SecsPerHour;  //
     Min   = DayTime/SecsPerMin;  DayTime -= (uint16_t)Min*SecsPerMin;
     Sec   = DayTime;
     mSec=0;
     Days -= 10957+1;                                       // [day] since 2000 minus 1 day
     Year  = (Days*4)/((365*4)+1);                          // [year] since 1970
     Days -= 365*Year + (Year/4);
     Month = Days/31;
     Day   = Days-(uint16_t)Month*31+1; Month++;
     uint32_t CheckTime = getUnixTime();
     if(CheckTime<Time) incrDate((Time-CheckTime)/SecsPerDay); }

  void setUnixTime_ms(uint64_t Time_ms)
  { uint32_t Time=Time_ms/1000;
    setUnixTime(Time);
    mSec = Time_ms-(uint64_t)Time*1000; }

  uint64_t getUnixTime_ms(void) const
  { return (uint64_t)getUnixTime()*1000 + mSec; }

   int8_t ReadTime(const char *Value)                         // read the Time field: HHMMSS.sss and check if it is a new one or the same one
   { int8_t Prev; int8_t Same=1;
     Prev=Hour;
     Hour=Read_Dec2(Value);  if(Hour<0) return -1;            // read hour (two digits), return when invalid
     if(Prev!=Hour) Same=0;
     Value+=2;
     if(Value[0]==':') Value++;
     Prev=Min;
     Min=Read_Dec2(Value); if(Min<0)  return -1;            // read minute (two digits), return when invalid
     if(Prev!=Min) Same=0;
     Value+=2;
     if(Value[0]==':') Value++;
     Prev=Sec;
     Sec=Read_Dec2(Value); if(Sec<0)  return -1;            // read second (two digits), return when invalid
     Value+=2;
     if(Prev!=Sec) Same=0;
     int16_t mPrev = mSec;
     if(Value[0]=='.')                                        // is there a fraction
     { uint16_t Frac=0; int8_t Len=Read_UnsDec(Frac, Value+1); if(Len<1) return -1; // read the fraction, return when invalid
            if(Len==1) mSec = Frac*100;
       else if(Len==2) mSec = Frac*10;
       else if(Len==3) mSec = Frac;
       else if(Len==4) mSec = Frac/10;
       else return -1; }
     if(mPrev!=mSec) Same=0;                                  // return 0 when time is valid but did not change
     return Same; }                                           // return 1 when time did not change (both RMC and GGA were for same time)

   int8_t ReadDate(const char *Param)                         // read the field DDMMYY
   { Day=Read_Dec2(Param);   if(Day<0)   return -1;           // read calendar year (two digits - thus need to be extended to four)
     Param+=2;
     if(Param[0]=='/') Param++;
     Month=Read_Dec2(Param); if(Month<0) return -1;           // read calendar month
     Param+=2;
     if(Param[0]=='/') Param++;
     Year=Read_Dec2(Param);  if(Year<0)  return -1;           // read calendar day
     return 0; }                                              // return 0 when field valid and was read correctly

  private:

   static const uint32_t SecsPerMin  =       60;
   static const uint32_t SecsPerHour =    60*60;
   static const uint32_t SecsPerDay  = 24*60*60;
   static const uint32_t mSecsPerDay = 24*60*60*1000;

   uint8_t isLeapYear(void) const { return (Year&3)==0; }

#ifdef __AVR__
   int16_t DaysSimce1jan(void) const
   { static const uint8_t DaysDiff[12] PROGMEM = { 0, 3, 3, 6, 8, 11, 13, 16, 19, 21, 24, 26 } ;
     uint16_t Days = (uint16_t)(Month-1)*28 + pgm_read_byte(DaysDiff+(Month-1)) + Day - 1;
     if(isLeapYear() && (Month>2) ) Days++;
     return Days; }
#else
   int16_t DaysSimce1jan(void) const  //  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
   { static const uint8_t DaysDiff[12] = { 0,  3,  3,  6,  8, 11, 13, 16, 19, 21, 24, 26 } ;
     uint16_t Days = (uint16_t)(Month-1)*28 + DaysDiff[Month-1] + Day - 1;
     if(isLeapYear() && (Month>2) ) Days++;
     return Days; }
#endif

   uint16_t DaysSinceYear2000(void) const
   { uint16_t Days = 365*Year;
     if(Year>0) Days += ((Year-1)>>2)+1;
     return Days; }

   template <class Type>
     static Type Times60(Type X) { return ((X<<4)-X)<<2; }

   template <class Type>
     static Type Times28(Type X) { X+=(X<<1)+(X<<2); return X<<2; }

   template <class Type>
     static Type Times24(Type X) { X+=(X<<1);        return X<<3; }

} ;

class GPS_Position: public GPS_Time
{ public:

  union
  { uint32_t Flags;             // bit #0 = GGA and RMC had same Time
    struct
    { bool hasGPS   :1;         // all required GPS information has been supplied (but this is not the GPS lock status)
      bool hasTime  :1;         // Time has been supplied
      bool hasDate  :1;         // Date has been supplied
      bool hasRMC   :1;         // GxRMC has been supplied
      bool hasGGA   :1;         // GxGGA has been supplied
      bool hasGSA   :1;         // GxGSA has been supplied
      bool hasGSV   :1;
      bool isReady  :1;         // is ready for the following treaement
      bool Sent     :1;         // has been transmitted
      bool hasBaro  :1;         // pressure sensor information: pressure, standard pressure altitude, temperature
      bool hasHum   :1;         // has humidity (not all baro have humiditiy)
      bool hasClimb :1;         // has climb-rate computed or measured
      bool hasTurn  :1;         //
      bool hasAccel :1;         //
      bool hasIAS   :1;         // has Indicated Air Speed calculated/measured
      bool hasAHRS  :1;         // has Attitude Heading Reference System data
      bool InFlight :1;         // take-off and landing detection
    } ;
  } ;

   // uint16_t SatSNRsum;          // sum of cSNR from GPGSV
   // uint8_t SatSNRcount;         // count of satellites from GPGSV

   int8_t FixQuality;           // 0 = none, 1 = GPS, 2 = Differential GPS (can be WAAS)
   int8_t FixMode;              // 0 = not set (from GSA) 1 = none, 2 = 2-D, 3 = 3-D
   int8_t Satellites;           // number of active satellites

   uint8_t PDOP;                // [0.1] dilution of precision
   uint8_t HDOP;                // [0.1] horizontal dilution of precision
   uint8_t VDOP;                // [0.1] vertical dilution of precision

   int16_t Speed;               // [0.1 m/s] speed-over-ground
   int16_t Heading;             // [0.1 deg]  heading-over-ground

   uint16_t AirSpeed;           // [0.1m/s] Indicated Air Speed
    int16_t Pitch;              // [] AHRS Attitude
    int16_t Roll;               // [] AHRS Inclination

   int16_t ClimbRate;           // [0.1 meter/sec)
   int16_t TurnRate;            // [0.1 deg/sec]

   int32_t Altitude;            // [0.1 meter] height above Geoid (sea level)

   int32_t Latitude;            // [0.0001/60 deg] about 0.18m accuracy (to convert to u-Blox GPS 1e-7deg units mult by 50/3)
   int32_t Longitude;           // [0.0001/60 deg]
   int16_t GeoidSeparation;     // [0.1 meter] difference between Geoid and Ellipsoid
  uint16_t LatitudeCosine;      // [2^-12] Latitude cosine for distance calculation

  uint32_t Pressure;            // [0.25 Pa]   from pressure sensor
   int32_t StdAltitude;         // [0.1 meter] standard pressure altitude (from the pressure sensor and atmosphere calculator)
   int16_t Temperature;         // [0.1 degC]
   int16_t Humidity;            // [0.1%]      relative humidity
   int16_t LongAccel;           // [0.1m/s^2]  acceleration along the track
   uint16_t Seq;                // sequencial number to track GPS positions in a pipe

   // uint16_t LockTime;           // [sec] Time since lock
   uint8_t NMEAframes;          // count the correct NMEA frames
   uint8_t NMEAerrors;          // cound the NMEA check-sum errors;

  public:

   GPS_Position() { Clear(); }

   void Clear(void)
   { Flags=0; FixQuality=0; FixMode=0;
     PDOP=0; HDOP=0; VDOP=0;
     // SatSNRsum=0; SatSNRcount=0;
     setDefaultDate(); setDefaultTime();
     Latitude=0; Longitude=0; LatitudeCosine=3000;
     Altitude=0; GeoidSeparation=0;
     Speed=0; Heading=0; ClimbRate=0; TurnRate=0;
     Temperature=0; Pressure=0; StdAltitude=0; Humidity=0;
     NMEAframes=0; NMEAerrors=0; }

   bool isValid(void) const                          // is GPS data is valid = GPS lock
   { if(!isTimeValid()) return 0;                    // is GPS time valid/present ?
     if(!isDateValid()) return 0;                    // is GPS date valid/present ?
     if(FixQuality==0)  return 0;                    // Fix quality must be 1=GPS or 2=DGPS
     if(FixMode==1)     return 0;                    // if GSA says "no lock" (when GSA is not there, FixMode=0)
     if(Satellites<=0)  return 0;                    // if number of satellites none or invalid
     return 1; }

   void copyDOP(GPS_Position &RefPos)
   { HDOP = RefPos.HDOP;
     VDOP = RefPos.VDOP;
     PDOP = RefPos.PDOP;
     FixMode = RefPos.FixMode; }

   void copyBaro(GPS_Position &RefPos, int16_t dTime=0)
   { if(!RefPos.hasBaro) { hasBaro=0; return; }
     StdAltitude = RefPos.StdAltitude;
     Pressure    = RefPos.Pressure;
     Temperature = RefPos.Temperature;
     Humidity    = RefPos.Humidity;
     if(dTime)
     { int32_t dAlt = calcAltitudeExtrapolation(dTime);                         // [0.1m]
       StdAltitude += dAlt;                                                     // [0.1m]
       if(Pressure) Pressure += 4000*dAlt/Atmosphere::PressureLapseRate(Pressure/4, Temperature); } // [0.25Pa] ([Pa], [0.1degC])
     hasBaro=1; }

#ifndef __AVR__ // there is not printf() with AVR
   void PrintDateTime(void) const { printf("%02d.%02d.%04d %02d:%02d:%06.3f", Day, Month, 2000+Year, Hour, Min, Sec+0.001*mSec ); }
   void PrintTime(void)     const { printf("%02d:%02d:%06.3f", Hour, Min, Sec+0.001*mSec ); }

   int PrintDateTime(char *Out) const { return sprintf(Out, "%02d.%02d.%04d %02d:%02d:%02d.%03d", Day, Month, Year, Hour, Min, Sec, mSec ); }
   int PrintTime(char *Out)     const { return sprintf(Out, "%02d:%02d:%02d.%03d", Hour, Min, Sec, mSec ); }

   void Print(void) const
   { printf("Time/Date: "); PrintDateTime();
     printf(" FixQual/Mode=%d/%d: %d sats DOP/H/V=%3.1f/%3.1f/%3.1f", FixQuality, FixMode, Satellites, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     printf(" Lat/Lon/Alt = [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m LatCos=%+6.3f", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation, 1.0/(1<<12)*LatitudeCosine);
     printf(" Speed/Heading = %3.1fm/s %05.1fdeg", 0.1*Speed, 0.1*Heading);
     printf(" Climb = %+5.1fm/s", 0.1*ClimbRate);
     printf(" Turn = %+5.1fdeg/s", 0.1*TurnRate);
     printf(" %d(%d)\n", NMEAframes, NMEAerrors); }

   int Print(char *Out) const
   { int Len=0;
     Len+=sprintf(Out+Len, "Time/Date = "); Len+=PrintDateTime(Out+Len); printf(" "); // Len+=sprintf(Out+Len, " = %10ld.%02dsec\n", (long int)UnixTime, FracSec);
     Len+=sprintf(Out+Len, "FixQuality/Mode=%d/%d: %d satellites DOP/H/V=%3.1f/%3.1f/%3.1f ", FixQuality, FixMode, Satellites, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     Len+=sprintf(Out+Len, "Lat/Lon/Alt = [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m ", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation);
     Len+=sprintf(Out+Len, "Speed/Heading = %3.1fm/s %05.1fdeg\n", 0.1*Speed, 0.1*Heading);
     return Len; }

   void PrintLine(void) const
   { PrintTime();
     printf(" %d/%d/%02d/%4.1f/%4.1f/%4.1f", FixQuality, FixMode, Satellites, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     printf(" [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation);
     printf(" %4.1fm/s %05.1fdeg", 0.1*Speed, 0.1*Heading);
     printf(" %d(%d)\n", NMEAframes, NMEAerrors); }

   int PrintLine(char *Out) const
   { int Len=0; // PrintDateTime(Out);
     Out[Len++]=hasTime?'T':'_';
     Out[Len++]=hasGPS ?'G':'_';
     Out[Len++]=hasBaro?'B':'_';
     Out[Len++]=hasHum ?'H':'_';
     Out[Len++]=hasRMC ?'R':'_';
     Out[Len++]=hasGGA ?'G':'_';
     Out[Len++]=hasGSA ?'S':'_';
     Out[Len++]=isValid() ?'V':'_';
     Out[Len++]=isTimeValid() ?'T':'_';
     Out[Len++]=isDateValid() ?'D':'_';

     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, (uint16_t)Hour, 2);
     Out[Len++]=':'; Len+=Format_UnsDec(Out+Len, (uint16_t)Min,  2);
     Out[Len++]=':'; Len+=Format_UnsDec(Out+Len, (uint16_t)Sec,  2);
     Out[Len++]='.'; Len+=Format_UnsDec(Out+Len, (uint16_t)mSec, 3);
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, (uint16_t)FixQuality);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, (uint16_t)FixMode);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, (uint16_t)Satellites, 2);
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, PDOP, 2, 1);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, HDOP, 2, 1);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, VDOP, 2, 1);
     Out[Len++]=' ';
     Out[Len++]='['; Len+=Format_SignDec(Out+Len, Latitude/6, 7, 5);
     Out[Len++]=','; Len+=Format_SignDec(Out+Len, Longitude/6, 8, 5);
     Out[Len++]=']'; Out[Len++]='d'; Out[Len++]='e'; Out[Len++]='g';
     Out[Len++]=' '; Len+=Format_SignDec(Out+Len, Altitude, 4, 1); Out[Len++]='m';
     Out[Len++]='/'; Len+=Format_SignDec(Out+Len, GeoidSeparation, 4, 1); Out[Len++]='m';
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Speed,     2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Heading,   4, 1); Out[Len++]='d'; Out[Len++]='e'; Out[Len++]='g';
     Out[Len++]=' '; Len+=Format_SignDec(Out+Len, ClimbRate, 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' '; Len+=Format_SignDec(Out+Len, TurnRate, 2, 1); Out[Len++]='d'; Out[Len++]='e'; Out[Len++]='g'; Out[Len++]='/'; Out[Len++]='s';
     if(hasBaro)
     { Out[Len++]=' '; Len+=Format_SignDec(Out+Len, Temperature, 2, 1); Out[Len++]='C';
       Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Pressure/4        ); Out[Len++]='P'; Out[Len++]='a';
       Out[Len++]=' '; Len+=Format_SignDec(Out+Len, StdAltitude, 2, 1); Out[Len++]='m'; }
     Out[Len++]='\n'; Out[Len++]=0; return Len; }
#endif // __AVR__

   int8_t ReadUBX(UBX_RxMsg &RxMsg)
   { if(!RxMsg.isNAV()) return 0;
     if(RxMsg.isNAV_TIMEUTC()) return ReadUBX_NAV_TIMEUTC(RxMsg);
     if(RxMsg.isNAV_POSLLH() ) return ReadUBX_NAV_POSLLH(RxMsg);
     if(RxMsg.isNAV_SOL()    ) return ReadUBX_NAV_SOL(RxMsg);
     return 0; }

   int8_t ReadUBX_NAV_TIMEUTC(UBX_RxMsg &RxMsg)
   { UBX_NAV_TIMEUTC *TIMEUTC = (UBX_NAV_TIMEUTC *)(RxMsg.Byte);
     Year  = TIMEUTC->year-2000;
     Month = TIMEUTC->month;
     Day   = TIMEUTC->day;
     Hour  = TIMEUTC->hour;
     Min   = TIMEUTC->min;
     Sec   = TIMEUTC->sec;
     if(TIMEUTC->nano<0) { decrTimeDate(); TIMEUTC->nano+=1000000000; }
     mSec  =  (TIMEUTC->nano+500000)/1000000;             // [ms]
     if(mSec>=1000) { incrTimeDate(); mSec-=1000; }
     hasTime = (TIMEUTC->valid&0x02)!=0;
     return hasTime; }

   int8_t ReadUBX_NAV_POSLLH(UBX_RxMsg &RxMsg)
   { UBX_NAV_POSLLH *POSLLH = (UBX_NAV_POSLLH *)(RxMsg.Byte);
     Latitude        =  3*(int64_t)POSLLH->lat/50;
     Longitude       =  3*(int64_t)POSLLH->lon/50;
     Altitude        =  POSLLH->hMSL/100;
     GeoidSeparation = (POSLLH->height-POSLLH->hMSL)/100;
     hasGPS = 1;
     return 1; }

   int8_t ReadUBX_NAV_SOL(UBX_RxMsg &RxMsg)
   { UBX_NAV_SOL *SOL = (UBX_NAV_SOL *)(RxMsg.Byte);
     FixMode    = SOL->gpsFix;
     FixQuality = FixMode>=2;
     PDOP       = SOL->PDOP/10;
     Satellites = SOL->numSV;
     return 1; }

   int8_t ReadNMEA(NMEA_RxMsg &RxMsg)
   { // if(RxMsg.isGPGGA()) return ReadGGA(RxMsg);
     if(RxMsg.isGxGGA()) return ReadGGA(RxMsg);
     // if(RxMsg.isGPRMC()) return ReadRMC(RxMsg);
     if(RxMsg.isGxRMC()) return ReadRMC(RxMsg);
     // if(RxMsg.isGPGSA()) return ReadGSA(RxMsg);
     if(RxMsg.isGxGSA()) return ReadGSA(RxMsg);
     if(RxMsg.isPGRMZ()) return ReadPGRMZ(RxMsg); // (pressure) altitude
     // if(RxMsg.isGxGSV()) return ReadGSV(RxMsg);
     return 0; }

   int8_t ReadNMEA(const char *NMEA)
   { int Err=0;
     Err=ReadGGA(NMEA); if(Err!=(-1)) return Err;
     Err=ReadGSA(NMEA); if(Err!=(-1)) return Err;
     Err=ReadRMC(NMEA); if(Err!=(-1)) return Err;
     // Err=ReadGSV(NMEA); if(Err!=(-1)) return Err;
     return 0; }

   int8_t ReadPGRMZ(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms<3) return -2;
     int8_t Ret=Read_Float1(StdAltitude, (const char *)(RxMsg.ParmPtr(0)));
     if(Ret<=0) return -1;
     char Unit=RxMsg.ParmPtr(1)[0];
     hasBaro=1; Pressure=0; Temperature=0;
     if(Unit=='m' || Unit=='M') return 1;
     if(Unit!='f' && Unit!='F') return -1;
     StdAltitude = (StdAltitude*312+512)>>10;
     return 1; }

   int8_t ReadGGA(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms<14) return -2;                                                        // no less than 14 paramaters
     hasGPS = ReadTime((const char *)RxMsg.ParmPtr(0))>0;                                 // read time and check if same as the RMC says
     FixQuality =Read_Dec1(*RxMsg.ParmPtr(5)); if(FixQuality<0) FixQuality=0;             // fix quality: 0=invalid, 1=GPS, 2=DGPS
     Satellites=Read_Dec2((const char *)RxMsg.ParmPtr(6));                                // number of satellites
     if(Satellites<0) Satellites=Read_Dec1(RxMsg.ParmPtr(6)[0]);
     if(Satellites<0) Satellites=0;
     ReadHDOP((const char *)RxMsg.ParmPtr(7));                                            // horizontal dilution of precision
     ReadLatitude(*RxMsg.ParmPtr(2), (const char *)RxMsg.ParmPtr(1));                     // Latitude
     ReadLongitude(*RxMsg.ParmPtr(4), (const char *)RxMsg.ParmPtr(3));                    // Longitude
     ReadAltitude(*RxMsg.ParmPtr(9), (const char *)RxMsg.ParmPtr(8));                     // Altitude
     ReadGeoidSepar(*RxMsg.ParmPtr(11), (const char *)RxMsg.ParmPtr(10));                 // Geoid separation
     calcLatitudeCosine();
     return 1; }

   uint8_t WriteGGA(char *GGA)
   { uint8_t Len=0;
     Len+=Format_String(GGA+Len, "$GPGGA,");
     Len+=Format_UnsDec(GGA+Len, (uint16_t)Hour, 2);
     Len+=Format_UnsDec(GGA+Len, (uint16_t)Min, 2);
     Len+=Format_UnsDec(GGA+Len, (uint16_t)Sec, 2);
     GGA[Len++]='.';
     Len+=Format_UnsDec(GGA+Len, (uint16_t)mSec, 3);
     GGA[Len++]=',';
     Len+=Format_Latitude(GGA+Len, Latitude);
     GGA[Len]=GGA[Len-1]; GGA[Len-1]=','; Len++;
     GGA[Len++]=',';
     Len+=Format_Longitude(GGA+Len, Longitude);
     GGA[Len]=GGA[Len-1]; GGA[Len-1]=','; Len++;
     GGA[Len++]=',';
     GGA[Len++]='0'+FixQuality;
     GGA[Len++]=',';
     Len+=Format_UnsDec(GGA+Len, (uint16_t)Satellites);
     GGA[Len++]=',';
     Len+=Format_UnsDec(GGA+Len, (uint16_t)HDOP, 2, 1);
     GGA[Len++]=',';
     Len+=Format_SignDec(GGA+Len, Altitude, 3, 1);
     GGA[Len++]=',';
     GGA[Len++]='M';
     GGA[Len++]=',';
     Len+=Format_SignDec(GGA+Len, GeoidSeparation, 3, 1);
     GGA[Len++]=',';
     GGA[Len++]='M';
     GGA[Len++]=',';
     GGA[Len++]=',';
     Len += NMEA_AppendCheckCRNL(GGA, Len);
     GGA[Len]=0;
     return Len; }

   int8_t ReadGGA(const char *GGA)
   { if( (memcmp(GGA, "$GPGGA", 6)!=0) && (memcmp(GGA, "$GNGGA", 6)!=0) ) return -1;                                           // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, GGA)<14) return -2;                           // index parameters and check the sum
     hasGPS = ReadTime(GGA+Index[0])>0;
     FixQuality =Read_Dec1(GGA[Index[5]]); if(FixQuality<0) FixQuality=0;                 // fix quality
     Satellites=Read_Dec2(GGA+Index[6]);                                                  // number of satellites
     if(Satellites<0) Satellites=Read_Dec1(GGA[Index[6]]);
     if(Satellites<0) Satellites=0;
     ReadHDOP(GGA+Index[7]);                                                              // horizontal dilution of precision
     ReadLatitude( GGA[Index[2]], GGA+Index[1]);                                          // Latitude
     ReadLongitude(GGA[Index[4]], GGA+Index[3]);                                          // Longitude
     ReadAltitude(GGA[Index[9]], GGA+Index[8]);                                           // Altitude
     ReadGeoidSepar(GGA[Index[11]], GGA+Index[10]);                                       // Geoid separation
     calcLatitudeCosine();
     return 1; }

   int8_t ReadGSA(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms<17) return -1;
     FixMode =Read_Dec1(*RxMsg.ParmPtr(1)); if(FixMode<0) FixMode=0;                      // fix mode
     ReadPDOP((const char *)RxMsg.ParmPtr(14));                                           // total dilution of precision
     ReadHDOP((const char *)RxMsg.ParmPtr(15));                                           // horizontal dilution of precision
     ReadVDOP((const char *)RxMsg.ParmPtr(16));                                           // vertical dilution of precision
     return 1; }

   int8_t ReadGSA(const char *GSA)
   { if( (memcmp(GSA, "$GPGSA", 6)!=0) && (memcmp(GSA, "$GNGSA", 6)!=0) ) return -1;      // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, GSA)<17) return -2;                           // index parameters and check the sum
     FixMode =Read_Dec1(GSA[Index[1]]); if(FixMode<0) FixMode=0;
     ReadPDOP(GSA+Index[14]);
     ReadHDOP(GSA+Index[15]);
     ReadVDOP(GSA+Index[16]);
     return 1; }
/*
   int8_t ReadGSV(NMEA_RxMsg &RxMsg)
   { //
     return 1; }

   int8_t ReadGSV(const char *GSV)
   { if( (memcmp(GSV, "$GPGSV", 6)!=0) && (memcmp(GSV, "$GNGSV", 6)!=0) ) return -1;      // check if the right sequence
     uint8_t Index[24]; if(IndexNMEA(Index, GSV)<20) return -2;                           // index parameters and check the sum
     //
     return 1; }
*/
   int ReadRMC(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms<11) return -2;                                                        // no less than 12 parameters
     hasGPS = ReadTime((const char *)RxMsg.ParmPtr(0))>0;                                 // read time and check if same as the GGA says
     if(ReadDate((const char *)RxMsg.ParmPtr(8))<0) setDefaultDate();                     // date
     ReadLatitude(*RxMsg.ParmPtr(3), (const char *)RxMsg.ParmPtr(2));                     // Latitude
     ReadLongitude(*RxMsg.ParmPtr(5), (const char *)RxMsg.ParmPtr(4));                    // Longitude
     ReadSpeed((const char *)RxMsg.ParmPtr(6));                                           // Speed
     ReadHeading((const char *)RxMsg.ParmPtr(7));                                         // Heading
     calcLatitudeCosine();
     return 1; }

   int8_t ReadRMC(const char *RMC)
   { if( (memcmp(RMC, "$GPRMC", 6)!=0) && (memcmp(RMC, "$GNRMC", 6)!=0) ) return -1;      // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, RMC)<11) return -2;                           // index parameters and check the sum
     hasGPS = ReadTime(RMC+Index[0])>0;
     if(ReadDate(RMC+Index[8])<0) setDefaultDate();
     ReadLatitude( RMC[Index[3]], RMC+Index[2]);
     ReadLongitude(RMC[Index[5]], RMC+Index[4]);
     ReadSpeed(RMC+Index[6]);
     ReadHeading(RMC+Index[7]);
     calcLatitudeCosine();
     return 1; }
/*
   int32_t calcTimeDiff(GPS_Position &RefPos) const
   { int32_t TimeDiff = ((int32_t)Min*6000+(int16_t)Sec*100+FracSec) - ((int32_t)RefPos.Min*6000+(int16_t)RefPos.Sec*100+RefPos.FracSec);
     if(TimeDiff<(-180000)) TimeDiff+=360000;                                               // wrap-around 60min
     else if(TimeDiff>=180000) TimeDiff-=360000;
     return TimeDiff; }                                                                     // [0.01s]
*/
   int16_t calcDifferentials(GPS_Position &RefPos, bool useBaro=1) // calculate climb rate and turn rate with an earlier reference position
   { // ClimbRate=0; hasClimb=0;
     // TurnRate=0;  hasTurn=0;
     // LongAccel=0; hasAccel=0;
     if(RefPos.FixQuality==0) return 0;            // give up if no fix on the reference position
     int16_t TimeDiff = calcTimeDiff(RefPos);      // [ms] time difference between positions
     if(TimeDiff<10) return 0;                     // [ms] give up if smaller than 10ms (as well when negative)
     TurnRate = Heading-RefPos.Heading;            // [0.1deg/s] turn rate
     if(TurnRate>1800) TurnRate-=3600; else if(TurnRate<(-1800)) TurnRate+=3600; // wrap-around
     ClimbRate = Altitude-RefPos.Altitude;         // [0.1m/s] climb rate as altitude difference
     if(useBaro && hasBaro && RefPos.hasBaro && (abs(Altitude-StdAltitude)<2500) ) // if there is baro data then
     { ClimbRate += StdAltitude-RefPos.StdAltitude; // [0.1m/s] on pressure altitude
       ClimbRate = (ClimbRate+1)>>1; }             // take average of the GPS and baro climb rate
     LongAccel = Speed-RefPos.Speed;               // longitual acceleration
     if(TimeDiff==100)                             // [ms] if 0.1sec difference
     { ClimbRate *=10;
       TurnRate  *=10;
       LongAccel *=10; }
     if(TimeDiff==200)                             // [ms] if 0.2sec difference
     { ClimbRate *=5;
       TurnRate  *=5;
       LongAccel *=5; }
     if(TimeDiff==250)                             // [ms] if 0.25sec difference
     { ClimbRate *=4;
       TurnRate  *=4;
       LongAccel *=4; }
     else if(TimeDiff==500)
     { ClimbRate *=2;
       TurnRate  *=2;
       LongAccel *=2; }
     else if(TimeDiff==1000)
     { }
     else if(TimeDiff==2000)
     { ClimbRate = (ClimbRate+1)>>1;
       TurnRate  = ( TurnRate+1)>>1;
       LongAccel = (LongAccel+1)>>1; }
     else if(TimeDiff!=0)
     { ClimbRate = ((int32_t)ClimbRate*1000)/TimeDiff;
       TurnRate  = ((int32_t) TurnRate*1000)/TimeDiff;
       LongAccel = ((int32_t)LongAccel*1000)/TimeDiff; }
     // printf("calcDifferences( , %d) %02d.%03ds hasBaro:%d:%d %4dms %3.1f/%3.1f m %+4.1f m/s\n",
     //         useBaro, Sec, mSec, hasBaro, RefPos.hasBaro, TimeDiff, 0.1*Altitude, 0.1*StdAltitude, 0.1*ClimbRate);
     hasClimb=1; hasTurn=1; hasAccel=1;
     return TimeDiff; } // [ms]

   void Write(MAV_GPS_RAW_INT *MAV) const
   { MAV->time_usec = (int64_t)1000000*getUnixTime()+1000*mSec;  // [usec]
     MAV->lat = ((int64_t)50*Latitude+1)/3;
     MAV->lon = ((int64_t)50*Longitude+1)/3;
     MAV->alt = 100*Altitude;
     MAV->vel = 10*Speed;
     MAV->cog = 10*Heading;;
     MAV->fix_type = 1+FixQuality;
     MAV->eph = 10*HDOP;
     MAV->epv = 10*VDOP;
     MAV->satellites_visible = Satellites; }

   void Read(const MAV_GPS_RAW_INT *MAV, uint64_t UnixTime_ms=0)
   { if(UnixTime_ms) { setUnixTime_ms(UnixTime_ms); hasTime=1; }
     Latitude   = ((int64_t)MAV->lat*3+25)/50;
     Longitude  = ((int64_t)MAV->lon*3+25)/50;
     Altitude   = (MAV->alt+50)/100;               // [0.1m] AMSL
     Heading    = (MAV->cog+5)/10;                 // [0.1deg]
     Speed      = (MAV->vel+5)/10;                 // [0.1m/s]
     HDOP       = (MAV->eph+5)/10;
     VDOP       = (MAV->epv+5)/10;
     Satellites = MAV->satellites_visible;
     FixMode    = MAV->fix_type-1;
     FixQuality = 1;
     hasGPS     = 1; }

   void Read(const MAV_GLOBAL_POSITION_INT *MAV, uint64_t UnixTime_ms=0)
   { if(UnixTime_ms) { setUnixTime_ms(UnixTime_ms); hasTime=1; }
     Latitude   = ((int64_t)MAV->lat*3+25)/50;
     Longitude  = ((int64_t)MAV->lon*3+25)/50;
     Altitude   = (MAV->alt+50)/100;                                                         // [0.1m] AMSL
     ClimbRate  = -MAV->vz/10;                                                               // [0.1m/s]
     Heading    = (uint32_t)((uint16_t)IntAtan2(MAV->vy, MAV->vx)*(uint32_t)450+0x1000)>>13; // [0.1degC]
     Speed      = IntSqrt((int32_t)MAV->vx*MAV->vx+(int32_t)MAV->vy*MAV->vy)/10;             // [0.1m/s]
     FixMode    = 3;
     FixQuality = 1;
     hasGPS     = 1; }

   void Read(const MAV_SCALED_PRESSURE *MAV, uint64_t UnixTime_ms=0)
   { if(UnixTime_ms) { setUnixTime_ms(UnixTime_ms); hasTime=1; }
     Pressure = 100*4*MAV->press_abs;
     Temperature = MAV->temperature/10;
     hasBaro=1; }

   static int32_t getCordic(int32_t Coord) { return ((int64_t)Coord*83399993+(1<<21))>>22; } // [0.0001/60 deg] => [cordic]
   int32_t getCordicLatitude (void) const { return getCordic(Latitude ); }
   int32_t getCordicLongitude(void) const { return getCordic(Longitude); }

   // [deg]  [0.0001/60deg]    [Cordic]   [FANET Cordic]
   //  180    0x066FF300      0x80000000   0x7FFFBC00
   static int32_t getFANETcordic(int32_t Coord) { return ((int64_t)Coord*83399317+(1<<21))>>22; } // [0.0001/60 deg] => [FANET cordic]

   void EncodeAirPos(FANET_Packet &Packet, uint8_t AcftType=1, bool Track=1)
   { int32_t Alt = Altitude; if(Alt<0) Alt=0; else Alt=(Alt+5)/10;
     int32_t Lat = getFANETcordic(Latitude);                                     // Latitude:  [0.0001/60deg] => [cordic]
     int32_t Lon = getFANETcordic(Longitude);                                    // Longitude: [0.0001/60deg] => [cordic]
                             // other, glider, tow, heli, chute, drop, hang, para, powered, jet, UFO, balloon, air, UAV, ground, static
     const uint8_t FNTtype[16] = { 0,     4,     5,   6,     1,     5,    2,   1,     5,      5,   0,    3,      5,   7,    0,     0  } ; // convert aircraft-type from OGN to FANET
     Packet.setAirPos(FNTtype[AcftType&0x0F], Track, Lat, Lon, Alt, (((uint16_t)Heading<<4)+112)/225, Speed, ClimbRate, TurnRate);
     if(hasBaro) { Packet.setQNE((StdAltitude+5)/10); }
   }

   void Encode(GDL90_REPORT &Report) const
   { Report.setAccuracy(9, 9);
     int32_t Lat = getCordicLatitude();                                     // Latitude:  [0.0001/60deg] => [cordic]
     int32_t Lon = getCordicLongitude();                                    // Longitude: [0.0001/60deg] => [cordic]
     int32_t Alt = Altitude;                                                // [0.1m]
     if(hasBaro) Alt = StdAltitude;
     Alt=MetersToFeet(Alt); Alt=(Alt+5)/10;                                 // [feet]
     Report.setLatitude(Lat);
     Report.setLongitude(Lon);
     Report.setAltitude(Alt);
     uint16_t HeadAngle = ((int32_t)Heading<<12)/225;                       // [16-bit cordic] heading angle
     int32_t SpeedKts = (3981*(int32_t)Speed+1024)>>11;                     // [0.1m/s] => [0.1kts]
     Report.setHeading((HeadAngle+0x80)>>8);                                // [8-bit cordic]
     Report.setMiscInd(0x2);                                                //
     Report.setSpeed((SpeedKts+5)/10);                                      // [knot]
     Report.setClimbRate(6*MetersToFeet(ClimbRate));
   }

  void Encode(ADSL_Packet &Packet) const
  { Packet.setAlt((Altitude+GeoidSeparation+5)/10);
    Packet.setLat(Packet.OGNtoFNT(Latitude));
    Packet.setLon(Packet.OGNtoFNT(Longitude));
    Packet.TimeStamp = (Sec*4+mSec/250)&0x3F;
    Packet.setSpeed(((uint32_t)Speed*4+5)/10);
    Packet.setClimb(((int32_t)ClimbRate*8+5)/10);
    // if(hasClimb) Packet.setClimb(((int32_t)ClimbRate*8+5)/10);
    //        else  Packet.clrClimb();
    Packet.setTrack(((uint32_t)Heading*32+112)/225);
    Packet.Integrity[0]=0; Packet.Integrity[1]=0;
    if((FixQuality>0)&&(FixMode>=2))
    { Packet.setHorPrec((HDOP*2+5)/10);
      Packet.setVerPrec((VDOP*3+5)/10); }
  }

  // template <class OGNx_Packet>
  //  void Encode(OGNx_Packet &Packet) const
   void Encode(OGN1_Packet &Packet) const
   { Packet.Position.FixQuality = FixQuality<3 ? FixQuality:3;             //
     if((FixQuality>0)&&(FixMode>=2)) Packet.Position.FixMode = FixMode-2; //
                                 else Packet.Position.FixMode = 0;
     if(PDOP>0) Packet.EncodeDOP(PDOP-10);                                 // encode PDOP from GSA
           else Packet.EncodeDOP(HDOP-10);                                 // or if no GSA: use HDOP
     int8_t ShortTime=Sec;                                                 // the 6-bit time field in the OGN packet
     if(mSec>=500) { ShortTime+=1; if(ShortTime>=60) ShortTime-=60; }      // round to the closest full second
     Packet.Position.Time=ShortTime;                                       // Time
     Packet.EncodeLatitude(Latitude);                                      // Latitude
     Packet.EncodeLongitude(Longitude);                                    // Longitude
     Packet.EncodeSpeed(Speed);                                            // Speed
     Packet.EncodeHeading(Heading);                                        // Heading = track-over-ground
     Packet.EncodeClimbRate(ClimbRate);                                    // Climb rate
     Packet.EncodeTurnRate(TurnRate);                                      // Turn rate
     Packet.EncodeAltitude((Altitude+5)/10);                               // Altitude
     if(hasBaro) Packet.EncodeStdAltitude((StdAltitude+5)/10);             // Pressure altitude
            else Packet.clrBaro();                                         //or no-baro if pressure sensor data not there
   }

/*
  template <class OGNx_Packet>
   void EncodeStatus(OGNx_Packet &Packet) const
   { Packet.Status.ReportType=0;
     int ShortTime=Sec;
     if(FracSec>=50) { ShortTime+=1; if(ShortTime>=60) ShortTime-=60; }
     Packet.Status.Time=ShortTime;
     Packet.Status.FixQuality = FixQuality<3 ? FixQuality:3;
     Packet.Status.Satellites = Satellites<15 ? Satellites:15;
     Packet.EncodeAltitude((Altitude+5)/10);
     if(hasBaro)
     { Packet.EncodeTemperature(Temperature);
       Packet.Status.Pressure = (Pressure+16)>>5; }
     else
     { Packet.Status.Pressure = 0; }
     Packet.Status.Humidity=0;
   }
*/
  template <class OGNx_Packet>
   void EncodeStatus(OGNx_Packet &Packet) const
   { Packet.Status.ReportType=0;
     int ShortTime=Sec;
     if(mSec>=500) { ShortTime+=1; if(ShortTime>=60) ShortTime-=60; }
     Packet.Status.Time=ShortTime;
     Packet.Status.FixQuality = FixQuality<3 ? FixQuality:3;
     Packet.Status.Satellites = Satellites<15 ? Satellites:15;
     Packet.EncodeAltitude((Altitude+5)/10);
     if(hasBaro)
     { Packet.EncodeTemperature(Temperature);
       Packet.Status.Pressure = (Pressure+16)>>5;
       if(hasHum) Packet.EncodeHumidity(Humidity);
             else Packet.clrHumidity(); }
     else
     { Packet.Status.Pressure=0;
       Packet.clrTemperature();
       Packet.clrHumidity(); }
   }

   // uint8_t getFreqPlan(void) const // get the frequency plan from Lat/Lon: 1 = Europe + Africa, 2 = USA/CAnada, 3 = Australia + South America, 4 = New Zeeland
   // { if( (Longitude>=(-20*600000)) && (Longitude<=(60*600000)) ) return 1; // between -20 and 60 deg Lat => Europe + Africa: 868MHz band
   //   if( Latitude<(20*600000) )                                            // below 20deg latitude
   //   { if( ( Longitude>(164*600000)) && (Latitude<(-30*600000)) && (Latitude>(-48*600000)) ) return 4;  // => New Zeeland
   //     return 3; }                                                         // => Australia + South America: upper half of 915MHz band
   //   return 2; }                                                           // => USA/Canada: full 915MHz band

  template <class OGNx_Packet>
   void Decode(const OGNx_Packet &Packet)
  { FixQuality = Packet.Position.FixQuality;
    FixMode = Packet.Position.FixMode+2;
    PDOP = 10+Packet.DecodeDOP();
    HDOP = PDOP; VDOP = PDOP+PDOP/2;
    mSec=0; Sec=Packet.Position.Time;
    Speed = Packet.DecodeSpeed();
    ClimbRate = Packet.DecodeClimbRate();
    TurnRate = Packet.DecodeTurnRate();
    Heading  = Packet.DecodeHeading();                                              // Heading = track-over-ground
    Latitude = Packet.DecodeLatitude();
    Longitude = Packet.DecodeLongitude();
    Altitude = Packet.DecodeAltitude()*10;
    hasBaro=0;
    if(Packet.hasBaro())
    { StdAltitude = Altitude + 10*Packet.getBaroAltDiff();
      hasBaro=1; }
  }

  template <class OGNx_Packet>
   void Encode(OGNx_Packet &Packet, int16_t dTime) const                       // Encode position which is extrapolated by the given fraction of a second
   { Packet.Position.FixQuality = FixQuality<3 ? FixQuality:3;                //
     if((FixQuality>0)&&(FixMode>=2)) Packet.Position.FixMode = FixMode-2;    //
                                 else Packet.Position.FixMode = 0;
     if(PDOP>0) Packet.EncodeDOP(PDOP-10);                                    // encode PDOP from GSA
           else Packet.EncodeDOP(HDOP-10);                                    // or if no GSA: use HDOP
     int32_t Lat, Lon, Alt; int16_t Head;
     calcExtrapolation(Lat, Lon, Alt, Head, dTime);
     int16_t ShortTime=Sec;                                                   // the 6-bit time field in the OGN packet
     dTime += mSec;
     while(dTime>= 500 ) { dTime-=1000; ShortTime++; if(ShortTime>=60) ShortTime-=60; }
     while(dTime<(-500)) { dTime+=1000; ShortTime--; if(ShortTime<  0) ShortTime+=60; }
     Packet.Position.Time=ShortTime;                                          // Time
     Packet.EncodeLatitude(Lat);                                              // Latitude
     Packet.EncodeLongitude(Lon);                                             // Longitude
     Packet.EncodeSpeed(Speed);                                               // Speed
     Packet.EncodeHeading(Head);                                              // Heading = track-over-ground
     Packet.EncodeClimbRate(ClimbRate);                                       // Climb rate
     Packet.EncodeTurnRate(TurnRate);                                         // Turn rate
     Packet.EncodeAltitude((Alt+5)/10);                                       // Altitude
     if(hasBaro) Packet.EncodeStdAltitude((StdAltitude+(Alt-Altitude)+5)/10); // Pressure altitude
            else Packet.clrBaro();                                            //or no-baro if pressure sensor data not there
   }

   void Extrapolate(int32_t dTime)                                            // [ms] extrapolate the position by dTime
   { int16_t dSpeed = ((int32_t)LongAccel*dTime)/1000;
     Speed += dSpeed/2;
     int16_t HeadAngle = ((int32_t)Heading<<12)/225;                          // [cordic] heading angle
     int16_t TurnAngle = (((dTime*TurnRate)/250)<<9)/225;                     // [cordic]
             HeadAngle += TurnAngle/2;
     int32_t LatSpeed = ((int32_t)Speed*Icos(HeadAngle)+0x800)>>12;           // [0.1m/s]
     int32_t LonSpeed = ((int32_t)Speed*Isin(HeadAngle)+0x800)>>12;           // [0.1m/s]
             HeadAngle += TurnAngle-TurnAngle/2;
     Speed += dSpeed-dSpeed/2; if(Speed<0) Speed=0;
     Latitude  += calcLatitudeExtrapolation (dTime, LatSpeed);                //
     Longitude += calcLongitudeExtrapolation(dTime, LonSpeed);                //
     int32_t dAlt = calcAltitudeExtrapolation(dTime);                         // [0.1m]
     Altitude += dAlt;                                                        // [0.1m]
     if(hasBaro)
     { StdAltitude += dAlt;                                                   // [0.1m]
       Pressure += 4000*dAlt/Atmosphere::PressureLapseRate(Pressure/4, Temperature); } // [0.25Pa] ([Pa], [0.1degC])
     Heading += (dTime*TurnRate)/1000;                                        // [0.1deg]
     if(Heading<0) Heading+=3600; else if(Heading>=3600) Heading-=3600;       // [0.1deg]
     int16_t fTime   = mSec+dTime;                                            // [msec]
     while(fTime>=1000) { incrTimeDate(); fTime-=1000; }
     while(fTime<    0) { decrTimeDate(); fTime+=1000; }
     mSec=fTime; }

   // extrapolate GPS position by a fraction of a second
   void calcExtrapolation(int32_t &Lat, int32_t &Lon, int32_t &Alt, int16_t &Head, int32_t dTime) const // [msec]
   { int16_t HeadAngle = ((int32_t)Heading<<12)/225;                         // []
     int16_t TurnAngle = (((dTime*TurnRate)/250)<<9)/225;                    // []
             HeadAngle += TurnAngle;
     int32_t LatSpeed = ((int32_t)Speed*Icos(HeadAngle)+0x800)>>12;                // [0.1m/s]
     int32_t LonSpeed = ((int32_t)Speed*Isin(HeadAngle)+0x800)>>12;                // [0.1m/s]
     Lat = Latitude  + calcLatitudeExtrapolation (dTime, LatSpeed);
     Lon = Longitude + calcLongitudeExtrapolation(dTime, LonSpeed);
     Alt = Altitude  + calcAltitudeExtrapolation(dTime);
     Head = Heading  + (dTime*TurnRate)/1000;
     if(Head<0) Head+=3600; else if(Head>=3600) Head-=3600; }

   int32_t calcAltitudeExtrapolation(int32_t Time)  const                    // [ms]
   { return Time*ClimbRate/1000; }                                           // [0.1m]

   // int32_t calcLatitudeExtrapolation(int32_t Time, int32_t LatSpeed)  const  // [ms] [0.1m/s]
   // { return (Time/10*LatSpeed*177+0x4000)>>15; }                             // [0.1m]

   int32_t calcLatitudeExtrapolation(int32_t Time, int32_t LatSpeed)  const  // [ms] [0.1m/s]
   { return (Time*LatSpeed*283+0x40000)>>19; }                             // [0.1m]

   int32_t calcLongitudeExtrapolation(int32_t Time, int32_t LonSpeed)  const // [ms]
   { int16_t LatCosine = calcLatCosine(calcLatAngle16(Latitude));
     return calcLongitudeExtrapolation(Time, LonSpeed, LatCosine); }

   // int32_t calcLongitudeExtrapolation(int32_t Time, int32_t LonSpeed, int16_t LatCosine)  const // [ms]
   // { return (((Time/10*LonSpeed*177+4)>>3))/LatCosine; }

   int32_t calcLongitudeExtrapolation(int32_t Time, int32_t LonSpeed, int16_t LatCosine)  const // [ms]
   { return (((Time*LonSpeed*283+64)>>7))/LatCosine; }

   // static int32_t calcLatDistance(int32_t Lat1, int32_t Lat2)             // [m] distance along latitude
   // { return ((int64_t)(Lat2-Lat1)*0x2f684bda+0x80000000)>>32; }

   // static int32_t calcLatAngle32(int32_t Lat)                             // convert latitude to 32-bit integer angle
   // { return ((int64_t)Lat*2668799779u+0x4000000)>>27; }

   static int16_t calcLatAngle16(int32_t Lat)                                // convert latitude to 16-bit integer angle
   { return ((int64_t)Lat*1303125+0x80000000)>>32; }

   // static int32_t calcLatCosine(int32_t LatAngle)                         // calculate the cosine of the latitude 32-bit integer angle
   // { return IntSine((uint32_t)(LatAngle+0x40000000)); }

   // static int32_t calcLatCosine(int16_t LatAngle)                         // calculate the cosine of the latitude 16-bit integer angle
   // { return IntSine((uint16_t)(LatAngle+0x4000)); }

   static int16_t calcLatCosine(int16_t LatAngle)
   { int16_t LatCos=Icos(LatAngle);
     if(LatCos<=0) LatCos=1;                                                 // protect against zero as it is used for division
     return LatCos; }

   // int32_t getLatDistance(int32_t RefLatitude) const                      // [m] distance along latitude
   // { return calcLatDistance(RefLatitude, Latitude); }

   // int32_t getLonDistance(int32_t RefLongitude) const                     // [m] distance along longitude
   // { int32_t Dist = calcLatDistance(RefLongitude, Longitude);             //
   //   int16_t LatAngle =  calcLatAngle16(Latitude);
   //   int32_t LatCos = calcLatCosine(LatAngle);
   //   // printf("Latitude=%+d, LatAngle=%04X LatCos=%08X\n", Latitude, (uint16_t)LatAngle, LatCos);
   //   return ((int64_t)Dist*LatCos+0x40000000)>>31; }                      // distance corrected by the latitude cosine

   void calcLatitudeCosine(void)
   { int16_t LatAngle =  calcLatAngle16(Latitude);
       LatitudeCosine = calcLatCosine(LatAngle); }

   int WriteAPRS(char *Out, const char *Call, const char *Icon, uint32_t ID)
   { int Len=0;
     Len+=Format_String(Out+Len, Call);                                       // Call
     Len+=Format_String(Out+Len, ">APRS:/");
     Len+=WriteHHMMSS(Out+Len);                                               // Time
     Out[Len++]='h';
     Len+=WriteIGCcoord(Out+Len, Latitude, 2, "NS");                          // [DDMM.MM] Latitude
     char LatW = Out[Len-2];
     Out[Len-2]=Out[Len-3]; Out[Len-3]=Out[Len-4]; Out[Len-4]='.';
     Out[Len++]=Icon[0];
     Len+=WriteIGCcoord(Out+Len, Longitude, 3, "EW");                         // [DDDMM.MM] Longitude
     char LonW = Out[Len-2];
     Out[Len-2]=Out[Len-3]; Out[Len-3]=Out[Len-4]; Out[Len-4]='.';
     Out[Len++]=Icon[1];
     Len+=Format_UnsDec(Out+Len, Heading/10, 3);                              // [deg] Heading
     Out[Len++]='/';
     Len+=Format_UnsDec(Out+Len, ((uint32_t)Speed*199+512)>>10, 3);           // [kt] speed
     Out[Len++] = '/'; Out[Len++] = 'A'; Out[Len++] = '='; Len+=Format_UnsDec(Out+Len, (MetersToFeet(Altitude)+5)/10, 6); // [feet] altitude
     Out[Len++]=' '; Out[Len++]='!'; Out[Len++]='W'; Out[Len++]=LatW; Out[Len++]=LonW; Out[Len++]='!'; // more accurate Lat/Lon
     Out[Len++]=' '; Out[Len++]='i'; Out[Len++]='d'; Len+=Format_Hex(Out+Len, ID); // ID

     Out[Len++] = ' '; Len+=Format_SignDec(Out+Len, ((int32_t)ClimbRate*10079+256)>>9, 3); Out[Len++] = 'f'; Out[Len++] = 'p'; Out[Len++] = 'm'; // [fpm]
     Out[Len++] = ' '; Len+=Format_SignDec(Out+Len, TurnRate/3, 2, 1); Out[Len++] = 'r'; Out[Len++] = 'o'; Out[Len++] = 't'; // [ROT]

     if(hasBaro)
     { int32_t Alt=(StdAltitude+5)/10;                                       // [m] standard pressure altitude
       if(Alt<0) Alt=0;
       Out[Len++] = ' '; Out[Len++] = 'F'; Out[Len++] = 'L';
       Len+=Format_UnsDec(Out+Len, MetersToFeet((uint32_t)Alt), 5, 2); }     // [feet] "Flight Level"

     uint16_t DOP=PDOP; if(DOP==0) DOP=HDOP;
     uint16_t HorPrec=(DOP*2+5)/10; if(HorPrec>63) HorPrec=63;               // [m]
     uint16_t VerPrec=(DOP*3+5)/10; if(VerPrec>63) VerPrec=63;               // [m]
     Out[Len++] = ' ';  Out[Len++] = 'g'; Out[Len++] = 'p'; Out[Len++] = 's';
     Len+=Format_UnsDec(Out+Len, HorPrec); Out[Len++] = 'x'; Len+=Format_UnsDec(Out+Len, VerPrec);

     Out[Len]=0; return Len; }

   static int WriteIGCcoord(char *Out, int32_t Coord, uint8_t DegSize, const char *SignChar)
   { int Len=0;
     bool Neg = Coord<0; if(Neg) Coord=(-Coord);
     int32_t Deg = Coord/600000;
     Len+=Format_UnsDec(Out+Len, Deg, DegSize);
     Coord-=Deg*600000; Coord/=10;
     Len+=Format_UnsDec(Out+Len, Coord, 5);
     Out[Len++]=SignChar[Neg];
     return Len; }

   int WriteHHMMSS(char *Out) const
   { Format_UnsDec(Out  , Hour, 2);
     Format_UnsDec(Out+2, Min , 2);
     Format_UnsDec(Out+4, Sec , 2);
     return 6; }

   int WriteIGC(char *Out) const                                  // write IGC B-record
   { // if(!isValid()) return 0;
     int Len=0;
     Out[Len++] = 'B';
     if(isTimeValid()) Len+=WriteHHMMSS(Out+Len);                 // if time is valid
                  else Len+=Format_String(Out+Len, "      ");     // or leave empty
     if(isValid())                                                // if position valid
     { Len+=WriteIGCcoord(Out+Len, Latitude, 2, "NS");            // DDMM.MMM latitude
       Len+=WriteIGCcoord(Out+Len, Longitude, 3, "EW");           // DDDMM.MMM longitude
       Out[Len++] = FixMode>2 ? 'A':'V'; }                        // fix mode
     else Len+=Format_String(Out+Len, "                    ");    // is position not valid then leave empty
     if(hasBaro)                                                  // if pressure data is there
     { int32_t Alt = StdAltitude/10;                              // [m] pressure altitude
       if(Alt<0) { Alt = (-Alt); Out[Len++] = '-'; Len+=Format_UnsDec(Out+Len, (uint32_t)Alt, 4); } // -AAAA (when negative)
            else { Len+=Format_UnsDec(Out+Len, (uint32_t)Alt, 5); }                                 // AAAAA
     } else Len+=Format_String(Out+Len, "     ");
     if(isValid())                                                // if position is valid
     { int32_t Alt = (Altitude+GeoidSeparation)/10;               // [m] HAE GPS altitude
       if(Alt<0) { Alt = (-Alt); Out[Len++] = '-'; Len+=Format_UnsDec(Out+Len, (uint32_t)Alt, 4); } // -AAAA (when negative)
            else { Len+=Format_UnsDec(Out+Len, (uint32_t)Alt, 5); }                                 // AAAAA
     } else Len+=Format_String(Out+Len, "     ");
     Out[Len++]='\n'; Out[Len]=0; return Len; }

  private:

   int8_t ReadLatitude(char Sign, const char *Value)
   { int8_t Deg=Read_Dec2(Value); if(Deg<0) return -1;
     int8_t Min=Read_Dec2(Value+2); if(Min<0) return -1;
     if(Value[4]!='.') return -1;
     int16_t FracMin=Read_Dec4(Value+5); if(FracMin<0) return -1;
     // printf("Latitude: %c %02d %02d %04d\n", Sign, Deg, Min, FracMin);
     Latitude = (int16_t)Deg*60 + Min;
     Latitude = Latitude*(int32_t)10000 + FracMin;
     // printf("Latitude: %d\n", Latitude);
     if(Sign=='S') Latitude=(-Latitude);
     else if(Sign!='N') return -1;
     // printf("Latitude: %d\n", Latitude);
     return 0; }                                    // Latitude units: 0.0001/60 deg

   int8_t ReadLongitude(char Sign, const char *Value)
   { int16_t Deg=Read_Dec3(Value); if(Deg<0) return -1;
     int8_t Min=Read_Dec2(Value+3); if(Min<0) return -1;
     if(Value[5]!='.') return -1;
     int16_t FracMin=Read_Dec4(Value+6); if(FracMin<0) return -1;
     Longitude = (int16_t)Deg*60 + Min;
     Longitude = Longitude*(int32_t)10000 + FracMin;
     if(Sign=='W') Longitude=(-Longitude);
     else if(Sign!='E') return -1;
     return 0; }                                    // Longitude units: 0.0001/60 deg

   int8_t ReadAltitude(char Unit, const char *Value)
   { if(Unit!='M') return -1;
     return Read_Float1(Altitude, Value); }          // Altitude units: 0.1 meter

   int8_t ReadGeoidSepar(char Unit, const char *Value)
   { if(Unit!='M') return -1;
     return Read_Float1(GeoidSeparation, Value); }   // GeoidSepar units: 0.1 meter

   int8_t ReadSpeed(const char *Value)
   { int32_t Knots;
     if(Read_Float1(Knots, Value)<1) return -1;      // Speed: 0.1 knots
     Speed=(527*Knots+512)>>10; return 0; }          // convert speed to 0.1 meter/sec

   int8_t ReadHeading(const char *Value)
   { return Read_Float1(Heading, Value); }           // Heading units: 0.1 degree

   int8_t ReadPDOP(const char *Value)
   { int16_t DOP;
     if(Read_Float1(DOP, Value)<1) return -1;
     if(DOP<10) DOP=10;
     else if(DOP>255) DOP=255;
     PDOP=DOP; return 0; }

   int ReadHDOP(const char *Value)
   { int16_t DOP;
     if(Read_Float1(DOP, Value)<1) return -1;
     if(DOP<10) DOP=10;
     else if(DOP>255) DOP=255;
     HDOP=DOP; return 0; }

   int ReadVDOP(const char *Value)
   { int16_t DOP;
     if(Read_Float1(DOP, Value)<1) return -1;
     if(DOP<10) DOP=10;
     else if(DOP>255) DOP=255;
     VDOP=DOP; return 0; }

  public:

   int8_t static IndexNMEA(uint8_t Index[20], const char *Seq) // index parameters and verify the NMEA checksum
   { int8_t Ptr=0;
     uint8_t Check=0;
     if(Seq[Ptr]!='$') return -1;                              // first chat. must be dollar sign
     Ptr++;
     for( ; Ptr<=6; Ptr++)                                     // go through the sentence name
     { if(Seq[Ptr]==',') break;                                // stop at comma
       Check^=Seq[Ptr]; }                                      // take char. to checksum
     if(Seq[Ptr]!=',') return -1;                              // comma after the sentence name
     Check^=Seq[Ptr++];                                        // take comma to the checksum
     Index[0]=Ptr; int8_t Params=1;                            // first parameter
     for( ; ; )
     { char ch=Seq[Ptr++]; if(ch<' ') return -1;               // go through the chars
       if(ch=='*') break;                                      // break at star (check-sum should follow)
       Check^=ch;                                              // get chars to the checksum
       if(ch==',') { Index[Params++]=Ptr; }                    // if comma then counr next parameter
     }
     if(Seq[Ptr++]!=HexDigit(Check>>4)  ) return -2;           // verify checksum
     if(Seq[Ptr++]!=HexDigit(Check&0x0F)) return -2;
     // printf("%s => [%d]\n", Seq, Params);
     return Params; }

} ;

#endif // of __OGN_H__

