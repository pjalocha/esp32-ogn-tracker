#ifndef __OGN2_H__
#define __OGN2_H__

#include <stdio.h>

#include <string.h>
#include <stdint.h>

#include "intmath.h"

#include "ognconv.h"

#include "bitcount.h"
#include "nmea.h"
#include "mavlink.h"

#include "format.h"

// uint32_t OGN2_SYNC = 0xF56D3738;

                           // the packet description here is how it look on the little-endian CPU before sending it to the RF chip
                           // nRF905, CC1101, SPIRIT1, RFM69 chips actually reverse the bit order within every byte
                           // thus on the air the bits appear MSbit first for every byte transmitted

class OGN2_Packet          // Packet structure for the OGN tracker
{ public:

   static const int Words =  5;
   static const int Bytes = 20;

 union
 { uint32_t HeaderWord;    //    ECNR POTT AAAA AAAA AAAA AAAA AAAA AAAA
                           // E=Emergency, C=enCrypt/Custom, R=Relay, P=Parity, O=Other, TT=address Type, AA..=Address:24-bit
                           // When enCrypt/Custom is set the data (position or whatever) can only be decoded by the owner
                           // This option is indented to pass any type of custom data not foreseen otheriwse

   struct
   { unsigned int Address    :24; // aircraft address
     unsigned int AddrType   : 2; // address type: 0 = random, 1 = ICAO, 2 = FLARM, 3 = OGN
     unsigned int Relay      : 1; // 0 = direct packet, 1 = relayed packet
     unsigned int Parity     : 1; // parity takes into account bits 0..27 thus only the 28 lowest bits
     unsigned int NonPos     : 1; // 0 = position packet, 1 = other information like status
     // unsigned int Auth       : 2; // Authentication: 00 = no auth. 01 = auth. this packet, 10/11 = auth response
                                  // Auth:NonPos: 000 = position, 010 = position, to be followed by crypto-response,
                                  //              101 = response #0, 111 = response #1
                                  //              001 = non-position: status, info, etc.
                                  //              100 = ???, 110 = ???, 011 = ???
     unsigned int NonOGN     : 1; // 0 = OGN packet, 1 = other systems, like MAVlink
     unsigned int Encrypted  : 1; // packet is encrypted
     unsigned int Emergency  : 1; // aircraft in emergency (not used for now)
   } Header ;

 } ;

 union
 { uint32_t Data[4];

   struct
   { unsigned int    AcftType: 4; // [0..15]          // type of aircraft: 1 = glider, 2 = towplane, 3 = helicopter, ...
     unsigned int     Heading:10; // [0.35deg]        // Ground track in cordic units
     unsigned int   ClimbRate: 9; // [0.1m/s] VR
     unsigned int BaroAltDiff: 9; // [m]              // lower 8 bits of the altitude difference between baro and GPS

     unsigned int    Altitude:14; // [m] VR           // Altitude
     unsigned int       Speed:10; // [0.1m/s] VR      // Ground speed
     unsigned int    TurnRate: 8; // [0.1deg/s] VR    // Ground heading (track) rate

     unsigned int    Latitude:24; // [1.2m]           // Latitude in cordic units
     unsigned int        Time: 6; // [sec]            // time, just second thus ambiguity every every minute
     unsigned int  FixQuality: 2; //                  // 0 = no fix, 1 = GPS, 2 = diff. GPS, 3 = other

     unsigned int   Longitude:25; // [1.2m]           // Longitude in cordic units
     unsigned int         DOP: 6; //                  // GPS Dilution of Precision
     unsigned int     FixMode: 1; // [2-D/3-D]
   } Position;


   struct
   {
     unsigned int ReportType: 4; // [0]                 // 0 for the status report
     unsigned int TxPower   : 4; // [dBm]               // RF trancmitter power
     unsigned int Firmware  : 8; // [ ]                 // firmware version
     unsigned int Hardware  : 8; // [ ]                 // hardware version
     unsigned int Voltage   : 8; // [1/64V] VR          // supply voltager

     unsigned int Altitude  :14; // [m] VR              // same as in the position packet
     unsigned int Pressure  :14; // [0.08hPa]           // barometric pressure
     unsigned int Satellites: 4; // [ ]

     unsigned int Pulse     : 8; // [bpm]               // pilot: heart pulse rate
     unsigned int Oxygen    : 7; // [%]                 // pilot: oxygen level in the blood
     // unsigned int FEScurr   : 5; // [A]                 // 
     unsigned int SatSNR    : 5; // [dB]                // average SNR of GPS signals
     unsigned int RxRate    : 4; // [/min]              // log2 of received packet rate
     unsigned int Time      : 6; // [sec]               // same as in the position packet
     unsigned int FixQuality: 2;

     unsigned int AudioNoise: 8; // [dB]                //
     unsigned int RadioNoise: 8; // [dBm]               // noise seen by the RF chip
     unsigned int Temperature:8; // [0.1degC] VR        // temperature by the baro or RF chip
     unsigned int Humidity  : 8; // [%]                 // humidity
   } Status ;

   union
   {      uint8_t Byte[16];
     struct
     {    uint8_t ReportType: 4; // [1]                 // 1 for the Info packets
          uint8_t DataChars:  4; // [int] number of characters in the packed string
          uint8_t Data[14];      // [16x7bit]packed string of 16-char: 7bit/char
          uint8_t Check;         // CRC check
     } ;
   } Info;

 } ;

   uint8_t  *Byte(void) const { return (uint8_t  *)&HeaderWord; } // packet as bytes
   uint32_t *Word(void) const { return (uint32_t *)&HeaderWord; } // packet as words

   static const uint8_t InfoParmNum = 14; // [int]  number of info-parameters and their names
   static const char *InfoParmName(uint8_t Idx) { static const char *Name[InfoParmNum] =
                                                  { "Pilot", "Manuf", "Model", "Type", "SN", "Reg", "ID", "Class",
                                                    "Task" , "Base" , "ICE"  , "PilotID", "Hard", "Soft" } ;
                                                  return Idx<InfoParmNum ? Name[Idx]:0; }

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX\n",
             (long int)HeaderWord, (long int)Data[0], (long int)Data[1],
             (long int)Data[2], (long int)Data[3] ); }

   void DumpBytes(void) const
   { for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { printf(" %02X", Byte()[Idx]); }
     printf("\n"); }

   int WriteDeviceStatus(char *Out)
   { return sprintf(Out, " h%02X v%02X %dsat/%d %ldm %3.1fhPa %+4.1fdegC %3.1f%% %4.2fV %d/%+4.1fdBm %d/min",
             Status.Hardware, Status.Firmware, Status.Satellites, Status.FixQuality, (long int)DecodeAltitude(),
             0.08*Status.Pressure, 0.1*DecodeTemperature(), 0.1*DecodeHumidity(),
             (1.0/64)*DecodeVoltage(), Status.TxPower+4, -0.5*Status.RadioNoise, (1<<Status.RxRate)-1 );
   }

   int WriteDeviceInfo(char *Out)
   { int Len=0;
     char Value[16];
     uint8_t InfoType;
     uint8_t Idx=0;
     for( ; ; )
     { uint8_t Chars = readInfo(Value, InfoType, Idx);
       if(Chars==0) break;
       if(InfoType<InfoParmNum)
       { Len += sprintf(Out+Len, " %s=%s", InfoParmName(InfoType), Value); }
       else
       { Len += sprintf(Out+Len, " #%d=%s", InfoType, Value); }
     Idx+=Chars; }
     Out[Len]=0; return Len; }

   void Print(void) const
   { if(!Header.NonPos) { PrintPosition(); return; }
     if(Status.ReportType==0) { PrintDeviceStatus(); return; }
   }

   void PrintDeviceStatus(void) const
   { printf("%c:%06lX R%c%c %02ds:",
             '0'+Header.AddrType, (long int)Header.Address, '0'+Header.Relay, Header.Emergency?'E':' ', Status.Time);
     printf(" h%02X v%02X %dsat/%d %ldm %3.1fhPa %+4.1fdegC %3.1f%% %4.2fV Tx:%ddBm Rx:%+4.1fdBm %d/min",
             Status.Hardware, Status.Firmware, Status.Satellites, Status.FixQuality, (long int)DecodeAltitude(),
             0.08*Status.Pressure, 0.1*DecodeTemperature(), 0.1*DecodeHumidity(),
             (1.0/64)*DecodeVoltage(), Status.TxPower+4, -0.5*Status.RadioNoise, (1<<Status.RxRate)-1 );
     printf("\n");
   }

   void PrintPosition(void) const
   { printf(" %X:%c:%06lX R%c%c",
            (int)Position.AcftType, '0'+Header.AddrType, (long int)Header.Address, '0'+Header.Relay,
            Header.Emergency?'E':' ');
     printf(" %d/%dD/%4.1f", (int)Position.FixQuality, (int)Position.FixMode+2, 0.1*(10+DecodeDOP()) );
     if(Position.Time<60) printf(" %02ds:", (int)Position.Time);
                 else printf(" ---:");
     printf(" [%+10.6f, %+11.6f]deg %ldm",
            0.0001/60*DecodeLatitude(), 0.0001/60*DecodeLongitude(), (long int)DecodeAltitude() );
     if(hasBaro())
     { printf("[%+dm]", (int)getBaroAltDiff() ); }
     printf(" %3.1fm/s %05.1fdeg %+4.1fm/s %+4.1fdeg/s",
            0.1*DecodeSpeed(), 0.1*DecodeHeading(), 0.1*DecodeClimbRate(), 0.1*DecodeTurnRate() );
     printf("\n");
   }

   // calculate distance vector [LatDist, LonDist] from a given reference [RefLat, Reflon]
   int calcDistanceVector(int32_t &LatDist, int32_t &LonDist, int32_t RefLat, int32_t RefLon, uint16_t LatCos=3000, int32_t MaxDist=0x7FFF)
   { LatDist = ((DecodeLatitude()-RefLat)*1517+0x1000)>>13;           // convert from 1/600000deg to meters (40000000m = 360deg) => x 5/27 = 151$
     if(abs(LatDist)>MaxDist) return -1;
     LonDist = ((DecodeLongitude()-RefLon)*1517+0x1000)>>13;
     if(abs(LonDist)>(4*MaxDist)) return -1;
             LonDist = (LonDist*LatCos+0x800)>>12;
     if(abs(LonDist)>MaxDist) return -1;
     return 1; }

   // sets position [Lat, Lon] according to given distance vector [LatDist, LonDist] from a reference point [RefLat, RefLon]
   void setDistanceVector(int32_t LatDist, int32_t LonDist, int32_t RefLat, int32_t RefLon, uint16_t LatCos=3000)
   { EncodeLatitude(RefLat+(LatDist*27)/5);
     LonDist = (LonDist<<12)/LatCos;                                  // LonDist/=cosine(Latitude)
     EncodeLongitude(RefLon+(LonDist*27)/5); }

   // Centripetal acceleration
   static int16_t calcCPaccel(int16_t Speed, int16_t TurnRate) { return ((int32_t)TurnRate*Speed*229+0x10000)>>17; } // [0.1m/s^2]
   int16_t calcCPaccel(void) { return calcCPaccel(DecodeSpeed(), DecodeTurnRate()); }

   // Turn radius
   static int16_t calcTurnRadius(int16_t Speed, int16_t TurnRate, int16_t MaxRadius=0x7FFF) // [m] ([0.1m/s], [], [m])
   { if(TurnRate==0) return 0;
     int32_t Radius = 14675*Speed;
     Radius /= TurnRate; Radius = (Radius+128)>>8;
     if(abs(Radius)>MaxRadius) return 0;
     return Radius; }
   int16_t calcTurnRadius(int16_t MaxRadius=0x7FFF) { return calcTurnRadius(DecodeSpeed(), DecodeTurnRate(), MaxRadius); }

   void Clear(void) { HeaderWord=0; Data[0]=0; Data[1]=0; Data[2]=0; Data[3]=0; }

   uint32_t getAddressAndType(void) const { return HeaderWord&0x03FFFFFF; } // Address with address-type: 26-bit
   void     setAddressAndType(uint32_t AddrAndType) { HeaderWord = (HeaderWord&0xFC000000) | (AddrAndType&0x03FFFFFF); }

   bool goodAddrParity(void) const  { return ((Count1s(HeaderWord&0x0FFFFFFF)&1)==0); }  // Address parity should be EVEN
   void calcAddrParity(void)        { if(!goodAddrParity()) HeaderWord ^= 0x08000000; }  // if not correct parity, flip the parity bit

   void clrBaro(void)                   { Position.BaroAltDiff=EncodeGray((uint16_t)0x100); }
   bool hasBaro(void) const             { return DecodeGray((uint16_t)Position.BaroAltDiff)!=0x100; }

   void setBaroAltDiff(int32_t AltDiff)
   { if(AltDiff<(-255)) AltDiff=(-255);
     else if(AltDiff>255) AltDiff=255;
     Position.BaroAltDiff=EncodeGray((uint16_t)(AltDiff&0x1FF)); }
   int16_t getBaroAltDiff(void) const
   { int16_t AltDiff=DecodeGray((uint16_t)Position.BaroAltDiff);
     if(AltDiff & 0x100) AltDiff |= 0xFE00;
     return AltDiff; }

   void EncodeStdAltitude(int32_t StdAlt) { setBaroAltDiff((StdAlt-DecodeAltitude())); }
   int32_t DecodeStdAltitude(void) const { return (DecodeAltitude()+getBaroAltDiff()); }

   void EncodeAltitude(int32_t Altitude)                               // encode altitude in meters
   { Altitude += 1024;
     if(Altitude<0) Altitude=0;
     Position.Altitude = EncodeGray((uint16_t)UnsVRencode<uint16_t, 12>((uint16_t)Altitude)); }

   int32_t DecodeAltitude(void) const                                   // return Altitude in meters
   { return (int32_t)UnsVRdecode<uint16_t, 12>(DecodeGray((uint16_t)Position.Altitude))-1024; }

   static const int32_t FullAngle = 360*600000;  // Latitude and Longitude are encoded in units of 1/10000 arcmin
   static const int32_t HalfAngle = FullAngle/2;

   // void EncodeLatitude(int32_t Latitude)
   // { Position.Latitude = ((int64_t)Latitude*83399993+(1<<28))>>29; }

   // int32_t DecodeLatitude(void) const
   // { int32_t Latitude = Position.Latitude;
   //   return ((int64_t)Latitude*HalfAngle+(1<<23))>>24; }

   void EncodeLatitude(int32_t Latitude)
   { Latitude = ((int64_t)Latitude*83399993+(1<<28))>>29;           // convert to cordic units
     Latitude &= 0x00FFFFFF;
     Position.Latitude = EncodeGray((uint32_t)Latitude); }

   int32_t DecodeLatitude(void) const
   { int32_t Latitude = DecodeGray((uint32_t)Position.Latitude);
     if(Latitude&0x00800000) Latitude |= 0xFF000000;
     return ((int64_t)Latitude*HalfAngle+(1<<23))>>24; }           // convert from cordic units to 1/10000 arcmin

   void EncodeLongitude(int32_t Longitude)
   { Longitude = ((int64_t)Longitude*83399993+(1<<28))>>29;
     Longitude &= 0x01FFFFFF;
     Position.Longitude = EncodeGray((uint32_t)Longitude); }

   int32_t DecodeLongitude(void) const
   { int32_t Longitude = DecodeGray((uint32_t)Position.Longitude);
     if(Longitude&0x01000000) Longitude |= 0xFE000000;
     Longitude = ((int64_t)Longitude*HalfAngle+(1<<23))>>24;
     return Longitude; }

   void EncodeDOP(uint8_t DOP)
   { Position.DOP = EncodeGray(UnsVRencode<uint8_t, 4>(DOP)); }

   uint8_t DecodeDOP(void) const
   { return UnsVRdecode<uint8_t, 4>(DecodeGray((uint8_t)Position.DOP)); }

   void EncodeSpeed(int16_t Speed)                                       // speed in 0.2 knots (or 0.1m/s)
   { if(Speed<0) Speed=0;
     else Speed=UnsVRencode<uint16_t, 8>(Speed);
     Position.Speed = EncodeGray((uint16_t)Speed); }

   int16_t DecodeSpeed(void) const                                       // return speed in 0.2 knots or 0.1m/s units
   { return UnsVRdecode<uint16_t, 8>(DecodeGray((uint16_t)Position.Speed)); }         // => max. speed: 3832*0.2 = 766 knots

   void EncodeHeading(int16_t Heading)
   { if(Heading<0) Heading+=3600;
     Heading = ((int32_t)Heading*1165+(1<<11))>>12;
     Position.Heading = EncodeGray((uint16_t)Heading); }

   int16_t DecodeHeading(void) const                                     // return Heading in 0.1 degree units 0..359.9 deg
   { int32_t Heading = DecodeGray((uint16_t)Position.Heading);
     return (Heading*3600+512)>>10; }

   void setHeadingAngle(uint16_t HeadingAngle)
   { Position.Heading = EncodeGray((uint16_t)((HeadingAngle+32)>>6)); }

   uint16_t getHeadingAngle(void) const
   { return (uint16_t)DecodeGray((uint16_t)Position.Heading)<<6; }

   void clrTurnRate(void)       { Position.TurnRate=0x80; }
   bool hasTurnRate(void) const { return Position.TurnRate==0x80; }

   void EncodeTurnRate(int16_t Turn)                                     // [0.1 deg/sec]
   { Position.TurnRate = SignVRencode<int16_t, 5>(Turn); }

   int16_t DecodeTurnRate(void) const
   { return SignVRdecode<int16_t, 5>(Position.TurnRate); }

   void clrClimbRate(void)       { Position.ClimbRate=0x100; }
   bool hasClimbRate(void) const { return Position.ClimbRate==0x100; }

   void EncodeClimbRate(int16_t Climb)                                   // [0.1m/s]
   { Position.ClimbRate = SignVRencode<int16_t, 6>(Climb); }

   int16_t DecodeClimbRate(void) const
   { return SignVRdecode<int16_t, 6>(Position.ClimbRate); }

// --------------------------------------------------------------------------------------------------------------
// Status fields

   void clrTemperature(void)              { Status.Temperature=EncodeGray((uint8_t)0x80); }
   bool hasTemperature(void)       const  { return DecodeGray((uint8_t)Status.Temperature)!=0x80; }
   void EncodeTemperature(int16_t Temp)   { Status.Temperature=EncodeGray(EncodeSR2V5(Temp-200)); } // [0.1degC]
   int16_t DecodeTemperature(void) const  { return 200+DecodeSR2V5(DecodeGray((uint8_t)Status.Temperature)); }

   void clrHumidity(void)                 { Status.Humidity=EncodeGray((uint8_t)0x80); }
   bool hasHumidity(void)        const    { return DecodeGray((uint8_t)Status.Humidity)!=0x80; }
   void EncodeHumidity(uint16_t Hum)      { Status.Humidity=EncodeGray(EncodeSR2V5((int16_t)(Hum-525))); }     // [0.1%]
   uint16_t DecodeHumidity(void) const     { return 525+DecodeSR2V5(DecodeGray((uint8_t)Status.Humidity)); }

   void EncodeVoltage_mV(uint16_t Voltage) { EncodeVoltage((Voltage*8+63)/125); }
   void EncodeVoltage(uint16_t Voltage)                      // [1/64V]
   { if(Voltage<80) Voltage = 0;
               else Voltage-=80;
     Status.Voltage=EncodeGray(EncodeUR2V6(Voltage)); }
  uint16_t DecodeVoltage(void) const      { return 80+DecodeUR2V6(DecodeGray((uint8_t)Status.Voltage)); }

// --------------------------------------------------------------------------------------------------------------
// Info fields: pack and unpack 7-bit char into the Info packets

   void setInfoChar(uint8_t Char, uint8_t Idx)                           // put 7-bit Char onto give position
   { if(Idx>=16) return;                                                 // Idx = 0..15
     Char&=0x7F;
     uint8_t BitIdx = Idx*7;                                             // [bits] bit index to the target field
             Idx = BitIdx>>3;                                            // [bytes] index of the first byte to change
     uint8_t Ofs = BitIdx&0x07;
     if(Ofs==0) { Info.Data[Idx] = (Info.Data[Idx]&0x80) |  Char     ; return; }
     if(Ofs==1) { Info.Data[Idx] = (Info.Data[Idx]&0x01) | (Char<<1) ; return; }
     uint8_t Len1 = 8-Ofs;
     uint8_t Len2 = Ofs-1;
     uint8_t Msk1 = 0xFF; Msk1<<=Ofs;
     uint8_t Msk2 = 0x01; Msk2 = (Msk2<<Len2)-1;
     Info.Data[Idx  ] = (Info.Data[Idx  ]&(~Msk1)) | (Char<<Ofs);
     Info.Data[Idx+1] = (Info.Data[Idx+1]&(~Msk2)) | (Char>>Len1); }

   uint8_t getInfoChar(uint8_t Idx) const                                // get 7-bit Char from given position
   { if(Idx>=16) return 0;                                               // Idx = 0..15
     uint8_t BitIdx = Idx*7;                                             // [bits] bit index to the target field
             Idx = BitIdx>>3;                                            // [bytes] index of the first byte to change
     uint8_t Ofs = BitIdx&0x07;
     if(Ofs==0) return Info.Data[Idx]&0x7F;
     if(Ofs==1) return Info.Data[Idx]>>1;
     uint8_t Len = 8-Ofs;
     return (Info.Data[Idx]>>Ofs) | ((Info.Data[Idx+1]<<Len)&0x7F); }

   void clrInfo(void)                                    // clear the info packet
   { Info.DataChars=0;                                   // clear number of characters
     Info.ReportType=1; }                                // just in case: set the report-type

   uint8_t addInfo(const char *Value, uint8_t InfoType)  // add an info field
   { uint8_t Idx=Info.DataChars;                         // number of characters already in the info packet
     if(Idx) Idx++;                                      // if at least one already, then skip over the terminator
     if(Idx>=15) return 0;
     uint8_t Len=0;
     for( ; ; )
     { uint8_t Char = Value[Len]; if(Char==0) break;
       if(Idx>=15) return 0;
       setInfoChar(Char, Idx++);
       Len++; }
     setInfoChar(InfoType, Idx);                         // terminating character
     Info.DataChars=Idx;                                 // update number of characters
     return Len+1; }                                     // return number of added Value characters

   uint8_t readInfo(char *Value, uint8_t &InfoType, uint8_t ValueIdx=0) const
   { uint8_t Len=0;                                       // count characters in the info-string
     uint8_t Chars = Info.DataChars;                      // total number of characters in the record
     char Char=0;
     for( ; ; )                                           // loop over characters
     { if((ValueIdx+Len)>Chars) return 0;                 // return failure if overrun the data
       Char = getInfoChar(ValueIdx+Len);                  // get the character
       if(Char<0x20) break;                               // if less than 0x20 (space) then this is the terminator
       Value[Len++]=Char; }
     Value[Len]=0;                                        // null-terminate the infor string
     InfoType=Char;                                       // get the info-type: Pilot, Type, etc.
     return Len+1; }                                      // return number of character taken thus info length + terminator

   uint8_t InfoCheck(void) const
   { uint8_t Check=0;
     for( uint8_t Idx=0; Idx<15; Idx++)
     { Check ^= Info.Byte[Idx]; }
     // printf("Check = %02X\n", Check);
     return Check; }

   void setInfoCheck(void)
   { Info.Check = InfoCheck();
     // printf("Check = %02X\n", Info.Check);
   }

  uint8_t goodInfoCheck(void) const
   { return Info.Check == InfoCheck(); }

// --------------------------------------------------------------------------------------------------------------

   void Whiten  (uint8_t Prefix=0x05)
   { uint32_t Mask = ((uint32_t)Prefix<<26) | (HeaderWord & 0x03FFFFFF);
     for( uint8_t Idx=0; Idx<4; Idx++)
     { XorShift32(Mask); Mask += 0x01234567;
       Data[Idx] ^= Mask; }
   }

   void Dewhiten(void) { Whiten(); }

} ;

#endif // __OGN2_H__

