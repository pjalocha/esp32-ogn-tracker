#ifndef __OGN1_H__
#define __OGN1_H__

#include <stdio.h>

#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "ognconv.h"

#include "intmath.h"

#include "bitcount.h"
#include "nmea.h"
#include "mavlink.h"

#include "format.h"

                           // the packet description here is how it look on the little-endian CPU before sending it to the RF chip
                           // nRF905, CC1101, SPIRIT1, RFM69 chips actually reverse the bit order within every byte
                           // thus on the air the bits appear MSbit first for every byte transmitted

class OGN1_Packet          // Packet structure for the OGN tracker
{ public:

   static const int Words =  5;
   static const int Bytes = 20;

 union
 { uint32_t HeaderWord;    //    ECRR PMTT AAAA AAAA AAAA AAAA AAAA AAAA
                           // E=Emergency, C=enCrypt/Custom, RR=Relay count, P=Parity, M=isMeteo/Other, TT=address Type, AA..=Address:24-bit
                           // When enCrypt/Custom is set the data (position or whatever) can only be decoded by the owner
                           // This option is indented to pass any type of custom data not foreseen otheriwse
   struct
   { unsigned int Address    :24; // aircraft address
     unsigned int AddrType   : 2; // address type: 0 = random, 1 = ICAO, 2 = FLARM, 3 = OGN
     unsigned int NonPos     : 1; // 0 = position packet, 1 = other information like status
     unsigned int Parity     : 1; // parity takes into account bits 0..27 thus only the 28 lowest bits
     unsigned int Relay      : 2; // 0 = direct packet, 1 = relayed once, 2 = relayed twice, ...
     unsigned int Encrypted  : 1; // packet is encrypted
     unsigned int Emergency  : 1; // aircraft in emergency (not used for now)
   } Header ;

 } ;

 union
 { uint32_t Data[4];       // 0: QQTT TTTT LLLL LLLL LLLL LLLL LLLL LLLL  QQ=fix Quality:2, TTTTTT=time:6, LL..=Latitude:20
                           // 1: MBDD DDDD LLLL LLLL LLLL LLLL LLLL LLLL  F=fixMode:1 B=isBaro:1, DDDDDD=DOP:6, LL..=Longitude:20
                           // 2: RRRR RRRR SSSS SSSS SSAA AAAA AAAA AAAA  RR..=turn Rate:8, SS..=Speed:10, AA..=Alt:14
                           // 3: BBBB BBBB YYYY PCCC CCCC CCDD DDDD DDDD  BB..=Baro altitude:8, YYYY=AcftType:4, P=Stealth:1, CC..=Climb:9, DD..=Heading:10

                           // meteo/telemetry types: Meteo conditions, Thermal wind/climb, Device status, Precise time,

                           // meteo report: Humidity, Barometric pressure, Temperature, wind Speed/Direction
                           // 2: HHHH HHHH SSSS SSSS SSAA AAAA AAAA AAAA
                           // 3: TTTT TTTT YYYY BBBB BBBB BBDD DDDD DDDD  YYYY = report tYpe (meteo, thermal, water level, other telemetry)

                           // Device status: Time, baro pressure+temperature, GPS altitude, supply voltage, TX power, RF noise, software version, software features, hardware features,
                           // 0: UUUU UUUU UUUU UUUU UUUU UUUU UUUU UUUU  UU..=Unix time
                           // 1: SSSS SSSS SSSS SSSS TTTT TTTT HHHH HHHH  SS..=slot time, TT..=temperature, HH..=humidity
                           // 2: BBBB BBBB BBBB BBBB BBAA AAAA AAAA AAAA  Baro pressure[0.5Pa], GPS altitude
                           // 3: VVVV VVVV YYYY HHHH HHHH XXXX VVVV VVVV  VV..=firmware version, YYYY = report type, TT..=Temperatature, XX..=TxPower, VV..=battery voltage

                           // Pilot status:
                           // 0: NNNN NNNN NNNN NNNN NNNN NNNN NNNN NNNN  Name: 9 char x 7bit or 10 x 6bit or Huffman encoding ?
                           // 1: NNNN NNNN NNNN NNNN NNNN NNNN NNNN NNNN
   struct
   {   signed int    Latitude:24; //                  // QQTT TTTT LLLL LLLL LLLL LLLL LLLL LLLL  QQ=fix Quality:2, TTTTTT=time:6, LL..=Latitude:24
     unsigned int        Time: 6; // [sec]            // time, just second thus ambiguity every every minute
     unsigned int  FixQuality: 2; //                  // 0 = none, 1 = GPS, 2 = Differential GPS (can be WAAS)
       signed int   Longitude:24; //                  // MBDD DDDD LLLL LLLL LLLL LLLL LLLL LLLL  F=fixMode:1 B=isBaro:1, DDDDDD=DOP:6, LL..=Longitude:24
     unsigned int         DOP: 6; //                  // GPS Dilution of Precision
     unsigned int     BaroMSB: 1; //                  // negated bit #8 of the altitude difference between baro and GPS
     unsigned int     FixMode: 1; //                  // 0 = 2-D, 1 = 3-D
     unsigned int    Altitude:14; // [m] VR           // RRRR RRRR SSSS SSSS SSAA AAAA AAAA AAAA  RR..=turn Rate:8, SS..=Speed:10, AA..=Alt:14
     unsigned int       Speed:10; // [0.1m/s] VR
     unsigned int    TurnRate: 8; // [0.1deg/s] VR
     unsigned int     Heading:10; // [360/1024deg]    // BBBB BBBB YYYY PCCC CCCC CCDD DDDD DDDD  BB..=Baro altitude:8, YYYY=AcftType:4, P=Stealth:1, CC..=Climb:9, DD..=Heading:10
     unsigned int   ClimbRate: 9; // [0.1m/s] VR      // rate of climb/decent from GPS or from baro sensor
     unsigned int     Stealth: 1; //                  // not really used till now
     unsigned int    AcftType: 4; // [0..15]          // type of aircraft: 1 = glider, 2 = towplane, 3 = helicopter, ...
     unsigned int BaroAltDiff: 8; // [m]              // lower 8 bits of the altitude difference between baro and GPS
   } Position;

   struct
   { unsigned int Pulse     : 8; // [bpm]               // pilot: heart pulse rate
     unsigned int Oxygen    : 7; // [%]                 // pilot: oxygen level in the blood
     unsigned int SatSNR    : 5; // [dB]                // average SNR of GPS signals
     // unsigned int FEScurr   : 5; // [A]                 // FES current
     unsigned int RxRate    : 4; // [/min]              // log2 of received packet rate
     unsigned int Time      : 6; // [sec]               // same as in the position packet
     unsigned int FixQuality: 2;
     unsigned int AudioNoise: 8; // [dB]                //
     unsigned int RadioNoise: 8; // [dBm]               // noise seen by the RF chip
     unsigned int Temperature:8; // [0.1degC] VR        // temperature by the baro or RF chip
     unsigned int Humidity  : 8; // [0.1%] VR           // humidity
     unsigned int Altitude  :14; // [m] VR              // same as in the position packet
     unsigned int Pressure  :14; // [0.08hPa]           // barometric pressure
     unsigned int Satellites: 4; // [ ]
     unsigned int Firmware  : 8; // [ ]                 // firmware version
     unsigned int Hardware  : 8; // [ ]                 // hardware version
     unsigned int TxPower   : 4; // [dBm]               // RF trancmitter power (offset = 4)
     unsigned int ReportType: 4; // [0]                 // 0 for the status report
     unsigned int Voltage   : 8; // [1/64V] VR          // supply/battery voltage
   } Status;

   union
   {      uint8_t Byte[16];
     struct
     {    uint8_t Data[14];      // [16x7bit]packed string of 16-char: 7bit/char
          uint8_t DataChars:  4; // [int] number of characters in the packed string
          uint8_t ReportType: 4; // [1]                 // 1 for the Info packets
          uint8_t Check;         // CRC check
     } ;
   } Info;

   struct
   {   signed int    Latitude:24; //                  // Latitude of the measurement
     unsigned int        Time: 6; // [sec]            // time, just second thus ambiguity every every minute
     unsigned int            : 2; //                  // spare
       signed int   Longitude:24; //                  // Longitude of the measurement
     unsigned int            : 6; //                  // spare
     unsigned int     BaroMSB: 1; //                  // negated bit #8 of the altitude difference between baro and GPS
     unsigned int            : 1; //                  // spare
     unsigned int    Altitude:14; // [m] VR           // Altitude of the measurement
     unsigned int       Speed:10; // [0.1m/s] VR      // Horizontal wind speed
     unsigned int            : 8; //                  // spare
     unsigned int     Heading:10; //                  // Wind direction
     unsigned int   ClimbRate: 9; // [0.1m/s] VR      // Vertical wind speed
     unsigned int            : 1; //                  // spare
     unsigned int  ReportType: 4; //                  // 2 for wind/thermal report
     unsigned int BaroAltDiff: 8; // [m]              // lower 8 bits of the altitude difference between baro and GPS
   } Wind;

   struct
   {      uint8_t Data[14];                           // up to 14 bytes od specific data
     unsigned int DataLen   : 4;                      // 0..14 number of bytes in the message
     unsigned int ReportType: 4;                      // 15 for the manufacturer specific mesage
     unsigned int ManufID   : 8;                      // Manufacturer identification: 0 for Cube-Board
   } ManufMsg;                                        // manufacturer-specific message

  } ;

   uint8_t  *Byte(void) const { return (uint8_t  *)&HeaderWord; } // packet as bytes
   uint32_t *Word(void) const { return (uint32_t *)&HeaderWord; } // packet as words

   // void recvBytes(const uint8_t *SrcPacket) { memcpy(Byte(), SrcPacket, Bytes); } // load data bytes e.g. from a demodulator

   static const uint8_t InfoParmNum = 15; // [int]  number of info-parameters and their names
   static const char *InfoParmName(uint8_t Idx) { static const char *Name[InfoParmNum] =
                                                  { "Pilot", "Manuf", "Model", "Type", "SN", "Reg", "ID", "Class",
                                                    "Task" , "Base" , "ICE"  , "PilotID", "Hard", "Soft", "Crew" } ;
                                                  return Idx<InfoParmNum ? Name[Idx]:0; }

#ifndef __AVR__

   uint32_t getTime(uint32_t RefTime, int FwdMargin=5) const
   { if(Position.Time>=60) return 0;
     int Sec=RefTime%60;
     int DiffSec=Position.Time-Sec;
     if(DiffSec>FwdMargin) DiffSec-=60;  // difference should always be zero or negative, but can be small positive for predicted positions
     else if(DiffSec<=(-60+FwdMargin)) DiffSec+=60;
     return RefTime+DiffSec; }           // get out the correct position time

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX\n",
             (long int)HeaderWord, (long int)Data[0], (long int)Data[1],
             (long int)Data[2], (long int)Data[3] ); }

   uint8_t Read(const char *Inp)
   { uint8_t Len=0;
     if(Inp[0]==' ') Inp++;
     int Chars = Read_Hex(HeaderWord, Inp); if(Chars!=8) return 0;
     Inp+=Chars; Len+=4;
     for( uint8_t Idx=0; Idx<4; Idx++)
     { if(Inp[0]==' ') Inp++;
       int Chars = Read_Hex(Data[Idx], Inp); if(Chars!=8) return 0;
       Inp+=Chars; Len+=4; }
     return Len; }

   uint8_t Dump(char *Out)
   { uint8_t Len=0;
     Len+=Format_Hex(Out+Len, HeaderWord);
     for(int Idx=0; Idx<4; Idx++)
     { Out[Len++]=' '; Len+=Format_Hex(Out+Len, Data[Idx]); }
     return Len; }

   void DumpBytes(void) const
   { for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { printf(" %02X", Byte()[Idx]); }
     printf("\n"); }

   int Print(char *Out) const
   { int Len=0;
     Out[Len++]='0'+Header.AddrType; Out[Len++]=':';
     uint32_t Addr = Header.Address;
     Len+=Format_Hex(Out+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(Out+Len, (uint16_t)Addr);
     Out[Len++]=' ';
     Out[Len++]='R';
     Out[Len++]='0'+Header.Relay;
     if(!Header.NonPos) return Len+PrintPosition(Out+Len);
     if(isStatus()) return Len+PrintDeviceStatus(Out+Len);
     if(isInfo  ()) return Len+PrintDeviceInfo(Out+Len);
     Out[Len]=0; return Len; }

   int PrintPosition(char *Out) const
   { int Len=0;
     Out[Len++]=' ';
     // Out[Len++]=HexDigit(Position.AcftType); Out[Len++]=':';
     // Len+=Format_SignDec(Out+Len, -(int16_t)RxRSSI/2); Out[Len++]='d'; Out[Len++]='B'; Out[Len++]='m';
     // Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint16_t)Position.Time, 2);
     Len+=Format_String(Out+Len, "s [");
     Len+=Format_SignDec(Out+Len, DecodeLatitude()/6, 7, 5);
     Out[Len++]=',';
     Len+=Format_SignDec(Out+Len, DecodeLongitude()/6, 8, 5);
     Len+=Format_String(Out+Len, "]deg ");
     // Len+=Format_Latitude(Out+Len, DecodeLatitude());
     // Out[Len++]=' ';
     // Len+=Format_Longitude(Out+Len, DecodeLongitude());
     // Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint32_t)DecodeAltitude()); Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, DecodeSpeed(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, DecodeClimbRate(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len]=0;
     return Len; }

   int PrintDeviceStatus(char *Out) const
   { int Len=0;
     Out[Len++]=' '; Out[Len++]='h'; Len+=Format_Hex(Out+Len, (uint8_t)Status.Hardware);
     Out[Len++]=' '; Out[Len++]='v'; Len+=Format_Hex(Out+Len, (uint8_t)Status.Firmware);
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Status.Satellites); Out[Len++]='s'; Out[Len++]='a'; Out[Len++]='t';
     Out[Len++]='/'; Out[Len++]='0'+Status.FixQuality;
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, Status.SatSNR+8); Out[Len++]='d'; Out[Len++]='B';
     Out[Len++]=' '; Len+=Format_SignDec(Out+Len, DecodeAltitude(), 1, 0, 1); Out[Len++]='m';
     if(Status.Pressure>0)
     { Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, (((uint32_t)Status.Pressure<<3)+5)/10, 2, 1); Out[Len++]='h'; Out[Len++]='P'; Out[Len++]='a'; }
     if(hasTemperature())
     { Out[Len++]=' '; Len+=Format_SignDec(Out+Len, DecodeTemperature(), 2, 1); Out[Len++]='d'; Out[Len++]='e'; Out[Len++]='g'; Out[Len++]='C'; }
     if(hasHumidity())
     { Out[Len++]=' '; Len+=Format_SignDec(Out+Len, DecodeHumidity(), 2, 1); Out[Len++]='%'; }
     Out[Len++]=' '; Len+=Format_SignDec(Out+Len, ((uint32_t)DecodeVoltage()*100+32)>>6, 3, 2); Out[Len++]='V';
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Status.TxPower+4);
     Out[Len++]='/'; Out[Len++]='-'; Len+=Format_UnsDec(Out+Len, 5*Status.RadioNoise, 2, 1); Out[Len++]='d'; Out[Len++]='B'; Out[Len++]='m';
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, (1<<Status.RxRate)-1); Out[Len++]='/'; Out[Len++]='m'; Out[Len++]='i'; Out[Len++]='n';
     Out[Len]=0; return Len; }
/*
   int WriteDeviceStatus(char *Out) const
   { return sprintf(Out, " h%02X v%02X %dsat/%d/%ddB %ldm %3.1fhPa %+4.1fdegC %3.1f%% %4.2fV %d/%+4.1fdBm %d/min",
             Status.Hardware, Status.Firmware, Status.Satellites, Status.FixQuality, 8+Status.SatSNR, (long int)DecodeAltitude(),
             0.08*Status.Pressure, 0.1*DecodeTemperature(), 0.1*DecodeHumidity(),
             (1.0/64)*DecodeVoltage(), Status.TxPower+4, -0.5*Status.RadioNoise, (1<<Status.RxRate)-1 );
   }
*/
   int PrintDeviceInfo(char *Out) const
   { int Len=0;
     char Value[16];
     uint8_t InfoType;
     uint8_t Idx=0;
     for( ; ; )
     { uint8_t Chars = readInfo(Value, InfoType, Idx);
       if(Chars==0) break;
       Out[Len++]=' ';
       if(InfoType<InfoParmNum)
       { Len += Format_String(Out+Len, InfoParmName(InfoType)); }
       else
       { Out[Len++]='#'; Out[Len++]=HexDigit(InfoType); }
       Out[Len++]='=';
       Len += Format_String(Out+Len, Value);
       Idx+=Chars; }
     Out[Len]=0; return Len; }

/*
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
*/
   void setStatus  (void) { Status.ReportType=0; }
   void setInfo    (void) { Status.ReportType=1; }
   void setManufMsg(void) { Status.ReportType=15; }

   bool isStatus  (void) const { return Status.ReportType==0; }
   bool isInfo    (void) const { return Status.ReportType==1; }
   bool isManufMsg(void) const { return Status.ReportType==15; }

   void Print(void) const
   { if(!Header.NonPos) return PrintPosition();
     if(isStatus()) return PrintDeviceStatus();
     if(isInfo  ()) return PrintDeviceInfo(); }

   void PrintDeviceInfo(void) const
   { printf("%c:%06lX R%c%c%c:",
             '0'+Header.AddrType, (long int)Header.Address, '0'+Header.Relay, Header.Emergency?'E':' ', goodInfoCheck()?'+':'-');
     char Value[16];
     uint8_t InfoType;
     uint8_t Idx=0;
     for( ; ; )
     { uint8_t Chars = readInfo(Value, InfoType, Idx);
       if(Chars==0) break;
       if(InfoType<InfoParmNum)
       { printf(" %s=%s", InfoParmName(InfoType), Value); }
       else
       { printf(" #%d=%s", InfoType, Value); }
     Idx+=Chars; }
     printf("\n");
   }

   void PrintDeviceStatus(void) const
   { printf("%c:%06lX R%c%c %02ds:",
             '0'+Header.AddrType, (long int)Header.Address, '0'+Header.Relay, Header.Emergency?'E':' ', Status.Time);
     printf(" h%02X v%02X %dsat/%d/%ddB %ldm %3.1fhPa %+4.1fdegC %3.1f%% %4.2fV Tx:%ddBm Rx:%+4.1fdBm %d/min",
             Status.Hardware, Status.Firmware, Status.Satellites, Status.FixQuality, 8+Status.SatSNR, (long int)DecodeAltitude(),
             0.08*Status.Pressure, 0.1*DecodeTemperature(), 0.1*DecodeHumidity(),
             (1.0/64)*DecodeVoltage(), Status.TxPower+4, -0.5*Status.RadioNoise, (1<<Status.RxRate)-1 );
     printf("\n");
   }

   void PrintPosition(void) const
   { printf("%c%X:%c:%06lX R%c%c",
            Position.Stealth ?'s':' ', (int)Position.AcftType, '0'+Header.AddrType, (long int)Header.Address, '0'+Header.Relay,
	    Header.Emergency?'E':' ');
     printf(" %d/%dD/%4.1f", (int)Position.FixQuality, (int)Position.FixMode+2, 0.1*(10+DecodeDOP()) );
     if(Position.Time<60) printf(" %02ds:", (int)Position.Time);
                 else printf(" ---:");
     printf(" [%+010.6f,%+011.6f]deg %ldm",
            0.0001/60*DecodeLatitude(), 0.0001/60*DecodeLongitude(), (long int)DecodeAltitude() );
     if(hasBaro())
     { printf("[%+dm]", (int)getBaroAltDiff() ); }
     printf(" %3.1fm/s %05.1fdeg %+4.1fm/s %+4.1fdeg/s",
            0.1*DecodeSpeed(), 0.1*DecodeHeading(), 0.1*DecodeClimbRate(), 0.1*DecodeTurnRate() );
     printf("\n");
   }

   int DecodePosition(float &Lat, float &Lon, int &Alt)
   { if(Header.NonPos) return 0;
     Lat = (0.0001f/60)*DecodeLatitude();
     Lon = (0.0001f/60)*DecodeLongitude();
     Alt = DecodeAltitude();
     return 3; }

   int WriteStxJSON(char *JSON) const                              // Stratux JSON message
   { int Len=0;
     Len+=Format_String(JSON+Len, "\"addr\":\"");
     Len+=Format_Hex(JSON+Len, (uint8_t) (Header.Address>>16));
     Len+=Format_Hex(JSON+Len, (uint16_t)(Header.Address));
     JSON[Len++]='\"';
     JSON[Len++]=',';
     Len+=Format_String(JSON+Len, "\"addr_type\":");
     JSON[Len++] = HexDigit(Header.AddrType);
     if(!Header.Encrypted && !Header.NonPos)                       // if non-encrypted position
     { Len+=Format_String(JSON+Len, ",\"acft_type\":\"");
       JSON[Len++] = HexDigit(Position.AcftType);
       JSON[Len++]='\"';
       Len+=Format_String(JSON+Len, ",\"acft_cat\":\"");           // GDL90 aircraft category
                            // no-info, glider, tow, heli, parachute, drop-plane, hang-glider, para-glider, powered, jet, UFO, balloon, Zeppelin, UAV, ground vehicle, static } ; 
       const uint8_t AcftCat[16] = { 0,      9,   1,    7,        11,          1,          12,          12,       1,   2,   0,      10,       10,  14, 18, 19 } ;
       Len+=Format_Hex(JSON+Len, AcftCat[Position.AcftType]);
       JSON[Len++]='\"';
       Len+=Format_String(JSON+Len, ",\"stealth\":");
       JSON[Len++] = '0'+Position.Stealth;
       Len+=Format_String(JSON+Len, ",\"lat_deg\":");
       Len+=Format_SignDec(JSON+Len, (int32_t)(((int64_t)50*DecodeLatitude()+1)/3), 8, 7, 1);
       Len+=Format_String(JSON+Len, ",\"lon_deg\":");
       Len+=Format_SignDec(JSON+Len, (int32_t)(((int64_t)50*DecodeLongitude()+1)/3), 8, 7, 1);
       int32_t Altitude=DecodeAltitude();
       Len+=Format_String(JSON+Len, ",\"alt_msl_m\":");
       Len+=Format_UnsDec(JSON+Len, (uint32_t)Altitude);
       if(hasBaro())
       { Altitude+=getBaroAltDiff();
         Len+=Format_String(JSON+Len, ",\"alt_std_m\":");
         Len+=Format_SignDec(JSON+Len, Altitude, 1, 0, 1); }
       Len+=Format_String(JSON+Len, ",\"track_deg\":");
       Len+=Format_UnsDec(JSON+Len, DecodeHeading(), 2, 1);
       Len+=Format_String(JSON+Len, ",\"speed_mps\":");
       Len+=Format_UnsDec(JSON+Len, DecodeSpeed(), 2, 1);
       if(hasClimbRate())
       { Len+=Format_String(JSON+Len, ",\"climb_mps\":");
         Len+=Format_SignDec(JSON+Len, DecodeClimbRate(), 2, 1, 1); }
       if(hasTurnRate())
       { Len+=Format_String(JSON+Len, ",\"turn_dps\":");
         Len+=Format_SignDec(JSON+Len, DecodeTurnRate(), 2, 1, 1); }
       Len+=Format_String(JSON+Len, ",\"DOP\":");
       Len+=Format_UnsDec(JSON+Len, 10+DecodeDOP(), 2, 1); }
     else if(!Header.Encrypted && Header.NonPos)                         // non-encrypted status and info
     { if(isStatus())                                                    // status
       { }
       else if(isInfo())                                                 // info
       { char Value[16];
         uint8_t InfoType;
         uint8_t Idx=0;
         for( ; ; )
         { uint8_t Chars = readInfo(Value, InfoType, Idx);
           if(Chars==0) break;
           if(InfoType<InfoParmNum)
           { Len+=sprintf(JSON+Len, ",\"%s\":\"%s\"", InfoParmName(InfoType), Value); }
           Idx+=Chars; }
       }
     }
     return Len; }
/*
   void Encode(MAV_ADSB_VEHICLE *MAV)
   { MAV->ICAO_address = HeaderWord&0x03FFFFFF;
     MAV->lat      = ((int64_t)50*DecodeLatitude()+1)/3;
     MAV->lon      = ((int64_t)50*DecodeLongitude()+1)/3;
     MAV->altitude = 1000*DecodeAltitude();
     MAV->heading  = 10*DecodeHeading();
     MAV->hor_velocity = 10*DecodeSpeed();
     MAV->ver_velocity = 10*DecodeClimbRate();
     MAV->flags         = 0x17;
     MAV->altitude_type =    1;
     MAV->callsign[0]   =    0;
     MAV->tslc          =    0;
     MAV->emiter_type   =    0; }
*/
   void Encode(MAV_ADSB_VEHICLE *MAV)
   { MAV->ICAO_address = Header.Address;
     MAV->lat           = ((int64_t)50*DecodeLatitude()+1)/3;          // convert coordinates to [1e-7deg]
     MAV->lon           = ((int64_t)50*DecodeLongitude()+1)/3;
     MAV->altitude      = 1000*DecodeAltitude();                       // convert to [mm[
     MAV->heading       = 10*DecodeHeading();                          // [cdeg/s]
     MAV->hor_velocity  = 10*DecodeSpeed();                            // [cm/s]
     MAV->ver_velocity  = 10*DecodeClimbRate();                        // [cm/s]
     MAV->flags         = 0x1F;                                        // all valid except for Squawk, not simulated
     MAV->altitude_type =    1;                                        // GPS altitude
     const static char Prefix[4] = { 'R', 'I', 'F', 'O' };             // prefix for Random, ICAO, Flarm and OGN address-types
     MAV->callsign[0]   =    Prefix[Header.AddrType];                  // create a call-sign from address-type and address
     Format_Hex((char *)MAV->callsign+1, ( uint8_t)(Header.Address>>16));      // highest byte
     Format_Hex((char *)MAV->callsign+3, (uint16_t)(Header.Address&0xFFFF));   // two lower bytes
     MAV->callsign[7]   =    0;                                        // end-of-string for call-sign
     MAV->squawk        =    0;                                        // what shall we put there for OGN ?
     MAV->tslc          =    1;                                        // 1sec for now but should be more precise
     const static uint8_t EmitterType[16] =                            // conversion table from OGN aircraft-type
     { 0,  9,  2,  7,     // unknown,          glider,           towplane,       helicopter
      11,  3,  9, 11,     // parachute,        drop plane,       hang-glider,    para-glider
       2,  3, 15, 10,     // powered aircraft, jet aircraft,     UFO,            balloon
      10, 14,  2, 19  };  // airship,          UAV,              ground vehiele, fixed object
     MAV->emiter_type   =    EmitterType[Position.AcftType];           // convert from the OGN
   }

   static const char *getAprsIcon(uint8_t AcftType)
   { static const char *AprsIcon[16] = // Icons for various FLARM acftType's
     { "/z",  //  0 = ?
       "/'",  //  1 = (moto-)glider    (most frequent)
       "/'",  //  2 = tow plane        (often)
       "/X",  //  3 = helicopter       (often)
       "/g" , //  4 = parachute        (rare but seen - often mixed with drop plane)
       "\\^", //  5 = drop plane       (seen)
       "/g" , //  6 = hang-glider      (rare but seen)
       "/g" , //  7 = para-glider      (rare but seen)
       "\\^", //  8 = powered aircraft (often)
       "/^",  //  9 = jet aircraft     (rare but seen)
       "/z",  //  A = UFO              (people set for fun)
       "/O",  //  B = balloon          (seen once)
       "/O",  //  C = airship          (seen once)
       "/'",  //  D = UAV              (drones, can become very common)
       "/z",  //  E = ground support   (ground vehicles at airfields)
       "\\n"  //  F = static object    (ground relay ?)
     } ;
     return AcftType<16 ? AprsIcon[AcftType]:0;
   }

   int ReadAPRS(const char *Msg)                                                 // read an APRS position message
   { Clear();

     const char *Data  = strchr(Msg, ':'); if(Data==0) return -1; // where the time/position data starts
     Data++;
     const char *Dest  = strchr(Msg, '>'); if(Dest==0) return -1; // where the destination call is
     Dest++;
     const char *Comma = strchr(Dest, ',');                       // the first comma after the destination call

     Position.AcftType=0xF;

     uint8_t AddrType;
     uint32_t Address;
          if(memcmp(Msg, "RND", 3)==0) AddrType=0;
     else if(memcmp(Msg, "ICA", 3)==0) AddrType=1;
     else if(memcmp(Msg, "FLR", 3)==0) AddrType=2;
     else if(memcmp(Msg, "OGN", 3)==0) AddrType=3;
     else AddrType=4;
     if(AddrType<4)
     { if(Read_Hex(Address, Msg+3)==6) Header.Address=Address;
       Header.AddrType=AddrType; }

     if(Comma)
     { if(memcmp(Comma+1, "RELAY*" , 6)==0) Header.Relay=1;
       else if(Comma[10]=='*') Header.Relay=1;
     }

     if(Data[0]!='/') return -1;
     int Sec, Min, Hour;
     if(Data[7]=='h')                                            // HHMMSS UTC time
     { Sec =Read_Dec2(Data+5); if(Sec<0)  return -1;
       Min =Read_Dec2(Data+3); if(Min<0)  return -1;
       Hour=Read_Dec2(Data+1); if(Hour<0) return -1;
     }
     else if(Data[7]=='z')                                       // DDHHMM UTC time
     { Sec =0;
       Min =Read_Dec2(Data+5); if(Min<0)  return -1;
       Hour=Read_Dec2(Data+3); if(Hour<0) return -1;
     }
     else return -1;

     int Time = Sec + Min*60 + Hour*3600;
     Position.Time=Sec;
     Data+=8;

     Position.FixMode=1;
     Position.FixQuality=1;
     EncodeDOP(0xFF);

     int8_t LatDeg  = Read_Dec2(Data);   if(LatDeg<0) return -1;
     int8_t LatMin  = Read_Dec2(Data+2); if(LatMin<0) return -1;
     if(Data[4]!='.') return -1;
     int8_t LatFrac = Read_Dec2(Data+5); if(LatFrac<0) return -1;
     int32_t Latitude = (int32_t)LatDeg*600000 + (int32_t)LatMin*10000 + (int32_t)LatFrac*100;
     char LatSign = Data[7];
     Data+=8+1;

     int16_t LonDeg  = Read_Dec3(Data);   if(LonDeg<0) return -1;
     int8_t  LonMin  = Read_Dec2(Data+3); if(LonMin<0) return -1;
     if(Data[5]!='.') return -1;
     int8_t LonFrac = Read_Dec2(Data+6); if(LonFrac<0) return -1;
     int32_t Longitude = (int32_t)LonDeg*600000 + (int32_t)LonMin*10000 + (int32_t)LonFrac*100;
     char LonSign = Data[8];
     Data+=9+1;

     int16_t Speed=0;
     int16_t Heading=0;
     if(Data[3]=='/')
     { Heading=Read_Dec3(Data);
       Speed=Read_Dec3(Data+4);
       Data+=7; }
     EncodeHeading(Heading*10);
     EncodeSpeed(((int32_t)Speed*337146+0x8000)>>16);

     uint32_t Altitude=0;
     if( (Data[0]=='/') && (Data[1]=='A') && (Data[2]=='=') && (Read_UnsDec(Altitude, Data+3)==6) )
     { Data+=9; }
     EncodeAltitude(FeetToMeters(Altitude));

     for( ; ; )
     { if(Data[0]!=' ') break;
       Data++;

       if( (Data[0]=='!') && (Data[1]=='W') && (Data[4]=='!') )
       { Latitude  += (Data[2]-'0')*10;
         Longitude += (Data[3]-'0')*10;
         Data+=5; continue; }

       if( (Data[0]=='i') && (Data[1]=='d') )
       { uint32_t ID; Read_Hex(ID, Data+2);
         Header.Address    = ID&0x00FFFFFF;
         Header.AddrType   = (ID>>24)&0x03;
         Position.AcftType = (ID>>26)&0x0F;
         Position.Stealth  = ID>>31;
         Data+=10; continue; }

       if( (Data[0]=='F') && (Data[1]=='L') && (Data[5]=='.') )
       { int16_t FLdec=Read_Dec3(Data+2);
         int16_t FLfrac=Read_Dec2(Data+6);
         if( (FLdec>=0) && (FLfrac>=0) )
         { uint32_t StdAlt = FLdec*100+FLfrac;
           EncodeStdAltitude(FeetToMeters(StdAlt)); }
         Data+=8; continue; }

       if( (Data[0]=='+') || (Data[0]=='-') )
       { int32_t Value; int8_t Len=Read_Float1(Value, Data);
         if(Len>0)
         { Data+=Len;
           if(memcmp(Data, "fpm", 3)==0) { EncodeClimbRate((333*Value+0x8000)>>16); Data+=3; continue; }
           if(memcmp(Data, "rot", 3)==0) { EncodeTurnRate(3*Value);    Data+=3; continue; }
         }
       }

       if( (Data[0]=='g') && (Data[1]=='p') && (Data[2]=='s') )
       { int16_t HorPrec=Read_Dec2(Data+3);
         if(HorPrec<0) HorPrec=Read_Dec1(Data[3]);
         if(HorPrec>=0)
         { uint16_t DOP=HorPrec*5; if(DOP<10) DOP=10; else if(DOP>230) DOP=230;
           EncodeDOP(DOP-10); Data+=5; }
       }
       while(Data[0]>' ') Data++;
     }

     if(LatSign=='S') Latitude=(-Latitude); else if(LatSign!='N') return -1;
     EncodeLatitude(Latitude);
     if(LonSign=='W') Longitude=(-Longitude); else if(LonSign!='E') return -1;
     EncodeLongitude(Longitude);

     return Time; }                                  // [sec] return Time-of-Day

   uint8_t WriteAPRS(char *Msg, uint32_t Time, const char *ProtName="APRS")    // write an APRS position message
   { uint8_t Len=0;
     static const char *AddrTypeName[4] = { "RND", "ICA", "FLR", "OGN" } ;
     memcpy(Msg+Len, AddrTypeName[Header.AddrType], 3); Len+=3;
     Len+=Format_Hex(Msg+Len, (uint8_t)(Header.Address>>16));
     Len+=Format_Hex(Msg+Len, (uint16_t)(Header.Address));
     Msg[Len++] = '>';
     uint8_t ProtLen = strlen(ProtName);
     memcpy(Msg+Len, ProtName, ProtLen); Len+=ProtLen;
     if(Header.Relay)
     { memcpy(Msg+Len, ",RELAY*", 7); Len+=7; }
     Msg[Len++] = ':';

     if(Header.NonPos && Status.ReportType>1) { Msg[Len]=0; return 0; } // give up if neither position nor status nor info

     if(Position.Time<60)
     // { uint32_t DayTime=Time%86400; int Sec=DayTime%60;           // second of the time the packet was recevied
     //   int DiffSec=Position.Time-Sec; if(DiffSec>4) DiffSec-=60;  // difference should always be zero or negative, but can be small positive for predicted positions
     //   Time+=DiffSec; }                                           // get out the correct position time
       Time = getTime(Time, 5);
     Msg[Len++] = Header.NonPos || Header.Encrypted?'>':'/';
     Len+=Format_HHMMSS(Msg+Len, Time);
     Msg[Len++] = 'h';

     if(Header.NonPos)                                            // status and info packets
     {      if(isStatus()) Len+=PrintDeviceStatus(Msg+Len);
       else if(isInfo()  ) Len+=PrintDeviceInfo(Msg+Len);
       Msg[Len]=0; return Len; }

     if(Header.Encrypted)                                         // encrypted packets
     { Msg[Len++]=' ';
       for(int Idx=0; Idx<4; Idx++)
       { Len+=EncodeAscii85(Msg+Len, Data[Idx]); }
       /* Msg[Len++]='\n'; */ Msg[Len]=0; return Len; }

     const char *Icon = getAprsIcon(Position.AcftType);

     int32_t Lat = DecodeLatitude();
     bool NegLat = Lat<0; if(NegLat) Lat=(-Lat);
     uint32_t LatDeg = Lat/600000;
     Len+=Format_UnsDec(Msg+Len, LatDeg, 2);
     Lat -= LatDeg*600000;
     uint32_t LatMin = Lat/100;
     Len+=Format_UnsDec(Msg+Len, LatMin, 4, 2);
     Lat -= LatMin*100;
     Msg[Len++] = NegLat ? 'S':'N';
     Msg[Len++] = Icon[0];

     int32_t Lon = DecodeLongitude();
     bool NegLon = Lon<0; if(NegLon) Lon=(-Lon);
     uint32_t LonDeg = Lon/600000;
     Len+=Format_UnsDec(Msg+Len, LonDeg, 3);
     Lon -= LonDeg*600000;
     uint32_t LonMin = Lon/100;
     Len+=Format_UnsDec(Msg+Len, LonMin, 4, 2);
     Lon -= LonMin*100;
     Msg[Len++] = NegLon ? 'W':'E';
     Msg[Len++] = Icon[1];

     Len+=Format_UnsDec(Msg+Len, (DecodeHeading()+5)/10, 3);
     Msg[Len++] = '/';
     Len+=Format_UnsDec(Msg+Len, ((uint32_t)DecodeSpeed()*199+512)>>10, 3);
     Msg[Len++] = '/'; Msg[Len++] = 'A'; Msg[Len++] = '='; Len+=Format_UnsDec(Msg+Len, MetersToFeet(DecodeAltitude()), 6);

     Msg[Len++] = ' ';
     Msg[Len++] = '!';
     Msg[Len++] = 'W';
     Msg[Len++] = '0'+Lat/10;
     Msg[Len++] = '0'+Lon/10;
     Msg[Len++] = '!';

     Msg[Len++] = ' '; Msg[Len++] = 'i'; Msg[Len++] = 'd'; Len+=Format_Hex(Msg+Len, ((uint32_t)Position.AcftType<<26) | ((uint32_t)Header.AddrType<<24) | Header.Address);

     if(hasClimbRate())
     { Msg[Len++] = ' '; Len+=Format_SignDec(Msg+Len, ((int32_t)DecodeClimbRate()*10079+256)>>9, 3); Msg[Len++] = 'f'; Msg[Len++] = 'p'; Msg[Len++] = 'm'; }

     if(hasTurnRate())
     { Msg[Len++] = ' '; Len+=Format_SignDec(Msg+Len, DecodeTurnRate()/3, 2, 1); Msg[Len++] = 'r'; Msg[Len++] = 'o'; Msg[Len++] = 't'; }

     if(hasBaro())
     { int32_t Alt = DecodeStdAltitude();
       if(Alt<0) Alt=0;
       Msg[Len++] = ' '; Msg[Len++] = 'F'; Msg[Len++] = 'L';
       Len+=Format_UnsDec(Msg+Len, MetersToFeet((uint32_t)Alt), 5, 2); }

     uint16_t DOP=10+DecodeDOP();
     uint16_t HorPrec=(DOP*2+5)/10; if(HorPrec>63) HorPrec=63;
     uint16_t VerPrec=(DOP*3+5)/10; if(VerPrec>63) VerPrec=63;
     Msg[Len++] = ' ';  Msg[Len++] = 'g'; Msg[Len++] = 'p'; Msg[Len++] = 's';
     Len+=Format_UnsDec(Msg+Len, HorPrec); Msg[Len++] = 'x'; Len+=Format_UnsDec(Msg+Len, VerPrec);

     // Msg[Len++]='\n';
     Msg[Len]=0;
     return Len; }

#endif // __AVR__

   // calculate distance vector [LatDist, LonDist] from a given reference [RefLat, Reflon]
   int calcDistanceVector(int32_t &LatDist, int32_t &LonDist, int32_t RefLat, int32_t RefLon, uint16_t LatCos=3000, int32_t MaxDist=0x7FFF)
   { LatDist = ((DecodeLatitude()-RefLat)*1517+0x1000)>>13;           // convert from 1/600000deg to meters (40000000m = 360deg) => x 5/27 = 1517/(1<<13)
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
/*
   uint8_t Print(char *Out) const
   { uint8_t Len=0;
     Out[Len++]=HexDigit(Position.AcftType); Out[Len++]=':';
     Out[Len++]='0'+Header.AddrType; Out[Len++]=':';
     uint32_t Addr = Header.Address;
     Len+=Format_Hex(Out+Len, (uint8_t)(Addr>>16));
     Len+=Format_Hex(Out+Len, (uint16_t)Addr);
     Out[Len++]=' ';
     // Len+=Format_SignDec(Out+Len, -(int16_t)RxRSSI/2); Out[Len++]='d'; Out[Len++]='B'; Out[Len++]='m';
     // Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint16_t)Position.Time, 2);
     Out[Len++]=' ';
     Len+=Format_Latitude(Out+Len, DecodeLatitude());
     Out[Len++]=' ';
     Len+=Format_Longitude(Out+Len, DecodeLongitude());
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, (uint32_t)DecodeAltitude()); Out[Len++]='m';
     Out[Len++]=' ';
     Len+=Format_UnsDec(Out+Len, DecodeSpeed(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]=' ';
     Len+=Format_SignDec(Out+Len, DecodeClimbRate(), 2, 1); Out[Len++]='m'; Out[Len++]='/'; Out[Len++]='s';
     Out[Len++]='\n'; Out[Len]=0;
     return Len; }
*/
   // OGN1_Packet() { Clear(); }
   void Clear(void) { HeaderWord=0; Data[0]=0; Data[1]=0; Data[2]=0; Data[3]=0; }

   uint32_t getAddressAndType(void) const { return HeaderWord&0x03FFFFFF; } // Address with address-type: 26-bit
   void     setAddressAndType(uint32_t AddrAndType) { HeaderWord = (HeaderWord&0xFC000000) | (AddrAndType&0x03FFFFFF); }

   bool goodAddrParity(void) const  { return ((Count1s(HeaderWord&0x0FFFFFFF)&1)==0); }  // Address parity should be EVEN
   void calcAddrParity(void)        { if(!goodAddrParity()) HeaderWord ^= 0x08000000; }  // if not correct parity, flip the parity bit

   void EncodeLatitude(int32_t Latitude)                                // encode Latitude: units are 0.0001/60 degrees
   { Position.Latitude = Latitude>>3; }

   int32_t DecodeLatitude(void) const
   { int32_t Latitude = Position.Latitude;
     // if(Latitude&0x00800000) Latitude|=0xFF000000;
     Latitude = (Latitude<<3)+4; return Latitude; }

   void EncodeLongitude(int32_t Longitude)                             // encode Longitude: units are 0.0001/60 degrees
   { Position.Longitude = Longitude>>=4; }

   int32_t DecodeLongitude(void) const
   { int32_t Longitude = Position.Longitude;
     Longitude = (Longitude<<4)+8; return Longitude; }

   bool hasBaro(void) const             { return Position.BaroMSB || Position.BaroAltDiff; }
   void clrBaro(void)                   { Position.BaroMSB=0; Position.BaroAltDiff=0; }
   int16_t getBaroAltDiff(void) const   { int16_t AltDiff=Position.BaroAltDiff; if(Position.BaroMSB==0) AltDiff|=0xFF00; return AltDiff; }
   void setBaroAltDiff(int32_t AltDiff)
   { if(AltDiff<(-255)) AltDiff=(-255); else if(AltDiff>255) AltDiff=255;
     Position.BaroMSB = (AltDiff&0xFF00)==0; Position.BaroAltDiff=AltDiff&0xFF; }
   void EncodeStdAltitude(int32_t StdAlt) { setBaroAltDiff((StdAlt-DecodeAltitude())); }
   int32_t DecodeStdAltitude(void) const { return (DecodeAltitude()+getBaroAltDiff()); }

   void EncodeAltitude(int32_t Altitude)                               // encode altitude in meters
   { if(Altitude<0)      Altitude=0;
     Position.Altitude = UnsVRencode<uint16_t, 12>((uint16_t)Altitude); }
//     Position.Altitude = EncodeUR2V12((uint16_t)Altitude); }

   int32_t DecodeAltitude(void) const                                   // return Altitude in meters
   { return UnsVRdecode<uint16_t, 12>(Position.Altitude); }
//   { return DecodeUR2V12(Position.Altitude); }

   void EncodeDOP(uint8_t DOP)
   { Position.DOP = UnsVRencode<uint8_t, 4>(DOP); }
//   { Position.DOP = EncodeUR2V4(DOP); }

   uint8_t DecodeDOP(void) const
   { return UnsVRdecode<uint8_t, 4>(Position.DOP); }
//   { return DecodeUR2V4(Position.DOP); }

   void EncodeSpeed(int16_t Speed)                                       // speed in 0.2 knots (or 0.1m/s)
   { if(Speed<0) Speed=0;
     else Speed = UnsVRencode<uint16_t, 8>(Speed); // EncodeUR2V8(Speed);
     Position.Speed = Speed; }

   int16_t DecodeSpeed(void) const           // return speed in 0.2 knots or 0.1m/s units
   { return UnsVRdecode<uint16_t, 8>(Position.Speed); }
//   { return DecodeUR2V8(Position.Speed); }   // => max. speed: 3832*0.2 = 766 knots

   int16_t DecodeHeading(void) const         // return Heading in 0.1 degree units 0..359.9 deg
   { int32_t Heading = Position.Heading;
     return (Heading*3600+512)>>10; }

   void EncodeHeading(int16_t Heading)
   { Position.Heading = (((int32_t)Heading<<10)+180)/3600; }

   void setHeadingAngle(uint16_t HeadingAngle)
   { Position.Heading = (((HeadingAngle+32)>>6)); }

   uint16_t getHeadingAngle(void) const
   { return (uint16_t)Position.Heading<<6; }

   void clrTurnRate(void)       { Position.TurnRate=0x80; }               // mark turn-rate as absent
   bool hasTurnRate(void) const { return Position.TurnRate!=0x80; }

   void EncodeTurnRate(int16_t Turn)                                      // [0.1 deg/sec]
   { Position.TurnRate = EncodeSR2V5(Turn); }

   int16_t DecodeTurnRate(void) const
   { return DecodeSR2V5(Position.TurnRate); }

   void clrClimbRate(void)       { Position.ClimbRate=0x100; }            // mark climb rate as absent
   bool hasClimbRate(void) const { return Position.ClimbRate!=0x100; }

   void EncodeClimbRate(int16_t Climb)
   { Position.ClimbRate = EncodeSR2V6(Climb); }

   int16_t DecodeClimbRate(void) const
   { return DecodeSR2V6(Position.ClimbRate); }

// --------------------------------------------------------------------------------------------------------------
// Status fields

   void clrTemperature(void)              { Status.Temperature=0x80; }
   bool hasTemperature(void)       const  { return Status.Temperature!=0x80; }
   void EncodeTemperature(int16_t Temp)   { Status.Temperature=EncodeSR2V5(Temp-200); } // [0.1degC]
   int16_t DecodeTemperature(void) const  { return 200+DecodeSR2V5(Status.Temperature); }

   void EncodeVoltage(uint16_t Voltage)   { Status.Voltage=EncodeUR2V6(Voltage); }      // [1/64V]
  uint16_t DecodeVoltage(void) const      { return DecodeUR2V6(Status.Voltage); }

   void clrHumidity(void)                 { Status.Humidity=0x80; }
   bool hasHumidity(void)        const    { return Status.Humidity!=0x80; }
   void EncodeHumidity(uint16_t Hum)      { Status.Humidity=EncodeSR2V5((int16_t)(Hum-520)); }     // [0.1%]
   uint16_t DecodeHumidity(void) const    { return 520+DecodeSR2V5(Status.Humidity); }

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

   void Encrypt (const uint32_t Key[4]) { XXTEA_Encrypt(Data, 4, Key, 8); }              // encrypt with given Key
   void Decrypt (const uint32_t Key[4]) { XXTEA_Decrypt(Data, 4, Key, 8); }              // decrypt with given Key

   void Whiten  (void) { TEA_Encrypt_Key0(Data, 8); TEA_Encrypt_Key0(Data+2, 8); } // whiten the position
   void Dewhiten(void) { TEA_Decrypt_Key0(Data, 8); TEA_Decrypt_Key0(Data+2, 8); } // de-whiten the position

  uint8_t getTxSlot(uint8_t Idx) const // Idx=0..15
  { const uint32_t *DataPtr = Data;
    uint32_t  Mask=1; Mask<<=Idx;
    uint8_t Slot=0;
    for(uint8_t Bit=0; Bit<6; Bit++)
    { Slot>>=1;
      if(DataPtr[Bit]&Mask) Slot|=0x20;
      Mask<<=1; Slot>>=1; }
    return EncodeGray(Slot); }

} ;

/*
class OGN1_DiffPacket
{ public:
   union
   { uint32_t Word;
     struct
     { uint8_t dTime:4;          // [0..15sec] time difference
       int32_t dLat :6;          // [-32..+31]
       int32_t dLon :6;          // [-32..+31]
       int32_t dAlt :5;          // [-16..+15]
       int32_t dVel :5;          // [-16..+15]
       int32_t dHead:6;          // [-32..+31]
     } ;
   } ;

  public:
   bool Encode(const OGN1_Packet &Pos, const OGN1_Packet &RefPos)
   { int8_t dT = RefPos.Position.Time - Pos.Position.Time; if(dT<0) dT+=60;
     if(dT>15) return 0;
     int32_t dLat  = (int32_t)RefPos.Position.Latitude  - (int32_t)Pos.Position.Latitude;
     int32_t dLon  = (int32_t)RefPos.Position.Longitude - (int32_t)Pos.Position.Longitude;
     int16_t dAlt  = (int16_t)RefPos.Position.Altitude  - (int16_t)Pos.Position.Altitude;
     int16_t dVel  = (int16_t)RefPos.Position.Speed     - (int16_t)Pos.Position.Speed;        // [0.1m/s] difference in speed
     int16_t dHead = (int16_t)RefPos.Position.Heading   - (int16_t)Pos.Position.Heading;      // [10bit cordic] difference in heading
             dHead&=0x03FF; if(dHead&0x0200) dHead|=0xFC00;
     return 1; }

} ;
*/

#endif // of __OGN1_H__

