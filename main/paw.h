#ifndef __PAW_H__
#define __PAW_H__

#include <stdint.h>
#include <string.h>

#include "ogn1.h"
#include "format.h"

class PAW_Packet
{ public:
   static const uint8_t Size = 24;
   union
   { uint8_t Byte[Size];        // 24 bytes per packet, counting the internal CRC but not the external one
     struct
     { union
       { uint32_t AddrWord;
         struct
         { uint8_t  Sync   : 8; // the first (thus lowest) byte is the "sync" = '$' = 0x24
           uint32_t Address:24; // 24-bit address: can be ICAO or internally produced
         } ;
       } ;
       float    Longitude;      // [deg]
       float    Latitude;       // [deg]
       uint16_t Altitude;       // [m]
       union
       { uint16_t HeadWord;
         struct
         { uint16_t Heading:9;  // [deg]
            int8_t  Climb  :7;  // [64fpm]
         } ;
       } ;
       union
       { uint32_t SeqMsg;
         struct
         { uint8_t  Seq;        // sequence number to transmit longer messages
           uint8_t  Msg[3];     // 3-byte part of the longer message
         } ;
       } ;
       union
       { uint16_t SpeedWord;
         struct
         { uint16_t Speed:10;   // [kt]
           uint8_t  Time : 6;   // [sec]
         } ;
       } ;
       union
       { uint8_t TypeByte;
         struct
         { uint8_t  AcftType:4; // [] lower nibble is the aircraft-type like for FLARM/OGN, upper nibble possibly retransmit flag
           bool     Relay   :1; // relay flag (by ground station)
           bool     OGN     :1; // for packets produced by OGN-Tracker
           uint8_t  AddrType:2; // address-type for OGN packets (if OGN==1)
         } ;
       } ;
       uint8_t  CRC;            // internal CRC: a XOR of all bytes
    } ;
  } ;

  public:
   void Copy(const uint8_t *Data) { memcpy(Byte, Data, Size); }
   void Clear(void)
   { Byte[0]=0x24;
     for(int Idx=1; Idx<Size; Idx++)
       Byte[Idx]=0; }

   uint8_t getAddrType(void) const            // get (or guess) address-type
   { if(OGN) return AddrType;                 // if OGN-Tracker then AddrType is explicit
     if(AcftType==0xF) return 3;              // if fixed object then OGN-type
     if(Address<0xD00000) return 1;           // ICAO-type
     if(Address<0xE00000) return 2;           // FLARM-type
     return 3; }

   // uint32_t getAddress(void) const { return Address>>8; }              // remove the sync '$'
   // void     setAddress(uint32_t Addr) { Address = (Addr<<8) | 0x24; }  // set new address and set the '$' sync char

   int Copy(const OGN1_Packet &Packet, bool Ext=0)
   { Clear();
     Address  = Packet.Header.Address;                     // [24-bit]
     if(Packet.Header.NonPos) return 0;                    // encode only position packets
     AcftType = Packet.Position.AcftType;                  // [4-bit] aircraft-type
     Altitude = Packet.DecodeAltitude();                   // [m]
     Heading = Packet.DecodeHeading()/10;                  // [deg]
     Speed = (398*(int32_t)Packet.DecodeSpeed()+1024)>>11; // [0.1m/s] => [kts]
     Latitude  = 0.0001f/60*Packet.DecodeLatitude();       // [deg]
     Longitude = 0.0001f/60*Packet.DecodeLongitude();      // [deg]
     if(Ext)
     { OGN=1;                                              // extended data flag
       AddrType = Packet.Header.AddrType;                  // [2-bit]
       Relay    = Packet.Header.Relay;                     // relay flag
       // Time = Packet.Position.Time;                        // [sec]
       int32_t ClimbRate = Packet.DecodeClimbRate();       // [0.1m/s]
       ClimbRate = (ClimbRate*315+512)>>10;                // [64fpm]
       if(ClimbRate>127) ClimbRate=127;
       else if(ClimbRate<(-127)) ClimbRate=(-127);
       Climb = ClimbRate; }
     SeqMsg = 0;
     setCRC(); return 1; }

   int WriteJSON(char *JSON) const
   { int Len=0;
     Len+=Format_String(JSON+Len, "\"addr\":\"");
     Len+=Format_Hex(JSON+Len, (uint8_t) (Address>>16));
     Len+=Format_Hex(JSON+Len, (uint16_t)(Address));
     JSON[Len++]='\"';
     Len+=Format_String(JSON+Len, ",\"addr_type\":");
     JSON[Len++] = HexDigit(getAddrType());
     Len+=Format_String(JSON+Len, ",\"acft_type\":\"");
     JSON[Len++] = HexDigit(AcftType);
     JSON[Len++]='\"';
     Len+=Format_String(JSON+Len, ",\"acft_cat\":\"");           // GDL90 aircraft category
                          // no-info, glider, tow, heli, parachute, drop-plane, hang-glider, para-glider, powered, jet, UFO, balloon, Zeppelin, UAV, ground vehicle, static } ; 
     const uint8_t AcftCat[16] = { 0,      9,   1,    7,        11,          1,          12,          12,       1,   2,   0,      10,       10,  14, 18, 19 } ;
     Len+=Format_Hex(JSON+Len, AcftCat[AcftType]);
     JSON[Len++]='\"';
     // uint32_t PosTime=Time; if(nsTime<300000000) PosTime--;
     // Len+=Format_String(JSON+Len, ",\"time\":");
     // Len+=Format_UnsDec(JSON+Len, PosTime);
     // int64_t RxTime=(int64_t)Time-PosTime; RxTime*=1000; RxTime+=nsTime/1000000;
     // Len+=Format_String(JSON+Len, ",\"rx_time\":");
     // Len+=Format_SignDec(JSON+Len, RxTime, 4, 3, 1);
     Len+=sprintf(JSON+Len, ",\"lat_deg\":%8.7f,\"lon_deg\":%8.7f,\"alt_msl_m\":%d", Latitude, Longitude, Altitude);
     Len+=sprintf(JSON+Len, ",\"track_deg\":%d,\"speed_mps\":%3.1f", Heading, 0.514*Speed);
     if(OGN) Len+=sprintf(JSON+Len, ",\"climb_mps\":%3.1f", 0.32512*Climb);
     return Len; }

   uint8_t Dump(char *Out)
   { uint8_t Len=0;
     for(int Idx=0; Idx<Size; Idx++)
     { Len+=Format_Hex(Out+Len, Byte[Idx]); }
     return Len; }

   void Print(const char *Name="PAW:") const
   { if(Name) printf("%s ", Name);
     printf("%02X:%06X [%+09.5f,%+010.5f]deg %4dm, %03ddeg %3dkt %02X:%02X%02X%02X ",
            TypeByte, Address, Latitude, Longitude, Altitude, Heading, Speed, Seq, Msg[0], Msg[1], Msg[2]);
     for(int Idx=0; Idx<3; Idx++)
     { printf("%c", Msg[Idx]<' '?'.':Msg[Idx]); }
     printf("\n"); }

  uint8_t Read(const char *Inp)                                      // read packet from a hex dump
  { uint8_t Len;
    for(Len=0; Len<Size; Len++)                                     // read as many hex bytes as you can
    { int8_t Upp = Read_Hex1(Inp[0]); if(Upp<0) break;               // 1st digit
      int8_t Low = Read_Hex1(Inp[1]); if(Low<0) break;               // 2nd digit
      Byte[Len] = (Upp<<4) | Low; Inp+=2; }                          // new byte, count input
    return Len; }                                                    // return number of bytes read = packet length = should be 24

   static void Whiten(uint8_t *Packet, int Len)                       // whitening applied to PAW packet, includes internal CRC
   { const static uint8_t White[] = { 0x05, 0xb4, 0x05, 0xae, 0x14, 0xda,
        0xbf, 0x83, 0xc4, 0x04, 0xb2, 0x04, 0xd6, 0x4d, 0x87, 0xe2, 0x01, 0xa3, 0x26,
        0xac, 0xbb, 0x63, 0xf1, 0x01, 0xca, 0x07, 0xbd, 0xaf, 0x60, 0xc8, 0x12, 0xed,
        0x04, 0xbc, 0xf6, 0x12, 0x2c, 0x01, 0xd9, 0x04, 0xb1, 0xd5, 0x03, 0xab, 0x06,
        0xcf, 0x08, 0xe6, 0xf2, 0x07, 0xd0, 0x12, 0xc2, 0x09, 0x34, 0x20 };
     for(int Idx=0; Idx<Len; Idx++)
     { Packet[Idx]^=White[Idx]; }
   }

   void setCRC(void) { CRC = IntCRC(Byte, Size, 0x00); }                  // set the internal CRC of the packet

   uint8_t IntCRC(void) const { return IntCRC(Byte, Size, 0x00); }        // over all bytes it should be a zero
                                                                          // thus XOR of all bytes including the CRC should be a zero

   static uint8_t IntCRC(const uint8_t *Packet, int Len=Size, uint8_t CRC=0x00) // internal PAW packet CRC
   { for(int Idx=0; Idx<Len; Idx++)
       CRC ^= Packet[Idx];
     return CRC; }

   static uint8_t CRC8(uint8_t *Packet, int Len, uint8_t CRC=0x71)  // external PAW packet checksum with 0x71 seed
   { for(int Idx=0; Idx<Len; Idx++)
       CRC = CRC8(Packet[Idx], CRC);
     return CRC; }

   uint8_t CRC8(void) { return CRC8(Byte, Size, 0x71); }            // calc. external CRC8 for the packet, Poly = 107

   static uint8_t CRC8(uint8_t Byte, uint8_t CRC)
   { static const uint8_t Table[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
							    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3 } ;
     return Table[CRC ^ Byte]; }

} ;

class PAW_RxPacket          // Received PilotAware packet
{ public:
   PAW_Packet Packet;
   // uint8_t  CRC;            // []        external CRC (we assume it is correct)
   uint8_t  CSNR;           // [0.5dB]  carrier Signal-to-Noise Ratio
   uint8_t   SNR;           // [0.25dB]
   int16_t FreqOfs;         // [10Hz]
   uint32_t Time;           // [sec]
   uint32_t nsTime;         // [nsec]

  public:
   void Print(void)
   { printf("PAW: %d.%03ds: %02X:%06X [%+09.5f, %+010.5f]deg %4dm, %03ddeg %3dkt #%02X %3.1f/%3.1fdB %+4.1fkHz\n",
            Time, nsTime/1000000, Packet.TypeByte, Packet.Address, Packet.Latitude, Packet.Longitude, Packet.Altitude, Packet.Heading, Packet.Speed,
            Packet.Seq, 0.25*SNR, 0.5*CSNR, 0.01*FreqOfs); }

   int WriteJSON(char *JSON) const
   { int Len = Packet.WriteJSON(JSON);
     uint32_t PosTime=Time; if(nsTime<300000000) PosTime--;
     if(Packet.OGN)
     { }
     Len+=Format_String(JSON+Len, ",\"time\":");
     Len+=Format_UnsDec(JSON+Len, PosTime);
     int64_t RxTime=(int64_t)Time-PosTime; RxTime*=1000; RxTime+=nsTime/1000000;
     Len+=Format_String(JSON+Len, ",\"rx_time\":");
     Len+=Format_SignDec(JSON+Len, RxTime, 4, 3, 1);
     return Len; }

} ;

#endif // __PAW_H__
