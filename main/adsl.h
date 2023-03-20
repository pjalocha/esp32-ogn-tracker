#ifndef __ADSL_H__
#define __ADSL_H__

// #include <stdlib.h>
// #include <string.h>
// #include "radiodemod.h"
// #include "intmath.h"
#include "ognconv.h"
// #include "bitcount.h"
// #include "format.h"
// #include "crc1021.h"

class ADSL_Packet
{ public:

   const static uint8_t TxBytes = 27; // including SYNC, Length and 24-bit CRC
   const static uint8_t SYNC1 = 0x72;
   const static uint8_t SYNC2 = 0x4B;

   uint8_t SYNC[2];          // two bytes for correct alignment: can contain the last two SYNC bytes
   uint8_t Length;           // [bytes] packet length = 24 = 0x18 (excluding length but including the 24-bit CRC)
   uint8_t Version;          // Version[4]/Sigmature[1]/Key[2]/Reserved[1]
   union
   { uint32_t Word[5];       // this part to be scrambled/encrypted, is aligned to 32-bit
     struct                  // this is aligned to 32-bit
     { uint8_t Type;         // 2=iConspicuity, bit #7 = Unicast
       uint8_t Address  [4]; // Address[30]/Reserved[1]/RelayForward[1] (not aligned to 32-bit !)
       union
       { uint8_t Meta     [2]; // Time[6]/Cat[5]/Emerg[3]/FlightState[2]
         struct
         { uint8_t TimeStamp   :6; // [0.25sec]
           uint8_t FlightState :2; // 0=unknown, 1=ground, 2=airborne
           uint8_t AcftCat     :5; // 1=light, 2=small-heavy, 3=heli, 4=glider, 5=baloon/airship, 6=para/hang-glider, 7=skydiver, 
           uint8_t Emergency   :3; // 1=OK
         } ;
       } ;
       uint8_t Position[11]; // Lat[24]/Lon[24]/Speed[8]/Alt[14]/Climb[9]/Track[9]
       union
       { uint8_t Integrity[2]; // SourceInteg[2]/DesignAssurance[2]/NavigationIntegrity[4]/NorizAccuracy[3]/VertAccuracy[2]/ValocityAccuracy[2]/Reserved[1]
         struct
         { uint8_t SourceIntegrity:2; // 3=1e-7/h, 2=1e-5/h, 1=1e-3/h
           uint8_t DesignAssurance:2; // 3=B, 2=C, 1=D
           uint8_t NavigIntegrity :4; // 12=7.5m, 11=25m, 10=75m
           uint8_t HorizAccuracy  :3; // 7=3m, 6=10m, 5=30m
           uint8_t VertAccuracy   :2; // 3=15m, 2=45m, 1=150m
           uint8_t VelAccuracy    :2; // 3=1m/s 2=3m/s 3=10m/s
           uint8_t Reserved       :1; //
         } ;
       } ;
     } ;
   } ;
   uint8_t CRC[3];           // 24-bit (is aligned to 32-bit)

   // uint8_t Spare;
   // uint32_t Time;            // [sec] receive time or other estimate

  public:
   void Init(void) { SYNC[0]=SYNC1; SYNC[1]=SYNC2; Length=TxBytes-3; Version=0x00; Type=0x02; }

   void Print(void) const
   { printf(" v%02X %4.1fs: %02X:%06X [%+09.5f,%+010.5f]deg %dm %+4.1fm/s %05.1fdeg %3.1fm/s\n",
         Version, 0.25*TimeStamp, getAddrTable(), getAddress(), FNTtoFloat(getLat()), FNTtoFloat(getLon()),
         getAlt(), 0.125*getClimb(), (45.0/0x40)*getTrack(), 0.25*getSpeed()); }

   int Print(char *Out) const
   { return sprintf(Out, "%02X:%06X %4.1fs [%+09.5f,%+010.5f]deg %dm %+4.1fm/s %05.1fdeg %3.1fm/s",
         getAddrTable(), getAddress(), 0.25*TimeStamp, FNTtoFloat(getLat()), FNTtoFloat(getLon()),
         getAlt(), 0.125*getClimb(), (45.0/0x40)*getTrack(), 0.25*getSpeed()); }

/*
   uint32_t getAddress(void)   const { return get3bytes(Address); }
   uint8_t  getAddrTable(void) const { return Address[3]&0x3F; }

    void    setAddress(uint32_t Addr)   { set3bytes(Address, Addr); }
    void    setAddrTable(uint8_t Table) { Address[3] = (Address[3]&0xC0) | Table; }
*/

   uint8_t  getRelay(void)     const { return Address[3]&0x80; }
   void     setRelay(uint8_t Relay)  { Address[3] = (Address[3]&0x7F) | (Relay<<7); }

   static uint32_t get3bytes(const uint8_t *Byte) { int32_t Word=Byte[2]; Word<<=8; Word|=Byte[1]; Word<<=8; Word|=Byte[0]; return Word; }
   static void     set3bytes(uint8_t *Byte, uint32_t Word) { Byte[0]=Word; Byte[1]=Word>>8; Byte[2]=Word>>16; }

   static uint32_t get4bytes(const uint8_t *Byte)
   { uint32_t Word =Byte[3]; Word<<=8;
              Word|=Byte[2]; Word<<=8;
              Word|=Byte[1]; Word<<=8;
              Word|=Byte[0];
     return Word; }
   static void     set4bytes(uint8_t *Byte, uint32_t Word) { Byte[0]=Word; Byte[1]=Word>>8; Byte[2]=Word>>16; Byte[3]=Word>>24; }

   uint32_t getAddress(void) const
   { uint32_t Addr = get4bytes(Address); return (Addr>>6)&0x00FFFFFF; }

   void setAddress(uint32_t NewAddr)
   { uint32_t Addr = get4bytes(Address);
     Addr = (Addr&0xC000003F) | (NewAddr<<6);
     set4bytes(Address, Addr); }

   uint8_t  getAddrTable(void) const { return Address[0]&0x3F; }
    void    setAddrTable(uint8_t Table) { Address[0] = (Address[0]&0xC0) | Table; }

   uint8_t getAddrType(void) const
   { uint8_t Table=getAddrTable();
     if(Table==0x05) return 1;         // ICAO
     if(Table==0x06) return 2;         // FLARM
     if(Table==0x07) return 3;         // OGN
     if(Table==0x08) return 2;         // FANET => FLARM ?
     return 0; }

   void setAddrType(uint8_t AddrType)
   { if(AddrType==0) setAddrTable(0);
     else setAddrTable(AddrType+4); }

   void setAcftType(uint8_t AcftType)
   { const uint8_t Map[16] = { 0, 4, 1, 3,                     // unknown, glider, tow-plane, helicopter
                               8, 1, 7, 7,                     // sky-diver, drop plane, hang-glider, para-glider
                               1, 2, 0, 5,                     // motor airplane, jet, UFO, balloon
                               5,11, 0, 0 } ;                  // airship, UAV, ground vehicle, static object
     if(AcftType<16) AcftCat=Map[AcftType];
                else AcftCat=0; }
   uint8_t getAcftType(void) const
   { const uint8_t Map[32] = { 0, 8, 9, 3, 1,12, 2, 7,
                               4,13, 3,13,13,13, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0 } ;
     return Map[AcftCat]; }

   uint8_t getHorPrec(void) const
   { const uint8_t Map[8] = { 63, 63, 63, 63, 63, 30, 10, 3 } ;
     return Map[HorizAccuracy]; }
   void setHorPrec(uint8_t Prec)
   {      if(Prec<= 3) HorizAccuracy=7;
     else if(Prec<=10) HorizAccuracy=6;
     else if(Prec<=30) HorizAccuracy=5;
     else HorizAccuracy=4;
     VelAccuracy = HorizAccuracy-4; }

   uint8_t getVerPrec(void) const
   { const uint8_t Map[8] = { 63, 63, 45, 15 } ;
     return Map[VertAccuracy]; }
   void setVerPrec(uint8_t Prec)
   {      if(Prec<=15) HorizAccuracy=3;
     else if(Prec<=45) HorizAccuracy=2;
     else HorizAccuracy=1; }

   static int32_t FNTtoUBX(int32_t Coord) { return ((int64_t)900007296*Coord+0x20000000)>>30; } // [FANET-cordic ] => [1e-7 deg]
   static int32_t OGNtoFNT(int32_t Coord) { return ((int64_t)Coord*83399317+(1<<21))>>22; }     // [0.0001/60 deg] => [FANET cordic]
   static int32_t UBXtoFNT(int32_t Coord) { return ((int64_t)Coord*5003959 +(1<<21))>>22; }     // [1e-7 deg]      => [FANET cordic]
   static float   FNTtoFloat(int32_t Coord)                             // convert from FANET cordic units to float degrees
   { const float Conv = 90.0007295677/0x40000000;                       // FANET cordic conversion factor (not exactly cordic)
     return Conv*Coord; }

    int32_t getLat(void) const { int32_t Lat=get3bytes(Position  ); Lat<<=8; Lat>>=1; return Lat; } // FANET-cordic
    int32_t getLon(void) const { int32_t Lon=get3bytes(Position+3); Lon<<=8; return Lon; }          // FANET-cordic

    void    setLat(int32_t Lat)  { Lat = (Lat+0x40)>>7; set3bytes(Position  , Lat); }           // FANET-cordic
    void    setLon(int32_t Lon)  { Lon = (Lon+0x80)>>8; set3bytes(Position+3, Lon); }           // FANET-cordic

    uint16_t getSpeed(void) const { return UnsVRdecode<uint16_t,6>(Position[6]); }              // [0.25 m/s]
    void setSpeed(uint16_t Speed) { Position[6] = UnsVRencode<uint16_t,6>(Speed); }             // [0.25 m/s]

   int32_t getAlt(void) const                                                                  // [m]
   { int32_t Word=Position[8]&0x3F; Word<<=8; Word|=Position[7];
     return UnsVRdecode<int32_t,12>(Word)-316; }
   void setAlt(int32_t Alt)
   { Alt+=316; if(Alt<0) Alt=0;
     int32_t Word=UnsVRencode<uint32_t,12>(Alt);
     Position[7]=Word;
     Position[8] = (Position[8]&0xC0) | (Word>>8); }

   int16_t getClimbWord(void) const                                                             //
   { int16_t Word=Position[9]&0x7F; Word<<=2; Word|=Position[8]>>6; return Word; }
   int16_t getClimb(void) const                                                                 // [0.125 m/s]
   { return SignVRdecode<int16_t,6>(getClimbWord()); }
   void setClimb(int16_t Climb)                                                                 // [0.125 m/s]
   { setClimbWord(SignVRencode<int16_t,6>(Climb)); }
   void setClimbWord(int16_t Word)
   { Position[8] = (Position[8]&0x3F) | ((Word&0x03)<<6);
     Position[9] = (Position[9]&0x80) |  (Word>>2); }
   bool hasClimb(void) { return getClimbWord()!=0x100; }                                        // climb-rate present or absent
   void clrClimb(void) { setClimbWord(0x100); }                                                 // declare climb-rate as absent

   uint16_t getTrack(void) const                                                                // 9-bit cordic
   { int16_t Word=Position[10]; Word<<=1; Word|=Position[9]>>7; return Word; }
   void setTrack(int16_t Word)
   { Position[9] = (Position[9]&0x7F) | ((Word&0x01)<<7);
     Position[10] = Word>>1; }

   void Scramble(void)
   { XXTEA_Encrypt_Key0(Word, 5, 6); }

   void Descramble(void)
   { XXTEA_Decrypt_Key0(Word, 5, 6); }

   static uint32_t PolyPass(uint32_t CRC, uint8_t Byte)     // pass a single byte through the CRC polynomial
   { const uint32_t Poly = 0xFFFA0480;
     CRC |= Byte;
     for(uint8_t Bit=0; Bit<8; Bit++)
     { if(CRC&0x80000000) CRC ^= Poly;
       CRC<<=1; }
     return CRC; }

   static uint32_t checkPI(const uint8_t *Byte, uint8_t Bytes) // run over data bytes and the three CRC bytes
   { uint32_t CRC = 0;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { CRC = PolyPass(CRC, Byte[Idx]); }
     return CRC>>8; }                                          // should be all zero for a correct packet

   static uint32_t calcPI(const uint8_t *Byte, uint8_t Bytes)  // calculate PI for the given packet data excluding the three CRC bytes
   { uint32_t CRC = 0;
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { CRC = PolyPass(CRC, Byte[Idx]); }
     CRC=PolyPass(CRC, 0); CRC=PolyPass(CRC, 0); CRC=PolyPass(CRC, 0);
     return CRC>>8; }                                          //

    void setCRC(void)
    { uint32_t Word = calcPI((const uint8_t *)&Version, TxBytes-6);
      CRC[0]=Word>>16; CRC[1]=Word>>8; CRC[2]=Word; }

    uint32_t checkCRC(void) const
    { return checkPI((const uint8_t *)&Version, TxBytes-3); }

    static uint32_t CRCsyndrome(uint8_t Bit)
    { const uint16_t PacketBytes = TxBytes-3;
      const uint16_t PacketBits = PacketBytes*8;
      const uint32_t Syndrome[PacketBits] = {
 0x7ABEE1, 0xC2A574, 0x6152BA, 0x30A95D, 0xE7AEAA, 0x73D755, 0xC611AE, 0x6308D7,
 0xCE7E6F, 0x98C533, 0xB3989D, 0xA6364A, 0x531B25, 0xD67796, 0x6B3BCB, 0xCA67E1,
 0x9AC9F4, 0x4D64FA, 0x26B27D, 0xECA33A, 0x76519D, 0xC4D2CA, 0x626965, 0xCECEB6,
 0x67675B, 0xCC49A9, 0x99DED0, 0x4CEF68, 0x2677B4, 0x133BDA, 0x099DED, 0xFB34F2,
 0x7D9A79, 0xC13738, 0x609B9C, 0x304DCE, 0x1826E7, 0xF3E977, 0x860EBF, 0xBCFD5B,
 0xA184A9, 0xAF3850, 0x579C28, 0x2BCE14, 0x15E70A, 0x0AF385, 0xFA83C6, 0x7D41E3,
 0xC15AF5, 0x9F577E, 0x4FABBF, 0xD82FDB, 0x93EDE9, 0xB60CF0, 0x5B0678, 0x2D833C,
 0x16C19E, 0x0B60CF, 0xFA4A63, 0x82DF35, 0xBE959E, 0x5F4ACF, 0xD05F63, 0x97D5B5,
 0xB410DE, 0x5A086F, 0xD2FE33, 0x96851D, 0xB4B88A, 0x5A5C45, 0xD2D426, 0x696A13,
 0xCB4F0D, 0x9A5D82, 0x4D2EC1, 0xD96D64, 0x6CB6B2, 0x365B59, 0xE4D7A8, 0x726BD4,
 0x3935EA, 0x1C9AF5, 0xF1B77E, 0x78DBBF, 0xC397DB, 0x9E31E9, 0xB0E2F0, 0x587178,
 0x2C38BC, 0x161C5E, 0x0B0E2F, 0xFA7D13, 0x82C48D, 0xBE9842, 0x5F4C21, 0xD05C14,
 0x682E0A, 0x341705, 0xE5F186, 0x72F8C3, 0xC68665, 0x9CB936, 0x4E5C9B, 0xD8D449,
 0x939020, 0x49C810, 0x24E408, 0x127204, 0x093902, 0x049C81, 0xFDB444, 0x7EDA22,
 0x3F6D11, 0xE04C8C, 0x702646, 0x381323, 0xE3F395, 0x8E03CE, 0x4701E7, 0xDC7AF7,
 0x91C77F, 0xB719BB, 0xA476D9, 0xADC168, 0x56E0B4, 0x2B705A, 0x15B82D, 0xF52612,
 0x7A9309, 0xC2B380, 0x6159C0, 0x30ACE0, 0x185670, 0x0C2B38, 0x06159C, 0x030ACE,
 0x018567, 0xFF38B7, 0x80665F, 0xBFC92B, 0xA01E91, 0xAFF54C, 0x57FAA6, 0x2BFD53,
 0xEA04AD, 0x8AF852, 0x457C29, 0xDD4410, 0x6EA208, 0x375104, 0x1BA882, 0x0DD441,
 0xF91024, 0x7C8812, 0x3E4409, 0xE0D800, 0x706C00, 0x383600, 0x1C1B00, 0x0E0D80,
 0x0706C0, 0x038360, 0x01C1B0, 0x00E0D8, 0x00706C, 0x003836, 0x001C1B, 0xFFF409,
 0x800000, 0x400000, 0x200000, 0x100000, 0x080000, 0x040000, 0x020000, 0x010000,
 0x008000, 0x004000, 0x002000, 0x001000, 0x000800, 0x000400, 0x000200, 0x000100,
 0x000080, 0x000040, 0x000020, 0x000010, 0x000008, 0x000004, 0x000002, 0x000001 } ;
      return Syndrome[Bit]; }

    static uint8_t FindCRCsyndrome(uint32_t Syndr)              // quick search for a single-bit CRC syndrome
    { const uint16_t PacketBytes = TxBytes-3;
      const uint16_t PacketBits = PacketBytes*8;
      const uint32_t Syndrome[PacketBits] = {
 0x000001BF, 0x000002BE, 0x000004BD, 0x000008BC, 0x000010BB, 0x000020BA, 0x000040B9, 0x000080B8,
 0x000100B7, 0x000200B6, 0x000400B5, 0x000800B4, 0x001000B3, 0x001C1BA6, 0x002000B2, 0x003836A5,
 0x004000B1, 0x00706CA4, 0x008000B0, 0x00E0D8A3, 0x010000AF, 0x01856788, 0x01C1B0A2, 0x020000AE,
 0x030ACE87, 0x038360A1, 0x040000AD, 0x049C816D, 0x06159C86, 0x0706C0A0, 0x080000AC, 0x0939026C,
 0x099DED1E, 0x0AF3852D, 0x0B0E2F5A, 0x0B60CF39, 0x0C2B3885, 0x0DD44197, 0x0E0D809F, 0x100000AB,
 0x1272046B, 0x133BDA1D, 0x15B82D7E, 0x15E70A2C, 0x161C5E59, 0x16C19E38, 0x1826E724, 0x18567084,
 0x1BA88296, 0x1C1B009E, 0x1C9AF551, 0x200000AA, 0x24E4086A, 0x2677B41C, 0x26B27D12, 0x2B705A7D,
 0x2BCE142B, 0x2BFD538F, 0x2C38BC58, 0x2D833C37, 0x304DCE23, 0x30A95D03, 0x30ACE083, 0x34170561,
 0x365B594D, 0x37510495, 0x38132373, 0x3836009D, 0x3935EA50, 0x3E44099A, 0x3F6D1170, 0x400000A9,
 0x457C2992, 0x4701E776, 0x49C81069, 0x4CEF681B, 0x4D2EC14A, 0x4D64FA11, 0x4E5C9B66, 0x4FABBF32,
 0x531B250C, 0x56E0B47C, 0x579C282A, 0x57FAA68E, 0x58717857, 0x5A086F41, 0x5A5C4545, 0x5B067836,
 0x5F4ACF3D, 0x5F4C215E, 0x609B9C22, 0x6152BA02, 0x6159C082, 0x62696516, 0x6308D707, 0x67675B18,
 0x682E0A60, 0x696A1347, 0x6B3BCB0E, 0x6CB6B24C, 0x6EA20894, 0x70264672, 0x706C009C, 0x726BD44F,
 0x72F8C363, 0x73D75505, 0x76519D14, 0x78DBBF53, 0x7A930980, 0x7ABEE100, 0x7C881299, 0x7D41E32F,
 0x7D9A7920, 0x7EDA226F, 0x800000A8, 0x80665F8A, 0x82C48D5C, 0x82DF353B, 0x860EBF26, 0x8AF85291,
 0x8E03CE75, 0x91C77F78, 0x93902068, 0x93EDE934, 0x96851D43, 0x97D5B53F, 0x98C53309, 0x99DED01A,
 0x9A5D8249, 0x9AC9F410, 0x9CB93665, 0x9E31E955, 0x9F577E31, 0xA01E918C, 0xA184A928, 0xA476D97A,
 0xA6364A0B, 0xADC1687B, 0xAF385029, 0xAFF54C8D, 0xB0E2F056, 0xB3989D0A, 0xB410DE40, 0xB4B88A44,
 0xB60CF035, 0xB719BB79, 0xBCFD5B27, 0xBE959E3C, 0xBE98425D, 0xBFC92B8B, 0xC1373821, 0xC15AF530,
 0xC2A57401, 0xC2B38081, 0xC397DB54, 0xC4D2CA15, 0xC611AE06, 0xC6866564, 0xCA67E10F, 0xCB4F0D48,
 0xCC49A919, 0xCE7E6F08, 0xCECEB617, 0xD05C145F, 0xD05F633E, 0xD2D42646, 0xD2FE3342, 0xD677960D,
 0xD82FDB33, 0xD8D44967, 0xD96D644B, 0xDC7AF777, 0xDD441093, 0xE04C8C71, 0xE0D8009B, 0xE3F39574,
 0xE4D7A84E, 0xE5F18662, 0xE7AEAA04, 0xEA04AD90, 0xECA33A13, 0xF1B77E52, 0xF3E97725, 0xF526127F,
 0xF9102498, 0xFA4A633A, 0xFA7D135B, 0xFA83C62E, 0xFB34F21F, 0xFDB4446E, 0xFF38B789, 0xFFF409A7 } ;

      uint16_t Bot=0;
      uint16_t Top=PacketBits;
      uint32_t MidSyndr=0;
      for( ; ; )
      { uint16_t Mid=(Bot+Top)>>1;
        MidSyndr = Syndrome[Mid]>>8;
        if(Syndr==MidSyndr) return (uint8_t)Syndrome[Mid];
        if(Mid==Bot) break;
        if(Syndr< MidSyndr) Top=Mid;
                       else Bot=Mid; }
      return 0xFF; }

} __attribute__((packed));

#endif // __ADSL_H__
