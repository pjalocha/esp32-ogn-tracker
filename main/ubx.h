#ifndef __UBX_H__
#define __UBX_H__

// UBX Class packet numbers
const uint8_t UBX_NAV = 0x01; // navigation
const uint8_t UBX_ACK = 0x05; // acknoledgement of configuration
const uint8_t UBX_CFG = 0x06; // configuration

class UBX_RxMsg // receiver for the UBX sentences
{ public:
   // most information in the UBX packets is already aligned to 32-bit boundary
   // thus it makes sense to have the packet so aligned when receiving it.
   static const uint8_t MaxWords=10;   // maximum number of 32-bit words (excl. head and tail)
   static const uint8_t MaxBytes=4*MaxWords; // max. number of bytes
   static const uint8_t SyncL=0xB5;    // UBX sync bytes
   static const uint8_t SyncH=0x62;

   union
   { uint32_t Word[MaxWords];          // here we store the UBX packet (excl. head and tail)
     uint8_t  Byte[MaxBytes]; } ;
   uint8_t  Class;                     // Class (01=NAV)
   uint8_t  ID;                        // ID
   uint8_t  Bytes;                     // number of bytes in the packet (excl. head and tail)

  private:
   uint8_t Padding;                    // just to make the structure size be a multiple of 4-bytes
   uint8_t State;                      // bits: 0:loading, 1:complete, 2:locked,
   uint8_t Idx;                        // loading index

   uint8_t CheckA;                     // UBX check sum (two bytes)
   uint8_t CheckB;
   void CheckInit(void) { CheckA=0; CheckB=0; }                   // initialize the checksum
   void CheckPass(uint8_t Byte) { CheckA+=Byte; CheckB+=CheckA; } // pass a byte through the checksum

  public:
   void RecalcCheck(void)
   { CheckInit();
     CheckPass(Class); CheckPass(ID); CheckPass(Bytes); CheckPass(0x00);
     for(uint8_t Idx=0; Idx<Bytes; Idx++) CheckPass(Byte[Idx]); }

   inline void Clear(void) { Idx=0; State=0; CheckInit(); }

   uint8_t isLoading(void) const
     { return State&0x01; }

   uint8_t isComplete(void) const
     { return State&0x02; }

   void ProcessByte(uint8_t RxByte) // pass all bytes through this call and it will build the frame
     {
       if(isComplete()) Clear(); // if already a complete frame, clear it
       switch(Idx)
       { case 0:  // expect SyncL
            if(RxByte!=SyncL) { Clear(); return; }
            State=0x01; break;  // declare "isLoading" state
         case 1: // expect SyncH
            if(RxByte!=SyncH) { Clear(); return; }
            break;
         case 2: // Class
            Class=RxByte; CheckPass(RxByte);
            break;
         case 3: // ID
            ID=RxByte; CheckPass(RxByte);
            break;
         case 4: // LSB of packet length
            Bytes=RxByte; CheckPass(RxByte); if(Bytes>MaxBytes) { Clear(); return; }
            break;
         case 5: // MSB of packet length (expect zero)
            CheckPass(RxByte); if(RxByte!=0) { Clear(); return; }
            break;
         default:                         // past the header, now load the packet content
            uint8_t ByteIdx=Idx-6;
            if(ByteIdx<Bytes)
            { Byte[ByteIdx]=RxByte; CheckPass(RxByte); }
            else if(ByteIdx==Bytes)        // already past the content, now the first checksum byte
            { if(RxByte!=CheckA) { Clear(); return; } }
            else if(ByteIdx==(Bytes+1))    // second checksum byte
            { if(RxByte!=CheckB) { Clear(); return; }
              State=0x02; }                // declare "isComplete" state
            else
            { Clear(); return; }
            break;
       }
       Idx++;
     }

   void Send(void (*SendByte)(char)) const
   { (*SendByte)(SyncL);
     (*SendByte)(SyncH);
     (*SendByte)(Class);
     (*SendByte)(ID);
     (*SendByte)(Bytes);
     (*SendByte)(0x00);
     for(uint8_t Idx=0; Idx<Bytes; Idx++)
     { (*SendByte)(Byte[Idx]); }
     (*SendByte)(CheckA);
     (*SendByte)(CheckB);
   }

   static void SendPoll(uint8_t Class, uint8_t ID, void (*SendByte)(char))
   { (*SendByte)(SyncL);
     (*SendByte)(SyncH);
     (*SendByte)(Class);
     (*SendByte)(ID);
     (*SendByte)(0x00);
     (*SendByte)(0x00);
     uint8_t CheckA = Class;   // pass Class through check sum
     uint8_t CheckB = CheckA;
     CheckA += ID;             // pass ID through check sum
     CheckB += CheckA;
     CheckB += CheckA;         // pass 0x00
     CheckB += CheckA;         // pass 0x00
     (*SendByte)(CheckA);      // send the check sum
     (*SendByte)(CheckB);
   }

   bool isNAV(void) const { return Class==0x01; }
   bool isACK(void) const { return Class==0x05; }
   bool isCFG(void) const { return Class==0x06; }

   bool isNAV_POSLLH (void) const { return isNAV() && (ID==0x02); }
   bool isNAV_STATUS (void) const { return isNAV() && (ID==0x03); }
   bool isNAV_DOP    (void) const { return isNAV() && (ID==0x04); }
   bool isNAV_VELNED (void) const { return isNAV() && (ID==0x12); }
   bool isNAV_TIMEGPS(void) const { return isNAV() && (ID==0x20); }
   bool isNAV_TIMEUTC(void) const { return isNAV() && (ID==0x21); }

   bool isACK_NAK    (void) const { return isACK() && (ID==0x00); }
   bool isACK_ACK    (void) const { return isACK() && (ID==0x01); }

   bool isCFG_PRT    (void) const { return isCFG() && (ID==0x00); }
   bool isCFG_NAV5   (void) const { return isCFG() && (ID==0x24); }
} ;

class UBX_NAV_POSLLH  // 0x01 0x02
{ uint32_t iTOW;      // [ms] Time-of-Week
   int32_t lon;       // [1e-7 deg] Longitude
   int32_t lat;       // [1e-7 deg] Latitude
   int32_t height;    // [mm] height above elipsoid (GPS altitude)
   int32_t hMSL;      // [mm] height above Mean Sea Level
  uint32_t hAcc;      // [mm] horizontal accuracy
  uint32_t vAcc;      // [mm] vertical accuracy
} ;

class UBX_NAV_STATUS // 0x01 0x03
{ uint32_t iTOW;     // [ms] Time-of-Week
  uint8_t  gpsFix;   // Fix type: 0:none, 1=dead reckoning, 2:2-D, 3:3-D, 4:GPS+dead reckoning, 5:time-only
  uint8_t  flags;    // xxxxTWDF => T:Time-of-Week is valid, W:Week-Number is valid, D:Diff. GPS is used, F:Fix valid
  uint8_t  diffStat; // DD => 00:none, 01:PR+PRR corr., 10: PR+PRR+CP corr., 11: high accuracy PR+PRR+CP
  uint8_t  res;      // reserved          PR=Pseudo-Range, PRR=Pseudo-Range Rate
  uint32_t ttff;     // [ms] Time To First Fix
  uint32_t msss;     // [ms] Since Startup
} ;

class UBX_NAV_DOP     // 0x01 0x04
{ uint32_t iTOW;      // [ms] Time-of-Week
  uint16_t gDOP;      // [1/100] geometrical
  uint16_t pDOP;      // [1/100] position
  uint16_t tDOP;      // [1/100] time
  uint16_t vDOP;      // [1/100] vertical
  uint16_t hDOP;      // [1/100] horizontal
  uint16_t nDOP;      // [1/100] north-south
  uint16_t eDOP;      // [1/100] east-west
  uint16_t padding;   // padding for round size
} ;

class UBX_NAV_VELNED  // 0x01 0x12
{ uint32_t iTOW;      // [ms] Time-of-Week
   int32_t velN;      // [cm/s] velocity North
   int32_t velE;      // [cm/s] velocity East
   int32_t velD;      // [cm/s] velocity Down
  uint32_t Speed;     // [cm/s] velocity
  uint32_t gSpeed;    // [cm/s] ground speed (horizontal velocity)
   int32_t heading;   // [1e-5 deg] ground heading
  uint32_t sAcc;      // [cm/s] speed accuracy
  uint32_t cAcc;      // [1e-5 deg] heading accuracy
} ;

class UBX_NAV_TIMEGPS  // 0x01 0x020
{ public:
   uint32_t iTOW;      // [ms] Time-of-Week
    int32_t fTOW;      // [ns] reminder of Time-of-Week
   uint16_t week;
   uint8_t  leapS;
   uint8_t  valid;     // bits: 0:ToW, 1:week, 2:leapS
   uint32_t tAcc;      // [ns]

  public:
   static const uint32_t SecsPerWeek = 7*24*60*60;
   uint8_t Valid(void) const
   { return (valid&0x03)==0x03; }
   uint32_t UnixTime() const
   { return (iTOW+10)/1000 + week*SecsPerWeek + 315964785; } // http://www.andrews.edu/~tzs/timeconv/timedisplay.php
} ;

class UBX_NAV_TIMEUTC  // 0x01 0x21
{ public:
   uint32_t iTOW;      // [ms] Time-of-Week
   uint32_t tAcc;      // [ns] accurary estimate
    int32_t nano;      // [ns]
   uint16_t year;      // 1999..2099
   uint8_t  month;     // 1..12
   uint8_t  day;       // 1..31
   uint8_t  hour;
   uint8_t  min;
   uint8_t  sec;
   uint8_t  valid;    // bits: 0:ToW, 1:WN, 2:UTC
} ;

class UBX_CFG_PRT     // 0x06 0x00
{ public:
   uint8_t  portID;       // 1 or 2
   uint8_t  reserved0;
   uint16_t txReady;
    int32_t mode;         // 00 10x x 11 x 1 xxxx => 0x08D0
   uint32_t baudRate;     // [bps]
    int16_t inProtoMask;  // bit 0:UBX, bit 1:NMEA
    int16_t outProtoMask; // bit 0:UBX, bit 1:NMEA
   uint16_t reserved4;
   uint16_t reserved5;
} ;

class UBX_CFG_MSG         // 0x06 0x01
{ public:
   uint8_t msgClass;
   uint8_t msgID;
   uint8_t rate;          // message send rate
} ;

class UBX_CFG_RATE        // 0x06 0x08
{ public:
   uint16_t measRate;     // [ms] measurement rate
   uint16_t navRate;      // [cycles] = 1
   uint16_t timeRef;      // 0=UTC, 1=GPS
} ;

class UBX_CFG_NAV5        // 0x06 0x24
{ public:
   uint16_t mask;         // bit #0 = apply dynamic mode settings, #1 = apply min. elev. settings, #2 = apply fix mode settings
   uint8_t  dynModel;     // 6 = airborne 1g, 7 = 2g, 8 = 4g
   uint8_t  fixMode;      // 1=2D only, 2=3D only, 3=auto 2/3D
   int32_t  fixAlt;       // [0.01m]
  uint32_t  fixAltVar;    // [0.001m]
   int8_t   minElev;      // [deg] minimum satelite elevation
  uint8_t   drLimit;      // [sec] Dead Reconning time limit
  uint16_t  pDop;
  uint16_t  tDop;
  uint16_t  pAcc;         // [m]
  uint16_t  tAcc;         // [m]
  uint8_t   staticHoldThres; // [cm/s]
  uint8_t   dgpsTimeout;     // [s]
  uint32_t  reserved2;
  uint32_t  reserved3;
  uint32_t  reserved4;
} ;

#endif // __UBX_H__
