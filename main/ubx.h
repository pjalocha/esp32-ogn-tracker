#ifndef __UBX_H__
#define __UBX_H__

class UBX_NAV_POSLLH  // 0x01 0x02
{ public:
   uint32_t iTOW;      // [ms] Time-of-Week
    int32_t lon;       // [1e-7 deg] Longitude
    int32_t lat;       // [1e-7 deg] Latitude
    int32_t height;    // [mm] height above elipsoid (GPS altitude)
    int32_t hMSL;      // [mm] height above Mean Sea Level
   uint32_t hAcc;      // [mm] horizontal accuracy
   uint32_t vAcc;      // [mm] vertical accuracy
} ;

class UBX_NAV_STATUS // 0x01 0x03
{ public:
   uint32_t iTOW;     // [ms] Time-of-Week
   uint8_t  gpsFix;   // Fix type: 0:none, 1=dead reckoning, 2:2-D, 3:3-D, 4:GPS+dead reckoning, 5:time-only
   uint8_t  flags;    // xxxxTWDF => T:Time-of-Week is valid, W:Week-Number is valid, D:Diff. GPS is used, F:Fix valid
   uint8_t  diffStat; // DD => 00:none, 01:PR+PRR corr., 10: PR+PRR+CP corr., 11: high accuracy PR+PRR+CP
   uint8_t  res;      // reserved          PR=Pseudo-Range, PRR=Pseudo-Range Rate
   uint32_t ttff;     // [ms] Time To First Fix
   uint32_t msss;     // [ms] Since Startup
} ;

class UBX_NAV_DOP     // 0x01 0x04
{ public:
   uint32_t iTOW;      // [ms] Time-of-Week
   uint16_t gDOP;      // [1/100] geometrical
   uint16_t pDOP;      // [1/100] position
   uint16_t tDOP;      // [1/100] time
   uint16_t vDOP;      // [1/100] vertical
   uint16_t hDOP;      // [1/100] horizontal
   uint16_t nDOP;      // [1/100] north-south
   uint16_t eDOP;      // [1/100] east-west
   uint16_t padding;   // padding for round size
} ;

class UBX_NAV_SOL     // 0x01 0x06
{ public:
   uint32_t iTOW;      // [ms] Time-of-Week
    int32_t fTOW;      // [ns] Time-of-Week
    int16_t Week;      // [week]
    uint8_t gpsFix;    // 0=none, 1=DR, 2=2D, 3=3D, 4=DR+GPS, 5=time-only
    uint8_t flags;
    int32_t ecefX;     // [cm]   ECEF position
    int32_t ecefY;     // [cm]
    int32_t ecefZ;     // [cm]
   uint32_t pAcc;      // [cm]   position accuracy
    int32_t ecefVX;    // [cm/s] ECEF velocity
    int32_t ecefVY;    // [cm/s]
    int32_t ecefVZ;    // [cm/s]
   uint32_t sAcc;      // [cm/s] speed accuracy
   uint16_t PDOP;      // [1/100]
    uint8_t reserved1; //
    uint8_t numSV;     // [satellites]
    uint8_t reserved2[4]; //
} ;

class UBX_NAV_PVT     // 0x01 0x07
{ public:
   uint32_t iTOW;      // [ms]
   uint16_t year;
   uint8_t  month;
   uint8_t  day;
   uint8_t  hour;
   uint8_t  min;
   uint8_t  sec;
   uint8_t  valid;
   uint32_t tAcc;
    int32_t nano;
   uint8_t  fixType;
   uint8_t  flags;
   uint8_t  flags2;
   uint8_t  numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
   uint32_t hAcc;
   uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
   uint32_t sAcc;
   uint32_t headAcc;
   uint16_t pDOP;
   uint8_t  reserved1[6];
    int32_t headVeh;
   uint8_t  reserved2[4];
} ;

class UBX_NAV_VELNED  // 0x01 0x12
{ public:
   uint32_t iTOW;      // [ms] Time-of-Week
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

class UBX_NAV_TIMEUTC     // 0x01 0x21
{ public:
   uint32_t iTOW;         // [ms] Time-of-Week
   uint32_t tAcc;         // [ns] accurary estimate
    int32_t nano;         // [ns]
   uint16_t year;         // 1999..2099
   uint8_t  month;        // 1..12
   uint8_t  day;          // 1..31
   uint8_t  hour;
   uint8_t  min;
   uint8_t  sec;
   uint8_t  valid;        // bits: 0:ToW, 1:WN, 2:UTC
} ;

class UBX_RXM_PMREQ       // 0x02 0x41
{ public:
   uint32_t duration;     // [ms]
   uint32_t flags;        // bit #1 = enter backup mode
} ;

class UBX_CFG_CFG
{ public:
   uint32_t clearMask;
   uint32_t  saveMask;
   uint32_t  loadMask;
   uint8_t deviceMask;
} ;

class UBX_CFG_PRT         // 0x06 0x00
{ public:
   uint8_t  portID;       // 0 = I2C, 1 = UART1, 2 = UART2, 3 = USB, 4 = SPI
   uint8_t  reserved1;
   uint16_t txReady;      // 0 = disable this feature
    int32_t mode;         // 00 10x x 11 x 1 xxxx => 0x08D0
   uint32_t baudRate;     // [bps]
    int16_t inProtoMask;  // bit 0:UBX, bit 1:NMEA
    int16_t outProtoMask; // bit 0:UBX, bit 1:NMEA
   uint16_t flags;        // bit 1:extendedTxTimeout
   uint16_t reserved2;
  public:
   // void setBaudRate(uint32_t BaudRate)
   // { }
} ;

class UBX_CFG_MSG         // 0x06 0x01
{ public:
   uint8_t msgClass;      // 0xF0:00=GGA, 0xF0:02=GSA, 0xF0:03=GSV, 0xF0:04=RMC, 0xF0:41=TXT
   uint8_t msgID;
   uint8_t rate;          // message send rate
} ;

/*
class UBX_CFG_MSG         // 0x06 0x01
{ public:
   uint8_t msgClass;      // 0xF0:00=GGA, 0xF0:02=GSA, 0xF0:04=RMC, 0xF0:41=TXT
   uint8_t msgID;
   uint8_t rate[6];       // message send rate
} ;
*/

class UBX_CFG_RATE        // 0x06 0x08
{ public:
   uint16_t measRate;     // [ms] measurement rate
   uint16_t navRate;      // [cycles] = 1
   uint16_t timeRef;      // 0=UTC, 1=GPS
} ;

class UBX_CFG_SBAS        // 0x06 0x16
{ public:
   uint8_t mode;          // #0 = enabled, #1 = test
   uint8_t usage;         // #0 = range, #1 = diffCorr, #2 = integrity
   uint8_t maxSBAS;       // number of channels: 0..3
   uint8_t scanmode2;     // #0 = PRN152, ... #6  = PRN158
  uint32_t scanmode1;     // #0 = PRN120, ... #31 = PRN151
} ;                       // when polled from M6:  01 03 03 00 51 62 06 00 => 129, 130, 137, 141, 142, 144, 148, 150
                          // for auto-select mode: 01 07 03 00 00 00 00 00 => in UE it finds and uses: 123, 125, 136

// for GPS:     0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01 (enable)
// for SBAS:    0x01, 0x02, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01 (enable)
// for BeiDou:  0x03, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01 (disable)
// for QZSS:    0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01 (disable)
// for Glonass: 0x06, 0x04, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01 (disable)
// for Galileo: 0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01 (disable)
class UBX_CFG_GNSS_Block
{ public:
   uint8_t gnssId;        // system identifier: 0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou, 4=IMES, 5=QZSS, 6=Glonass
   uint8_t resTrkCh;      // number of reserved tracking channels
   uint8_t maxTrkCh;      // 
   uint8_t reserved;
   uint32_t flags;        // bit #0 = enable, bit #16..23 = sigCfgMask
} ;

class UBX_CFG_GNSS        // 0x06 0x3E
{ public:
   uint8_t msgVer;        // version = 0
   uint8_t numTrkChHw;    // number of hardware tracking channels (read-only)
   uint8_t numTrkChUse;   // number of tracker channels to use (0xFF = use max. allowed by hardware)
   uint8_t numConfigBlocks; // number of config. blocks which follows: 8 bytes per block
   UBX_CFG_GNSS_Block Block[6];   // 5 or 6 blocks
} ;

class UBX_CFG_NAV5        // 0x06 0x24
{ public:
   uint16_t mask;            // bit #0 = apply dynamic mode settings, #1 = apply min. elev. settings, #2 = apply fix mode settings
   uint8_t  dynModel;        // 6 = airborne 1g, 7 = 2g, 8 = 4g
   uint8_t  fixMode;         // 1=2D only, 2=3D only, 3=auto 2/3D
   int32_t  fixAlt;          // [0.01m]
  uint32_t  fixAltVar;       // [0.001m]
   int8_t   minElev;         // [deg] minimum satelite elevation
  uint8_t   drLimit;         // [sec] Dead Reconning time limit
  uint16_t  pDop;
  uint16_t  tDop;
  uint16_t  pAcc;            // [m]
  uint16_t  tAcc;            // [m]
  uint8_t   staticHoldThres; // [cm/s]
  uint8_t   dgpsTimeout;     // [s]
  uint8_t   cnoThreshNumSVs; //
  uint8_t   cnoThresh;       // [dBHz]
  uint8_t   reserved2[2];
  uint32_t  reserved3;
  uint32_t  reserved4;
 public:
  void setDynModel(uint8_t DynModel) { mask = 0x0001; dynModel=DynModel; } // only change the dynamic model
} ;

// UBX Class packet numbers
const uint8_t UBX_NAV = 0x01; // navigation
const uint8_t UBX_ACK = 0x05; // acknoledgement of configuration
const uint8_t UBX_CFG = 0x06; // configuration

class UBX_RxMsg // receiver for the UBX sentences
{ public:
   // most information in the UBX packets is already aligned to 32-bit boundary
   // thus it makes sense to have the packet so aligned when receiving it.
   static const uint8_t MaxWords=32; // 10;   // maximum number of 32-bit words (excl. head and tail)
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

   static void Send(uint8_t Class, uint8_t ID, void (*SendByte)(char), const uint8_t *Data=0, uint8_t DataLen=0)
   { (*SendByte)(SyncL);
     (*SendByte)(SyncH);
     (*SendByte)(Class);
     uint8_t CheckA = Class;   // pass Class through check sum
     uint8_t CheckB = CheckA;
     (*SendByte)(ID);
     CheckA += ID;             // pass ID through check sum
     CheckB += CheckA;
     (*SendByte)(DataLen);
     CheckA += DataLen;
     CheckB += CheckA;         // pass DataLen LSB
     (*SendByte)(0x00);
     CheckB += CheckA;         // pass DataLen MSB = 0x00
     if(Data)
     { for(uint8_t Idx=0; Idx<DataLen; Idx++)
       { (*SendByte)(Data[Idx]);
         CheckA += Data[Idx];
         CheckB += CheckA; }
     }
     (*SendByte)(CheckA);      // send the check sum
     (*SendByte)(CheckB);
   }

   bool isNAV(void) const { return Class==0x01; }
   bool isACK(void) const { return Class==0x05; }
   bool isCFG(void) const { return Class==0x06; }

   bool isNAV_POSLLH (void) const { return isNAV() && (ID==0x02) && (Bytes==sizeof(UBX_NAV_POSLLH)); }
   bool isNAV_STATUS (void) const { return isNAV() && (ID==0x03); }
   bool isNAV_DOP    (void) const { return isNAV() && (ID==0x04); }
   bool isNAV_SOL    (void) const { return isNAV() && (ID==0x06) && (Bytes==sizeof(UBX_NAV_SOL)); }
   bool isNAV_VELNED (void) const { return isNAV() && (ID==0x12); }
   bool isNAV_TIMEGPS(void) const { return isNAV() && (ID==0x20); }
   bool isNAV_TIMEUTC(void) const { return isNAV() && (ID==0x21) && (Bytes==sizeof(UBX_NAV_TIMEUTC)); }

   bool isACK_NAK    (void) const { return isACK() && (ID==0x00); }
   bool isACK_ACK    (void) const { return isACK() && (ID==0x01); }

   bool isCFG_PRT    (void) const { return isCFG() && (ID==0x00); }
   bool isCFG_SBAS   (void) const { return isCFG() && (ID==0x16); }
   bool isCFG_NAV5   (void) const { return isCFG() && (ID==0x24); }
} ;


#endif // __UBX_H__
