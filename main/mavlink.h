#ifndef __MAVLINK_H__
#define __MAVLINK_H__

#include <stdio.h>
#include <stdint.h>

#include "intmath.h"

// ============================================================================

// https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml

// System-ID


// Component-ID
const uint8_t MAV_COMP_ID_AUTOPILOT1      =   1; // auto-pilot
const uint8_t MAV_COMP_ID_ADSB            = 156; // ADS-B receiver
const uint8_t MAV_COMP_ID_GPS             = 220;

// Message-ID
const uint8_t MAV_ID_HEARTBEAT             =  0; //
const uint8_t MAV_ID_SYS_STATUS            =  1;
const uint8_t MAV_ID_SYSTEM_TIME           =  2;
const uint8_t MAV_ID_GPS_RAW_INT           = 24; // GPS position estimate
const uint8_t MAV_ID_RAW_IMU               = 27;
const uint8_t MAV_ID_SCALED_PRESSURE       = 29; //
const uint8_t MAV_ID_ATTITUDE              = 30;
const uint8_t MAV_ID_GLOBAL_POSITION_INT   = 33; // combined position estimate
const uint8_t MAV_ID_RC_CHANNELS_RAW       = 35;
const uint8_t MAV_ID_SERVO_OUTPUT_RAW      = 36;
const uint8_t MAV_ID_MISSION_CURRENT       = 42;
const uint8_t MAV_ID_NAV_CONTROLLER_OUTPUT = 62;
const uint8_t MAV_ID_VFR_HUD               = 74;
const uint8_t MAV_ID_TIMESYNC             = 111;
const uint8_t MAV_ID_HIL_GPS              = 113;
const uint8_t MAV_ID_ADSB_VEHICLE         = 246; // traffic information sent by an ADS-B receiver
const uint8_t MAV_ID_COLLISION            = 247; // collision threat detected by auto-pilot
const uint8_t MAV_ID_STATUSTEXT           = 253;
const uint8_t MAV_ID_DEBUG                = 254;

// --------------------------------------------------------------------------------

class MAV_HEARTBEAT
{ public:
   uint32_t     custom_mode;
   uint8_t             type; // MAV-aircraft-type: 1=fixed wing, 2=quadrotor, 7=airship, 8=balloon, 13=hexarotor
   uint8_t        autopilot; // 3=ArduPilotMega, 4=OpenPilot, 13=PX4
   union
   { uint8_t      base_mode;
     struct
     { bool         custom:1;
       bool           test:1;
       bool     autonomous:1;
       bool         guided:1;
       bool      stabilize:1;
       bool hil_simulation:1;
       bool remote_control:1;
       bool          armed:1;
     } ;
   } ;
   uint8_t    system_status; // 1=boot, 2=calib., 3=standby, 4=active,  5=critical, 6=emergency, 7=power-off, 8=terminate
   uint8_t  mavlink_version;
  public:
   void Print(void) const
   { printf("HEARTBEAT: t%X v%d\n", type, mavlink_version); }
} ;

class MAV_SYSTEM_TIME
{ public:
   uint64_t time_unix_usec;  // [usec]
   uint32_t   time_boot_ms;  // [ms]
  public:
   void Print(void) const
   { printf("SYSTEM_TIME: %14.3f-%8.3f [sec]\n", 1e-6*time_unix_usec, 1e-3*time_boot_ms); }
} ;

class MAV_SYS_STATUS
{ public:
   uint32_t onboard_control_sensors_present;
   uint32_t onboard_control_sensors_enabled;
   uint32_t onboard_control_sensors_health;
   uint16_t load;                             // [0.1%]
   uint16_t battery_voltage;                  // [mV]
    int16_t battery_current;                  // [10mA]
   uint16_t error_comm;
   uint16_t error_comm1;
   uint16_t error_comm2;
   uint16_t error_comm3;
   uint16_t error_comm4;
   uint8_t battery_remaining;                 // [%]
   void Print(void) const
   { printf("SYS_STATUS: %3.1f%% %5.3fV %+5.2fA\n", 0.1*load, 1e-3*battery_voltage, 0.01*battery_current); }
} ;

class MAV_GPS_RAW_INT
{ public:
   uint64_t          time_usec;  // [usec]       Time
    int32_t                lat;  // [1e-7deg]    Latitude
    int32_t                lon;  // [1e-7deg]    Longitude
    int32_t                alt;  // [mm]         Altitude AMSL
   //  int32_t   alt_ellipsoid;  // [mm]
   // uint32_t           h_acc;  // [mm]
   // uint32_t           v_acc;  // [mm]
   // uint32_t         vel_acc;  // [mm/s]
   // uint32_t         hdg_acc;  // [0.00001deg]
   uint16_t                eph;  // [0.01]       HDOP
   uint16_t                epv;  // [0.01]       VDOP
   uint16_t                vel;  // [0.01m/s]    Valocity
   uint16_t                cog;  // [0.01deg]    course-over-Ground
   uint8_t            fix_type;  // []           0=no GPS, 1=no fix, 2=2D, 3=3D, 4=DGPS
   uint8_t  satellites_visible;  // []           Number of satellites
  public:
   void Print(void) const
   { printf("GPS_RAW_INT: [%+9.5f, %+10.5f]deg %+5.1fm %3.1fm/s %05.1fdeg %d/%dsat\n",
           1e-7*lat, 1e-7*lon, 1e-3*alt, 0.01*vel, 0.01*cog, fix_type, satellites_visible); }
} ;

class MAV_GLOBAL_POSITION_INT
{ public:
   uint32_t  time_boot_ms;  // [ms]
    int32_t           lat;  // [1e-7deg]
    int32_t           lon;  // [1e-7deg]
    int32_t           alt;  // [mm]
    int32_t  relative_alt;  // [mm]
    int16_t            vx;  // [cm]
    int16_t            vy;  // [cm]
    int16_t            vz;  // [cm]
    int16_t           hdg;  // [0.01deg]
  public:
   void Print(void) const
   { uint16_t Track = IntAtan2(vy, vx);
     printf("GLOBAL_POSITION_INT: [%+9.5f, %+10.5f]deg %5.1f(%+4.1f)m %3.1fm/s %05.1f/%05.1fdeg %+4.1fm/s\n",
           1e-7*lat, 1e-7*lon, 1e-3*alt, 1e-3*relative_alt,
           0.01*IntSqrt((int32_t)vx*vx+(int32_t)vy*vy), 360.0/0x10000*Track, 0.01*hdg,
           0.01*vz); }
} ;

class MAV_SCALED_PRESSURE
{ public:
   uint32_t time_boot_ms; // [msec]
    float      press_abs; // [hPa]
    float     press_diff; // [hPa]
   int16_t   temperature; // [0.01degC]
  public:
   void Print(void) const
   { printf("SCALED_PRESSURE: %8.3f [sec] %7.2f %+4.2f [hPa] %+5.2f [degC]\n", 1e-3*time_boot_ms, 2*press_abs, 2*press_diff, 0.01*temperature); }   
} ;

class MAV_ADSB_VEHICLE    // this message is sent by ADS-B or other traffic receiver
{ public:
   uint32_t ICAO_address; // ICAO ID (for ADS-B), other ID's for FLARM/OGN/...
    int32_t          lat; // [1e-7deg]
    int32_t          lon; // [1e-7deg]
    int32_t     altitude; // [mm]
   uint16_t      heading; // [0.01deg]
   uint16_t hor_velocity; // [cm/sec]
    int16_t ver_velocity; // [cm/sec]
   union
   { uint16_t        flags; // validity: 1=coord. 2=alt. 4=heading 8=velocity 16=callsign 32=squawk 64=simulated
     struct
     { bool    CoordValid:1;
       bool      AltValid:1;
       bool  HeadingValid:1;
       bool CallsignValid:1;
       bool VelocityValid:1;
       bool   SquawkValid:1;
       bool   isSimulated:1;
     } ;
   } ;
   uint16_t       squawk;
   uint8_t altitude_type; // 0 = pressure/QNH, 1 = GPS
   uint8_t   callsign[9]; // 8+null
   uint8_t   emiter_type; // 0=no-info, 1=light, 2=small. 3=large, 4=high-vortex, 5=heavy, 6=manuv, 7=rotor, 9=glider, 10=balloon/airship, 11=parachute, 12=ULM, 14=UAV, 15=space, 19=obstacle
   uint8_t          tslc; // [sec] time since last communication
  public:
   void Print(void) const
   { printf("ADSB_VEHICLE: %02X:%08lX [%+9.5f, %+10.5f]deg %5.1fm %3.1fm/s %05.1fdeg %+4.1fm/s\n",
            (int)emiter_type, (long int)ICAO_address, 1e-7*lat, 1e-7*lon, 1e-3*altitude, 0.01*hor_velocity, 360.0/0x10000*heading, 0.01*ver_velocity); }
} ;

class COLLISION                    // this message is sent by the autopilot when it detects a collision threat
{ public:
   uint32_t                    id;
   float    time_to_minimum_delta;
   float   altitude_minimum_delta;
   float horizontal_minimum_delta;
   uint8_t                    src;
   uint8_t                 action;
   uint8_t           threat_level;
} ;

// =============================================================================

// https://groups.google.com/forum/#!topic/mavlink/-ipDgVeYSiU
static const uint8_t mavlink_message_crcs[256] = {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 153, 183, 51, 59, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 0, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 34, 71, 15, 0, 0, 0, 0, 0, 0, 0, 163, 105, 0, 35, 0, 0, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 0, 8, 204, 49, 170, 44, 83, 46, 0};

class MAV_RxMsg // receiver for the MAV messages
{ public:
   static const uint8_t MaxBytes = 65; // max. number of bytes
   static const uint8_t Sync   = 0xFE; // MAV sync byte

   uint16_t Check;
   uint8_t  Byte[MaxBytes];
   uint8_t  Idx;

  public:
   void Clear(void) { Idx=0; CheckInit(Check); }

   uint8_t getLen   (void) const { return Byte[1]; }  // Payload length (not whole packet length)
   uint8_t getSeq   (void) const { return Byte[2]; }  // Sequence (increments with every new message)
   uint8_t getSysID (void) const { return Byte[3]; }  // System-ID
   uint8_t getCompID(void) const { return Byte[4]; }  // Component-ID
   uint8_t getMsgID (void) const { return Byte[5]; }  // Message-ID
   void  *getPayload(void) const { return (void *)(Byte+6); }   // message (pointer to) Payload

   void Print(bool Ext=1) const
   { printf("MAV[%2d:%2d] [%02X] %02X:%02X %3d:", Idx, getLen(), getSeq(), getSysID(), getCompID(), getMsgID() );
     if( (getMsgID()==MAV_ID_STATUSTEXT) && isComplete() )
     { printf("(%d) %s\n", Byte[6], Byte+7); }
     else
     { for(uint8_t i=6; i<Idx; i++)
         printf(" %02X", Byte[i]);
       printf(" %04X (%c)\n", Check, isComplete()?'+':'-');
       if(Ext)
       {      if(getMsgID()==MAV_ID_HEARTBEAT              ) { ((const MAV_HEARTBEAT               *)getPayload())->Print(); }
         else if(getMsgID()==MAV_ID_SYS_STATUS             ) { ((const MAV_SYS_STATUS              *)getPayload())->Print(); }
         else if(getMsgID()==MAV_ID_SYSTEM_TIME            ) { ((const MAV_SYSTEM_TIME             *)getPayload())->Print(); }
         else if(getMsgID()==MAV_ID_SCALED_PRESSURE        ) { ((const MAV_SCALED_PRESSURE         *)getPayload())->Print(); }
         else if(getMsgID()==MAV_ID_GPS_RAW_INT            ) { ((const MAV_GPS_RAW_INT             *)getPayload())->Print(); }
         else if(getMsgID()==MAV_ID_GLOBAL_POSITION_INT    ) { ((const MAV_GLOBAL_POSITION_INT     *)getPayload())->Print(); }
         else if(getMsgID()==MAV_ID_ADSB_VEHICLE           ) { ((const MAV_ADSB_VEHICLE            *)getPayload())->Print(); }
       }
     }
   }

   uint8_t ProcessByte(uint8_t RxByte)                       // process a single byte: add to the message or reject
   { // printf("Process[%2d] 0x%02X\n", Idx, RxByte);
     if(Idx==0)                                              // the very first byte: we only accept SYNC
     { if(RxByte==Sync) { Byte[Idx++]=RxByte; return 1; }
                   else {                     return 0; }
     }
     if(Idx==1)                                              // second byte: payload length
     { Byte[Idx++]=RxByte; CheckPass(Check, RxByte); return 1; }
     if(Idx>=MaxBytes) { Clear(); return 0; }                // take following bytes
     Byte[Idx++]=RxByte; if(Idx<(getLen()+7)) CheckPass(Check, RxByte);
     if(Idx==(getLen()+8))
     { CheckPass(Check, mavlink_message_crcs[getMsgID()]);
       // printf("[%2d]", Idx); for(uint8_t i=0; i<Idx; i++) printf(" %02X", Byte[i]); printf(" %04X\n", Check);
       if( ((Check&0xFF)!=Byte[Idx-2]) || ((Check>>8)!=Byte[Idx-1]) ) { Clear(); return 0; }
     }
     return 1; }

   uint8_t isComplete(void) const { return Idx==(getLen()+8); }

   void static CheckInit(uint16_t &Check) { Check=0xFFFF; }
   void static CheckPass(uint16_t &Check, uint8_t Byte)
   { uint8_t Tmp = Byte ^ (uint8_t)(Check&0xFF);
     Tmp ^= (Tmp<<4);
     Check = (Check>>8) ^ ((uint16_t)Tmp<<8) ^ ((uint16_t)Tmp<<3) ^ (Tmp>>4);
     // printf("CheckPass: 0x%02X => 0x%04X\n", Byte, Check);
   }

   static uint8_t Send(uint8_t Len, uint8_t Seq, uint8_t SysID, uint8_t CompID, uint8_t MsgID, const uint8_t *Payload, void (*SendByte)(char) )
   { uint16_t Check; CheckInit(Check);
     (*SendByte)(Sync);
     (*SendByte)(Len);    CheckPass(Check, Len);
     (*SendByte)(Seq);    CheckPass(Check, Seq);
     (*SendByte)(SysID);  CheckPass(Check, SysID);
     (*SendByte)(CompID); CheckPass(Check, CompID);
     (*SendByte)(MsgID);  CheckPass(Check, MsgID);
     for(uint8_t Idx=0; Idx<Len; Idx++)
     { (*SendByte)(Payload[Idx]); CheckPass(Check, Payload[Idx]); }
     CheckPass(Check, mavlink_message_crcs[MsgID]);
     (*SendByte)(Check&0xFF); (*SendByte)(Check>>8);
     return 8+Len; }

    uint8_t Send(void (*SendByte)(char)) const
    { return Send(Byte[1], Byte[2], Byte[3], Byte[4], Byte[5], Byte+6, SendByte); }

} ;

// ============================================================================

#endif // __MAVLINK_H__
