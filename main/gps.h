
#include "hal.h"

#include "timesync.h"

#include "ogn.h"

#include "lowpass2.h"

const  uint8_t GPS_PosPipeSize         = 4; // number of GPS positions held in a pipe

extern          uint32_t GPS_FatTime;       // [2 sec] UTC time in FAT format (for FatFS)
extern           int32_t GPS_Altitude;      // [0.1m] altitude (height above Geoid)
extern           int32_t GPS_Latitude;      // [0.0001/60 deg]
extern           int32_t GPS_Longitude;     // [0.0001/60 deg]
extern           int16_t GPS_GeoidSepar;    // [0.1m]
extern          uint16_t GPS_LatCosine;     // [1.0/(1<<12)] Latitude Cosine for distance calculations
extern          uint32_t GPS_TimeSinceLock; // [sec] time since GPS has a valid lock
extern          uint16_t GPS_PosPeriod;     // [0.01sec] how often (which period) the GPS/MAV is sending the positions

typedef union
         { uint8_t  Flags;
           struct
           { bool       NMEA:1; // got at least one valid NMEA message
             bool        UBX:1; // got at least one valid UBX message
             bool        MAV:1; // got at least one valid MAV message
             bool        PPS:1; // got at least one PPS signal
             bool BaudConfig:1; // baudrate is configured
             bool ModeConfig:1; // mode is configured
             bool           :1; //
             bool           :1; //
           } ;
         } Status;

extern Status GPS_Status;

uint32_t GPS_getBaudRate(void);             // [bps]

GPS_Position *GPS_getPosition(void);
GPS_Position *GPS_getPosition(int8_t Sec);                                                  // return GPS position for given Sec
GPS_Position *GPS_getPosition(uint8_t &BestIdx, int16_t &BestRes, int8_t Sec, int8_t Frac); // return GPS position closest to the given Sec.Frac

int16_t GPS_AverageSpeed(void);             // [0.1m/s] calc. average speed based on most recent GPS positions

#ifdef __cplusplus
  extern "C"
#endif
void vTaskGPS(void* pvParameters);

