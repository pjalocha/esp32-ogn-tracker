#ifndef __LOKOUT_H__
#define __LOKOUT_H__

#include <stdio.h>

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "intmath.h"

// #define DEBUG_PRINT

#include "gdl90.h"
#include "relpos.h"

// =======================================================================================================

class LookOut_Target           // describes a flying aircrafts
{ public:
   union
   { uint32_t        ID;      // ID of the target = aircraft ID
     struct
     { uint32_t  Address:24;  // 24-bit address
       uint8_t  AddrType: 8;  // ADS-L address-type 
     } ;
   } ;
   Acft_RelPos     Pos;        // Position relative to the reference Lat/Lon/Alt
    int8_t        Pred;        // [0.5sec] amount of time by which own position has been predicted/extrapolated
   uint8_t     GpsPrec;        // GPS position error (includes prediction errors)

   uint8_t     SysMask;        // bit mask for RF systems received: 0=FLR, 1=OGN, 2=PilotAware, 3=FANET, 4=ADS-L, 5=ADS-B

   uint8_t AcftType;           // ADS-B, ADSL or FLARN/OGN aircraft-type
   char Call[11];

   union
   { uint8_t Flags;            // single-bit flags
     struct
     { // bool   isMoving   :1;   // is a moving target
       bool   isTracked  :1;   // is being tracked or only stored
       // bool   isHidden   :1;   // hidden track, should normally not be revealed
       // bool   hasStdAlt  :1;   // has pressure StdAlt
       bool   Reported   :1;   // this target has already been reported with $PFLAA or GDL90
       bool   Alloc      :1;   // is allocated or not (a free slot, where a new target can go into)
     } ;
   } ;

   union
   { uint32_t    Rank;          //        rank: lowest means shorter time margin, shorter distance margin thus bigger thread
     struct
     { uint16_t DistMargin;     // [0.5m] remaining safety margin: if positive, then considered not a thread at all
       uint8_t  TimeMargin;     // [0.5s] time to target (if no distance margin left)
       uint8_t  WarnLevel;      // assigned warning level: 0, 1, 2 or 3
     } ;
   } ;

   int16_t        dX;        // [0.5m]   relative position of target
   int16_t        dY;        // [0.5m]
   int16_t        dZ;        // [0.5m]

   int16_t        Vx;        // [0.5m/s] relative speed of target
   int16_t        Vy;        // [0.5m/s]
   int16_t        Vz;        // [0.5m/s]

   // int16_t        dStdAlt;   // [0.5m]
   // int16_t        Ax;        // [1/16m/s^2] relative acceleration of target
   // int16_t        Ay;        // [1/16m/s^2]

  uint16_t   HorDist;        // [0.5m]   relative hor. distance to target
   int16_t  MissTime;        // [0.5s]   estimated closest approach time
  uint16_t  MissDist;        // [0.5m]   estimated closest approach distance

  public:
   void Clear(void) { Pred=0; Flags=0; HorDist=0; MissDist=0; Call[0]=0; Rank=0xFFFF; }

   // uint16_t HorRelSpeed(void) const { }

   uint32_t DistSqr(void) const { return (int32_t)dX*dX + (int32_t)dY*dY + (int32_t)dZ*dZ; } // [0.25m^2]  Distance-square to the Target
   uint32_t VelSqr (void) const { return (int32_t)Vx*Vx + (int32_t)Vy*Vy + (int32_t)Vz*Vz; } // [0.5m/s^2] Relative velocity square of the Target

   void calcVel(Acft_RelPos &RefPos)                           // calc. relative target velocity against a reference position
   { Pos.getSpeedVector(Vx, Vy);                               // get horizontal speed vector from Speed and Heading
     int16_t rVx, rVy; RefPos.getSpeedVector(rVx, rVy);        // same for the reference position
     Vx -= rVx; Vy -=rVy;                                      // difference
     Vz = Pos.Climb-RefPos.Climb; }                            // vertical different

    int16_t getBearing    (void) const { return IntAtan2(dY, dX); }                            // bearing to the target
   uint16_t getHorDist    (void) const { return Acft_RelPos::FastDistance(dX, dY); }           // relative horizontal distance to the target
   uint16_t getRelHorSpeed(void) const { return Acft_RelPos::FastDistance(Vx, Vy); }           // relative horiz. speed of the target

   void Print(void) const
   { if(Call[0]) printf("%-8s", Call);
           else  printf("%08X", ID);
     printf("/%+5.1fs/%7.1fm/%7.1fm/%5.1fs/%5.1fm/%+5.1fs/w%d",
                0.5*Pred, 0.5*DistMargin, 0.5*HorDist, 0.5*TimeMargin, 0.5*MissDist, 0.5*MissTime, WarnLevel);
     // printf(" [%+7.1f,%+7.1f,%+7.1f]m [%+5.1f,%+5.1f,%+5.1f]m/s", 0.5*dX, 0.5*dY, 0.5*dZ, 0.5*Vx, 0.5*Vy, 0.5*Vz);
     // printf(" [%+5.2f,%+5.2f]m/s^-2", 0.0625*Ax, 0.0625*Ay);
     Pos.Print(); }

   uint8_t Print(char *output)
   { uint8_t Len=0;

     return Len; }

   uint8_t WritePFLAA(char *NMEA)
   { uint8_t Len=0;
     Len+=Format_String(NMEA+Len, "$PFLAA,");                      // sentence name and alarm-level (but no alarms for trackers)
     NMEA[Len++]='0'+WarnLevel;                                    // warning level
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, dX/2, 1, 0, 1);                          // [m] distance in Latitude
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, dY/2, 1, 0, 1);                          // [m] distance in longitude
     NMEA[Len++]=',';
     Len+=Format_SignDec(NMEA+Len, dZ/2, 1, 0, 1);                          // [m] relative altitude
     NMEA[Len++]=',';
     uint8_t AddrType = (ID>>24)&0x03;
// #ifdef WITH_SKYDEMON                                               // SkyDemon hack which accepts only 1 or 2
     if(AddrType!=1) AddrType=2;
// #endif
     NMEA[Len++]='0'+AddrType;                                     // address-type (3=OGN, but not accepted by SkyDemon)
     NMEA[Len++]=',';
     uint32_t Addr = ID&0xFFFFFF;                                  // [24-bit] address
     Len+=Format_Hex(NMEA+Len, (uint8_t)(Addr>>16));               // 24-bit address: RND, ICAO, FLARM, OGN
     Len+=Format_Hex(NMEA+Len, (uint16_t)Addr);
     NMEA[Len++]=',';
     // Len+=Format_UnsDec(NMEA+Len, ((uint32_t)Pos.Heading*225+0x800)>>12, 4, 1); // [deg] heading (by GPS)
     Len+=Format_UnsDec(NMEA+Len, ((uint32_t)Pos.Heading*45+0x1000)>>13);  // [deg] heading - without decimal part
     NMEA[Len++]=',';
     // Len+=Format_SignDec(NMEA+Len, ((int32_t)Pos.Turn*225+0x800)>>12, 2, 1); // [deg/sec] turn rate
     if(Pos.hasTurn) Len+=Format_SignDec(NMEA+Len, ((int32_t)Pos.Turn*45+0x1000)>>13, 1, 0, 1); // [deg/s] turning rate - without decimal part
     NMEA[Len++]=',';
     // Len+=Format_UnsDec(NMEA+Len, (uint32_t)Pos.Speed*5, 2, 1);              // [approx. m/s] ground speed
     Len+=Format_UnsDec(NMEA+Len, (uint32_t)Pos.Speed/2, 1);              // [approx. m/s] ground speed - without decimal
     NMEA[Len++]=',';
     if(Pos.hasClimb) Len+=Format_SignDec(NMEA+Len, (int32_t)Pos.Climb*5, 2, 1, 1); // [m/s] climb/sink rate - with decimal part
     NMEA[Len++]=',';
     NMEA[Len++]=HexDigit(AcftType);                                // [0..F] aircraft-type: 1=glider, 2=tow plane, etc.
     Len+=NMEA_AppendCheckCRNL(NMEA, Len);
     NMEA[Len]=0;
     return Len; }                                                 // return number of formatted characters

} ;

// =======================================================================================================

template <const uint8_t MaxTgts=32>
 class LookOut
{ public:
   union
   { uint32_t        ID;      // own ID
     struct
     { uint32_t  Address:24;  //
       uint8_t  AddrType: 8;  // ADS-L address-type
     } ;
   } ;
   Acft_RelPos     Pos;                   // Own position relative to the reference Lat/Lon/Alt

   uint32_t    RefTime;                   // [sec] ref. T for the local T,X,Y,Z coord. system
                                          //       ref. X,Y,Z for the local T,X,Y,Z coord. system
   int32_t     RefLat;                    // [1/60000deg]
   int32_t     RefLon;                    // [1/60000deg]
   int32_t     RefAlt;                    // [m]
   int16_t     LatCos;                    // [2^-12]
   int16_t     GeoidSepar;                // [m]

   int8_t      Pred;                      // [0.5sec] amount of time by which position has been predicted/extrapolated

   union
   { uint8_t Flags;
     struct
     { uint8_t GpsPrec     :6;            // GPS position precision
       bool    isMoving    :1;            // own position moving
       bool    hasPosition :1;            // own position is valid
     } ;
   } ;

   uint8_t     WarnLevel;                 // highest warning level of all the targets
   uint8_t     WeakestIdx;                // index for the weakest target (or a not allocated target)
   uint32_t    WeakestRank;               // rank of the weakest target

   uint8_t AcftType;

   uint8_t Targets;                       // [aircrafts] actual number of targets monitored
   uint8_t WorstTgtIdx;                   // [] most dangereous target
   uint8_t WorstTgtTime;                  // [0.5s] time to closest approach

   const static uint8_t MaxTargets  = MaxTgts; // maximum number of targets
   LookOut_Target     Target[MaxTargets]; // array of Targets

   const static int32_t   DistRange = 10000; // [m] drop immediately anything beyond this distance
   const static int16_t MinHorizSepar = 100; // [m] minimum horizontal separation
   const static int16_t MinVertSepar  =  50; // [m] minimum vertical separation
   const static int16_t WarnTime      =  20; // [sec] target warning prior to impact

   Acft_RelPos PredMe, PredTgt;           // for temporary storage of predictions.

   char Line[120];                        // for printing

  public:

   void Clear(void)
   { Flags=0; ID=0; Pos.Clear(); Pred=0;
     Targets=0; WeakestIdx=0; WeakestRank=0xFFFFFFFF;
     WorstTgtIdx=0; WorstTgtTime=0xFF;
     for(uint8_t Idx=0; Idx<MaxTargets; Idx++)
     { Target[Idx].Clear(); }
   }

   int16_t getRelBearing(const LookOut_Target *Tgt) const  // [360/0x10000 deg] relative bearing to the target
   { return Tgt->getBearing()-Pos.Heading; }

   uint8_t WritePOGNA(char *NMEA, const LookOut_Target *Tgt) // Alert NMEA centence
   { uint8_t Len=0;
     Len+=Format_String(NMEA+Len, "$POGNA,");

     Len+=NMEA_AppendCheckCRNL(NMEA, Len);
     NMEA[Len]=0;
     return Len; }

   void PrintPFLA(void)                                    // print (for debug) $PFLAU and PFLAA
   { WritePFLAU(Line); printf("%s", Line);
     for(uint8_t Idx=0; Idx<MaxTargets; Idx++)
     { if(!Target[Idx].Alloc) continue;
       if( Target[Idx].DistMargin) continue;
       Target[Idx].WritePFLAA(Line);
       printf("%s", Line);
     }
   }

   void WritePFLA(void (*Output)(char))                    // produce $PFLAU and PFLAA on the console output
   { WritePFLAU(Line); Format_String(Output, Line);
     for(uint8_t Idx=0; Idx<MaxTargets; Idx++)
     { if(!Target[Idx].Alloc) continue;                    // skip empty slots
       if( Target[Idx].DistMargin) continue;               // skip slots with distance margin remaining
       Target[Idx].WritePFLAA(Line);
       Format_String(Output, Line);
     }
   }

   uint8_t WritePFLAU(char *NMEA)                          // produce the FLAM anti-collision status
   { const LookOut_Target *Tgt = 0;
     if(WarnLevel>0) Tgt = Target + WorstTgtIdx;
     uint8_t Len=0;
     Len+=Format_String(NMEA+Len, "$PFLAU,");
     Len+=Format_UnsDec(NMEA+Len, (uint32_t)Targets, 1);   // number of targets received
     NMEA[Len++]=',';
     NMEA[Len++]='0'+hasPosition;                          // TX status
     NMEA[Len++]=',';
     NMEA[Len++]='0'+hasPosition;                          // GPS status: 0=no fix, 1=on the ground, 2=airborne
     NMEA[Len++]=',';
     NMEA[Len++]='1';                                      // power status: one could monitor the supply
     NMEA[Len++]=',';
     NMEA[Len++]='0'+WarnLevel;                            // Warning level: 0..3
     NMEA[Len++]=',';
     if(Tgt)                                               // [deg] relative bearing: -180..+180
     { Len+=Format_SignDec(NMEA+Len, ((int32_t)getRelBearing(Tgt)*45+0x1000)>>13, 1); }
     NMEA[Len++]=',';
     NMEA[Len++]='0'+((WarnLevel>0)<<1);                   // alarm-type: 0=none, 2=aircraft, 3=obstacle/zone/terrain
     NMEA[Len++]=',';
     if(Tgt)                                               // [m] relative vertical distance
     { Len+=Format_SignDec(NMEA+Len, (int32_t)Tgt->dZ>>1, 1, 0, 1); }
     NMEA[Len++]=',';
     if(Tgt)                                               // [m] relative horizontal distance
     { Len+=Format_UnsDec(NMEA+Len, (uint32_t)Tgt->HorDist>>1, 1); }
     if(Tgt)                                               // ID
     { NMEA[Len++]=',';                                    //
       uint32_t Addr=Tgt->ID;
       Len+=Format_Hex(NMEA+Len, (uint8_t)(Addr>>16));     // 24-bit address: RND, ICAO, FLARM, OGN
       Len+=Format_Hex(NMEA+Len, (uint16_t)Addr); }
// #ifdef WITH_SKYDEMON
//     { Len+=Format_Hex(NMEA+Len, Tgt->ID & 0x00FFFFFF); }  // maybe just 6 digits should be produced ?
// #else
//     { Len+=Format_Hex(NMEA+Len, Tgt->ID); }
// #endif
     Len+=NMEA_AppendCheckCRNL(NMEA, Len);
     NMEA[Len]=0;
     return Len; }

   int32_t Latitude(int32_t LatDist) const { return RefLat + (LatDist*27)/10; }   // relative distance [0.5m] => Latitude [1/60000 deg]
   int32_t Longitude(int32_t LonDist) const { LonDist = (LonDist<<12)/LatCos; return RefLon + (LonDist*27)/10; }

   // static int32_t CordicCoord(int32_t Coord) { return ((int64_t)Coord*83399993+(1<<21))>>22; } // [1/60000deg] => [cordic]

   void Write(GDL90_REPORT &Report) const                   // Own GDL90 position report
   { Report.Clear();
     Report.setAddress(ID&0xFFFFFF);
     Report.setAddrType(((ID>>24)&0x03)!=1);                // ICAO or non-ICAO
     Report.setAcftTypeOGN(ID>>26);
     Report.setAcftCall(ID);
     int32_t Alt = RefAlt+(Pos.Z>>1)+(Pos.dStdAlt>>1);      // [m]
     Report.setAltitude(MetersToFeet(Alt));
     Report.setHeading(Pos.Heading>>8);
     Report.setSpeed(Pos.Speed);                            // [0.5m/s] => [knots]
     Report.setClimbRate((int32_t)99*Pos.Climb);            // [0.5m/s] => [fpm]
     int32_t Lat = Latitude (Pos.X); Report.setLatOGN(Lat);
     int32_t Lon = Longitude(Pos.Y); Report.setLonOGN(Lon);
     Report.setMiscInd(0x2);
     Report.setAccuracy(9, 9); }

   void Write(GDL90_REPORT &Report, const LookOut_Target *Tgt) const   // Target GDL90 position report
   { Report.Clear();
     Report.setAlertStatus(Tgt->WarnLevel>0);
     Report.setAddress(Tgt->ID&0xFFFFFF);
     Report.setAddrType(((Tgt->ID>>24)&0x03)!=1);
     Report.setAcftTypeOGN(Tgt->ID>>26);
     Report.setAcftCall(Tgt->ID);
     int32_t Alt = RefAlt+(Tgt->Pos.Z>>1)+(Tgt->Pos.dStdAlt>>1); // [m]
     Report.setAltitude(MetersToFeet(Alt));
     Report.setHeading(Tgt->Pos.Heading>>8);
     Report.setSpeed(Tgt->Pos.Speed);                       // [0.5m/s] => [knots]
     Report.setClimbRate((int32_t)99*Tgt->Pos.Climb);       // [0.5m/s] => [fpm]
     int32_t Lat = Latitude (Tgt->Pos.X); Report.setLatOGN(Lat);
     int32_t Lon = Longitude(Tgt->Pos.Y); Report.setLonOGN(Lon);
     Report.setMiscInd(0x2);
     Report.setAccuracy(9, 9); }

   uint8_t WritePGAV5(char *NMEA, const LookOut_Target *Tgt) const
   { uint8_t Len=0;
     Len+=Format_String(NMEA+Len, "$PGVA5,");                      // sentence name   }
     // age of position
     NMEA[Len++]=',';
     uint32_t Addr = Tgt->ID & 0xFFFFFF;                           // [24-bit] address
     Len+=Format_Hex(NMEA+Len, (uint8_t)(Addr>>16));               // 24-bit address: RND, ICAO, FLARM, OGN
     Len+=Format_Hex(NMEA+Len, (uint16_t)Addr);
     NMEA[Len++]=',';
     // [deg] Lattitude
     NMEA[Len++]=',';
     // [deg] Longitude
     NMEA[Len++]=',';
     // [feet] GPS altitude (HAE = MSL+GeoidSepar)
     NMEA[Len++]=',';
     // [feet] standard pressure altitude
     NMEA[Len++]=',';
     // FlightID (or call ?)
     NMEA[Len++]=',';
     // [deg] ground track
     NMEA[Len++]=',';
     // [kts] ground speed
     NMEA[Len++]=',';
     // [fpm] vertical speed
     NMEA[Len++]=',';
     // [] signal strength
     NMEA[Len++]=',';
     // [hex] aircraft category
     Len+=NMEA_AppendCheckCRNL(NMEA, Len);
     NMEA[Len]=0;
     return Len; }                                                 // return number of formatted characters

   void Print(void) const
   { if(!hasPosition) return;
     uint32_t Sec = RefTime%60;
     uint32_t Min = ((RefTime-Sec)/60)%60;
     uint32_t Hour = ((RefTime-Sec-Min*60)/3600)%24;
     printf("%08lX Ref: %02d:%02d:%02d: [%+10.6f, %+11.6f]deg %ldm\n", (long int)ID, Hour, Min, Sec, 0.0001/60*RefLat, 0.0001/60*RefLon, (long int)RefAlt);
     printf("%08lX/%+5.1fs/  Margin/ HorDist/Margin/  Miss/  Miss/w%d", (long int)ID, 0.5*Pred, WarnLevel); Pos.Print();
     for(uint8_t Idx=0; Idx<MaxTargets; Idx++)
     { const LookOut_Target *Tgt = Target+Idx;
       if(Tgt->Alloc) Tgt->Print();
     }
   }

   template <class OGNx_Packet>
    int32_t Start(OGNx_Packet &OwnPos, uint32_t RxTime)
   { Clear();
     // ID = OwnPos.getAddressAndType() | ((uint32_t)OwnPos.Position.AcftType<<26) ;
     ID = OwnPos.Header.Address |((uint32_t)OwnPos.Header.AddrType<<24);
     AcftType = OwnPos.Position.AcftType;
     RefTime = OwnPos.getTime(RxTime);                // set reference time
     RefLat = OwnPos.DecodeLatitude();                // and reference positon
     RefLon = OwnPos.DecodeLongitude();
     RefAlt = OwnPos.DecodeAltitude();                // and reference altitude
     LatCos = Icos(GPS_Position::calcLatAngle16(RefLat));
     Pred=0;
     return Pos.Read(OwnPos, RxTime, RefTime, RefLat, RefLon, RefAlt, LatCos, DistRange); }

   template <class OGNx_Packet>
    const LookOut_Target *ProcessOwn(OGNx_Packet &OwnPos, uint32_t RxTime)                // process own position
   { // printf("ProcessOwn() ... entry\n");
     if(hasPosition)                                                                      // in my position is valid
     { Pred=0;
       if(Pos.Read(OwnPos, RxTime, RefTime, RefLat, RefLon, RefAlt, LatCos, DistRange)<0)         // read the new position
       { hasPosition = Start(OwnPos, RxTime)>=0; }                                                // if this fails, attempt to start from the new position
       // if(!Pos.hasStdAlt)                                                              // if no StdAlt
       // { }                                                                             // get it from targets
     }
     else
     { hasPosition = Start(OwnPos, RxTime)>=0; }

     if(hasPosition)                                                                  // if already started
     { AdjustRefTime(Pos.T);                                                          // adjust time ref. point if needed
       AdjustRefAlt();                                                                // adjust vertical ref. altitude if needed
       AdjustRefLatLon(OwnPos); }                                                         // adjust horizontal Lat/Lon position if needed.

     WarnLevel=0;
     Targets=0;
     WorstTgtIdx=0;                                                                   // get ready to search the most dangerous aircraft
     WorstTgtTime=0xFF;
     for(uint8_t Idx=0; Idx<MaxTargets; Idx++)                                        // go over targets
     { LookOut_Target *Tgt = Target+Idx;
       if(!Tgt->Alloc) continue;                                                      // skip empty slots
       if(Tgt->DistMargin==0)                                                         // those with no safety margin
       { while(Tgt->Pos.T<=(Pos.T-4))                                                 // bring closer in time to my (new) position
         { Tgt->Pos.StepFwd2secs(); Tgt->Pred+=4; }
       }
       uint8_t Warn=calcTarget(Tgt);                                                  // (re)calculate the target
       if(Warn)
       { if(Warn>WarnLevel) WarnLevel=Warn;                                           // register highest warning level
         if(Tgt->TimeMargin<WorstTgtTime) { WorstTgtTime=Tgt->TimeMargin; WorstTgtIdx=Idx; } // and shortest time margin
       }
       Targets++; }
     // printf("ProcessOwn() ... exit\n");
     // if(Targets==0) return 0;                                                       // return NULL if no targets are tracked
     LookOut_Target *Tgt = Target+WorstTgtIdx;
     if( (!Tgt->Alloc) || (Tgt->DistMargin>0) ) return 0;                              // return NULL if target is not a thread
     return Tgt; }                                                                     // return the pointer to the most dangerous target

   const LookOut_Target *ProcessTarget(ADSL_Packet &Packet, uint32_t RxTime)           // process a position of another aircraft in ADS-L format
   { // printf("ProcessTarget(%d) ... entry\n", WeakestIdx);
     LookOut_Target *New = Target+WeakestIdx;                                          // get a free or lowest rank slot
     New->Clear();                                                                     // put the new position there
     if(New->Pos.Read(Packet, RxTime, RefTime, RefLat, RefLon, RefAlt, LatCos, GeoidSepar, DistRange)<0) return 0; // calculate the position against the reference position
     if(!New->Pos.hasStdAlt)                                                           // if no baro altitude
     { if(Pos.hasStdAlt) { New->Pos.dStdAlt=Pos.dStdAlt; New->Pos.hasStdAlt=1; } }     // take it from own
     New->Address  = Packet.getAddress();
     New->AddrType = Packet.getAddrTypeOGN();
     New->AcftType = Packet.getAcftTypeOGN();
     return ProcessTarget(New); }

   template <class OGNx_Packet>
    const LookOut_Target *ProcessTarget(OGNx_Packet &Packet, uint32_t RxTime, const char *Call=0)  // process a position of another aircraft in OGN format
   { // printf("ProcessTarget(%d) ... entry\n", WeakestIdx);
     LookOut_Target *New = Target+WeakestIdx;                                          // get a free or lowest rank slot
     New->Clear();                                                                     // put the new position there
     if(New->Pos.Read(Packet, RxTime, RefTime, RefLat, RefLon, RefAlt, LatCos, DistRange)<0) return 0; // calculate the position against the reference position
     if(!New->Pos.hasStdAlt)                                                           // if no baro altitude
     { if(Pos.hasStdAlt) { New->Pos.dStdAlt=Pos.dStdAlt; New->Pos.hasStdAlt=1;} }      // take it from own
     New->Address  = Packet.Header.Address;
     New->AddrType = Packet.Header.AddrType;
     New->AcftType = Packet.Position.AcftType;
     if(Call) { strncpy(New->Call, Call, 10); New->Call[10]=0; }
         else   New->Call[0]=0;
     return ProcessTarget(New); }

   const LookOut_Target *ProcessTarget(LookOut_Target *New)
   {  // printf("ProcessTarget() ... %08X\n", ID);
     uint8_t OldIdx;
     LookOut_Target *Old = 0;                                                                   // possible previous index to the same ID
     for(OldIdx=0; OldIdx<MaxTargets; OldIdx++)                                        // scan targets already on the list
     { if(Target[OldIdx].Alloc==0) continue;                                           // skip not allocated
       if(OldIdx==WeakestIdx) continue;                                                // skip the new position
       if(Target[OldIdx].ID==New->ID) break; }                                         // to find previous position for the target
     if(OldIdx<MaxTargets)                                                             // if found
     { Old = Target+OldIdx;
       if((Old->Pos.T-Old->Pred)>Target[WeakestIdx].Pos.T)                             // if position is not really newer
         return Old;                                                                   // then stop processing this (not new) position
       Old->Alloc=0; }                                                                 // mark old position as "not allocated"
     New->Alloc=1;                                                                     // mark this position as allocated

     if(Old && Old->Call[0] && New->Call[0]==0) { strncpy(New->Call, Old->Call, 10); New->Call[10]=0; } // copy the call

     if(Old && (!New->Pos.hasTurn || !New->Pos.hasClimb))
     { int16_t Told = Old->Pos.T - Old->Pred;
       int16_t dT = New->Pos.T - Told;
       int32_t dZ = 0;
       if(Old->Pos.hasClimb)
       { dZ = (Old->Pos.Climb*Old->Pred)>>1; }
       if(!New->Pos.hasClimb && dT>1)
       { New->Pos.Climb = (New->Pos.Z-(Old->Pos.Z-dZ))/dT*2;
         New->Pos.hasClimb=1; }
       int16_t dH=0;
       if(Old->Pos.hasTurn)
       { dH = (Old->Pos.Turn*Old->Pred)>>1; }
       if(!New->Pos.hasTurn && dT>1)
       { dH = New->Pos.Heading-(Old->Pos.Heading-dH);
         New->Pos.Turn = dH/dT*2 ;
         New->Pos.hasTurn=1; }
       // printf("Climb/Turn %08X dT=%3.1fs ", New->ID, 0.5*dT); New->Pos.Print();
     }

     AdjustRefTime(New->Pos.T);                                                        // possibly adjust the time reference after this new position time
     // printf("ProcessTarget() ... AdjustRefTime()\n");

     if(Pos.T<=(New->Pos.T-4))                                                         // if new position more than 2sec away from own
     { Pos.StepFwd2secs(); Pred+=4; }                                                  // bring own position closer in time

     uint8_t Warn=calcTarget(New);                                                     // calculate the safety margin for the target
     if(Warn>WarnLevel) WarnLevel=Warn;                                                // record higest warnign level
     // printf("ProcessTarget() ... calc()\n");

     uint8_t MaxIdx=WeakestIdx; uint32_t Max=New->Rank;                                // look for the lowest rank position on the list
     for( uint8_t Idx=MaxIdx; ; )                                                      // go over targets
     { Idx++; if(Idx>=MaxTargets) Idx=0;
       if(Idx==WeakestIdx) break;                                                      // end the loop when back at New
       LookOut_Target &Tgt = Target[Idx];
       if(!Tgt.Alloc) { MaxIdx=Idx; break; }                                           // if unallocated target found: stop the search
       // if(Tgt.Rank==0xFFFFFFFF) { MaxIdx=Idx; break; }                                 // if abs. weakest target found: stop the search
       if(Tgt.Rank>=Max) { Max=Tgt.Rank; MaxIdx=Idx; }                                 // if weaker rank found: note it
     }
     WeakestIdx=MaxIdx;                                                                // tqke the weakest slot for the nest time

     return New; }

   uint8_t calcTarget(LookOut_Target *Tgt)                                              // calculate the savety margin for the (new) target
   {
     Tgt->TimeMargin=0xFF;                                                              // initially set inf. time margin
     Tgt->WarnLevel=0;                                                                  // warning level=0
     Tgt->MissTime=0;
     Tgt->MissDist=0;
     uint16_t Margin  = calcVertMargin(Tgt);                                            // [0.5m] calc. vertical margin
     if(Margin==0) Margin = calcHorizMargin(Tgt);                                       // [0.5m] if vertical margin iz zero then get horizontal margin
     Tgt->DistMargin = Margin;                                                          // [0.5m]
     if(Margin>0)                                                                       // if there is still safety margin, no more calc. (dealloc. ?)
     { // Tgt->TimeMargin = Margin/;
       return 0; }                                                                      // return warning level = 0
                                                                                        // at this point the distance is possibly small enough so we may hit the target
#ifdef DEBUG_PRINT
     printf("calcTarget(0x%08X) ... no abs. safety margin\n", Tgt->ID);
#endif
     Tgt->calcVel(Pos);                                                                 // calculate relative velocity
     // uint32_t RelVelSqr  = Tgt->VelSqr();                                               // [0.25(m/s)^2] velocity square
     int16_t dT = Pos.T - Tgt->Pos.T;                                                   // [0.5s] we need to recalc. the distance if the target time is not same as mine
     if(dT)                                                                             // [0.5s] if time difference is non-zero
     { int16_t Vx,Vy,Vz;                                                                // [0.5m/s] Target speed vector
       Tgt->Pos.getSpeedVector(Vx, Vy); Vz=Tgt->Pos.Climb;                              // [0.5m/s]
       Tgt->dX += (dT*Vx)>>1;                                                           // update Target relative distance
       Tgt->dY += (dT*Vy)>>1;
       Tgt->dZ += (dT*Vz)>>1;
       Tgt->HorDist = Acft_RelPos::FastDistance(Tgt->dX, Tgt->dY); }                    // update Target horizontal distance
     // uint32_t RelDistSqr = Tgt->DistSqr();                                              // [0.25m^2]     distance square
     // uint32_t WarnTimeSqr = (uint32_t)WarnTime*WarnTime;                                // [s] warning time square
     // printf("calcTarget(0x%08X) ...\n", Tgt->ID);
     // printf("RelDistSqr = %3.1fm^ RelVelSqr=%3.1f(m/s)^2 Time=%ds\n", 0.25*RelDistSqr, 0.25*RelVelSqr, RelVelSqr ? IntSqrt(RelDistSqr/RelVelSqr):0xFFFF);
     // if((RelVelSqr*WarnTimeSqr)<=RelDistSqr) return 0;
     uint16_t RelVel = Acft_RelPos::FastDistance(Tgt->Vx, Tgt->Vy, Tgt->Vz);            // [0.5m/s]
     uint16_t MinMissDist = 4*RelVel+Pos.Error + Tgt->Pos.Error + 2*MinHorizSepar;      // [0.5m]
#ifdef DEBUG_PRINT
     printf("Target: [%+4.1f, %+4.1f, %+4.1f] = %3.1fm/s MinMissDist=%3.1fm\n",
             0.5*Tgt->Vx, 0.5*Tgt->Vy, 0.5*Tgt->Vz, 0.5*RelVel, 0.5*MinMissDist);
#endif

     PredMe=Pos; PredTgt=Tgt->Pos;                                                      // copy my position and the target to temporary variables
     int16_t TimeMargin = PredMe.StepTillMinSepar(PredTgt, MinMissDist, 2*(WarnTime+2)); // predict when minimum separation is reached
#ifdef DEBUG_PRINT
     printf("StepTillMinSepar(0x%08X, %3.1fm, %1ds) => %+4.1fs\n", Tgt->ID, 0.5*MinMissDist, WarnTime+2, 0.5*TimeMargin);
#endif
     Tgt->TimeMargin=TimeMargin;                                                        // store the time margin till minimum separation
     Tgt->MissTime=TimeMargin;
     if(TimeMargin>(2*WarnTime)) return 0;                                              // if time-to-margin longer than warning time then return no warning
     Tgt->WarnLevel=1;                                                                  // otherwise set already the first warning level
     if(TimeMargin>WarnTime) return Tgt->WarnLevel;                                     // is time-to-margin longer than half the warning time, then stop calculations here, return 1st warning level

#ifdef DEBUG_PRINT
     printf("Me :"); PredMe.Print();
     printf("Tgt:"); PredTgt.Print();
#endif
     for(uint8_t Count=3; Count; Count--)
     { int16_t MissTime = PredMe.MissTime(PredTgt, WarnTime);
#ifdef DEBUG_PRINT
       printf("%d: MissTime = %+4.1fs\n", Count, 0.5*MissTime);
#endif
       if(abs(MissTime)<=1) break;
       PredMe.StepFwd(MissTime);
       PredTgt.StepFwd(PredMe.T-PredTgt.T); }
#ifdef DEBUG_PRINT
     printf("Me :"); PredMe.Print();
     printf("Tgt:"); PredTgt.Print();
#endif
     Tgt->MissDist = PredMe.FastDistance(PredTgt);
     Tgt->MissTime = PredMe.T-Pos.T;
#ifdef DEBUG_PRINT
     printf("MissTime = %+4.1f, MissDist = %4.1f\n", 0.5*Tgt->MissTime, 0.5*Tgt->MissDist);
#endif
     if( (Tgt->MissTime<0) || (Tgt->MissTime>(2*WarnTime)) || (Tgt->MissDist>MinMissDist) ) Tgt->WarnLevel=0;
     else if(Tgt->MissDist<(2*MinHorizSepar)) { Tgt->WarnLevel=2; if(Tgt->MissTime<(2*WarnTime/3)) Tgt->WarnLevel=3; }
#ifdef DEBUG_PRINT
     printf("calcTarget(%08X) V=[%+5.1f, %+5.1f, %+5.1f]m/s D=[%+7.1f, %+7.1f, %+7.1f]m MissTime=%5.1fsec MissDist=%6.1fm\n",
              Tgt->ID, 0.5*Tgt->Vx, 0.5*Tgt->Vy, 0.5*Tgt->Vz, 0.5*Tgt->dX, 0.5*Tgt->dY, 0.5*Tgt->dZ, 0.5*Tgt->MissTime, 0.5*Tgt->MissDist);
#endif
     return Tgt->WarnLevel; }

   uint16_t calcVertMargin(LookOut_Target *Tgt)                        // calculate vertical savety margin
   { Tgt->dZ = Tgt->Pos.Z     - Pos.Z;                                 // [0.5m] relative vertical distance
     Tgt->Vz = Tgt->Pos.Climb - Pos.Climb;                             // [0.5ms/s] relative vertical speed
     int16_t VertError = Pos.Error+Tgt->Pos.Error; VertError+=VertError/2; // [0.5m] est. total vertical error
     VertError += 2*MinVertSepar;                                      // [0.5m]
     if(abs(Tgt->dZ)<=VertError) return 0;                             // if vertical distance less than margin required: return zero margin
     if(Tgt->dZ>0) { if(Tgt->Vz>=0) return  Tgt->dZ-VertError; }       // if target is higher and is climbing return relative vertical distance
              else { if(Tgt->Vz<=0) return -Tgt->dZ-VertError; }       // if target is lower and is falling, return like above
     int16_t dT    = Tgt->Pos.T - Pos.T;                               // [0.5sec] time diff. in measured positions
     int32_t MaxAlt = ( (int32_t)Tgt->Vz * (2*(WarnTime+4)+abs(dT)) )>>1;  // [0.5m] max. altitude change within the warning time
     MaxAlt = abs(MaxAlt) + VertError;                                 // [0.5m]
     // printf("calcVertMargin() dAlt=%+4.1f dT=%+4.1f Climb=%+4.1f MaxAlt=%+4.1f\n", 0.5*Tgt->dZ, 0.5*dT, 0.5*Tgt->Vz, 0.5*MaxAlt);
     if(Tgt->dZ>0)                                                     // if target is above
     { if(Tgt->dZ<MaxAlt) return 0;
       else return ( 2*Tgt->dZ-MaxAlt); }                              // [0.5m]
     else                                                              // if target is below
     { if((-Tgt->dZ)<MaxAlt) return 0;
       else return (-2*Tgt->dZ-MaxAlt); }                              // [0.5m]
   }                                                                   // return the vertical margin: if positive: we are safe, if zero: we are too close

   uint16_t calcHorizMargin(LookOut_Target *Tgt)
   { Tgt->dX = Tgt->Pos.X - Pos.X;                                     // [0.5m] relative distance
     Tgt->dY = Tgt->Pos.Y - Pos.Y;                                     // [0.5m]
     Tgt->HorDist = Acft_RelPos::FastDistance(Tgt->dX, Tgt->dY);       // [0.5m] estimate horizontal distance
     int16_t HorError = Pos.Error+Tgt->Pos.Error;                      // [0.5m] sum GPS error from me and the target
     HorError += 2*MinHorizSepar;                                      // [0.5m] add the min. separation required
     int16_t dT    = abs(Tgt->Pos.T - Pos.T);                          // [0.5m] time difference between my data and target data
     int32_t MaxDistT = ((int32_t)Tgt->Pos.Speed * (2*(WarnTime+4)+dT))>>1; // [0.5m] possible distance covered by the target
     int32_t MaxDistM = ((int32_t)     Pos.Speed * (2*(WarnTime+4)+dT))>>1; // [0.5m] possible distance covered by me
     int32_t MaxDist  = MaxDistT + MaxDistM + HorError;                // [0.5m] add horizontal separation
     if(MaxDist<Tgt->HorDist) return Tgt->HorDist-MaxDist;             // [0.5m] return the (positive) difference: we are safe
     return 0; }                                                       // zero-margin => bad !

   void AdjustRefTime(int16_t TimeDelta)                               // [0.5s] adjust the time reference point
   { if(TimeDelta<(2*12)) return;                                      // if less than 12sec into the future than skip it
     TimeDelta/=2;                                                     // [sec]
     RefTime+=TimeDelta;                                               // shift time reference
     Pos.T-=2*TimeDelta;                                               // shift the relative time on my own position
     if(Pos.T<(-2*30) || Pos.T>(+2*30)) hasPosition=0;                 // if older than 30sec declare "no position"
     for(uint8_t Idx=0; Idx<MaxTargets; Idx++)                         // go over the targets
     { LookOut_Target &Tgt = Target[Idx]; if(!Tgt.Alloc) continue;     // skip unallocated
       Tgt.Pos.T-=2*TimeDelta;                                         // shift the relative time
       if((Tgt.Pos.T-Tgt.Pred)<(-2*30)) Tgt.Alloc=0;                   // if older than 30sec then drop the target
     }
   }

   void AdjustRefAlt(void)                     // shift the vertical reference point when we get too far off
   { if(abs(Pos.Z)<(2*200)) return;           // don't shift if less than 200m from the reference point
     int16_t AltDelta=Pos.Z/2;
     RefAlt+=AltDelta;
     Pos.Z-=2*AltDelta;
     for(uint8_t Idx=0; Idx<MaxTargets; Idx++)
     { if(Target[Idx].Alloc) Target[Idx].Pos.Z-=2*AltDelta; }
   }

   template <class OGNx_Packet>
    void AdjustRefLatLon(OGNx_Packet &Me)                          // shift the horizontal reference point when we get too far off
   { if( (abs(Pos.X)<(2*1000)) && (abs(Pos.Y)<(2*1000)) ) return; // when we are off by 1km horizontal
     int32_t LatDist, LonDist;
     if(Me.calcDistanceVector(LatDist, LonDist, RefLat, RefLon, LatCos, DistRange)<0) { return; } // distance from the current reference
     RefLat = Me.DecodeLatitude();                                 // take the ref. Lat/Lon from the packet
     RefLon = Me.DecodeLongitude();
     LatDist*=2;                                                   // [m/s] => [0.5m/s]
     LonDist*=2;
     Pos.X -= LatDist;
     Pos.Y -= LonDist;
     for(uint8_t Idx=0; Idx<MaxTargets; Idx++)                     // shift the relative position of every target
     { if(Target[Idx].Alloc) { Target[Idx].Pos.X-=LatDist; Target[Idx].Pos.Y-=LonDist; } }
     LatCos = Icos(GPS_Position::calcLatAngle16(RefLat));
   }

} ;

// =======================================================================================================

#endif // __LOKOUT_H__
