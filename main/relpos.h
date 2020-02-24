#include <stdio.h>

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "intmath.h"

#include "ogn.h"

// =======================================================================================================

class Acft_RelPos             // 3-D relative position with speed and turn rate
{ public:
   int16_t T;                    // [0.5sec]
   int16_t X,Y,Z;                // [0.5m]
  uint16_t Speed;                // [0.5m/s]
  uint16_t Heading;              // [360/0x10000 deg]
   int16_t Climb;                // [0.5m/s]
   int16_t Turn;                 // [360/0x10000 deg/s] [2*PI/0x10000 rad/sec]
   int16_t Dx,Dy;                // [2^-12] directon vector - calc. on Heading
  uint8_t  Error;                // [0.5m]
  uint8_t  Spare;
   // int16_t Ax,Ay;                // [1/16m/s^2] acceleration vactor
   // int16_t R;                    // [0.5m] (signed) turning radius - calc. from Turn and Speed
   // int16_t Ox,Oy;                // [0.5m] turning circle center   - only valid when R!=0

  public:
   void Print(void) const
   { printf("%+7.1f: [%+7.1f,%+7.1f,%+7.1f]m %5.1fm/s %05.1fdeg %+5.1fm/s %+5.1fdeg/sec [%3.1fm]",
            0.5*T, 0.5*X, 0.5*Y, 0.5*Z, 0.5*Speed, (360.0/0x10000)*Heading, 0.5*Climb, (360.0/0x10000)*Turn, 0.5*Error);
     // printf(" [%+6.2f,%+6.2f]m/s^2", 0.0625*Ax, 0.0625*Ay);
     // if(R) printf(" R:%+8.1fm [%+7.1f, %+7.1f]", 0.5*R, 0.5*Ox, 0.5*Oy);
     printf("\n"); }

   void Clear(void)
   { T =0;
     X =0; Y =0; Z =0;
     Speed=0; Heading=0; Climb=0; Turn=0;
     Error=4; }

   uint32_t SqrDistance(Acft_RelPos &Target)
   { int32_t dX = Target.X-X;
     int32_t dY = Target.Y-Y;
     int32_t dZ = Target.Z-Z;
     return dX*dX+dY*dY+dZ*dZ; }                       // [0.25m^2]

   static uint32_t SqrDistance(int16_t dX, int16_t dY, int16_t dZ)
   { return (int32_t)dX*dX + (int32_t)dY*dY + (int32_t)dZ*dZ; }

   static uint32_t SqrDistance(int16_t dX, int16_t dY)
   { return (int32_t)dX*dX + (int32_t)dY*dY; }

   uint32_t FastDistance(Acft_RelPos &Target)
   { int16_t dX = Target.X-X;
     int16_t dY = Target.Y-Y;
     int16_t dZ = Target.Z-Z;
     return FastDistance(dX, dY, dZ); }                // [0.5m]

   static uint16_t FastDistance(int16_t dX, int16_t dY)
   { dX = abs(dX); dY = abs(dY);
     if(dX>dY) return dX+dY/2;
          else return dY+dX/2; }

   static uint16_t FastDistance(int16_t dX, int16_t dY, int16_t dZ)
   { return FastDistance((int16_t)FastDistance(dX, dY), dZ); }

   // predict self and target until MinSepar is reached but no longer than MaxTime
   int16_t StepTillMinSepar(Acft_RelPos &Target, uint16_t MinSepar, int16_t MaxTime=40) // [0.5m] [0.5s]
   {  int16_t PredTime=0;                                 // count time by which we predict
     uint16_t MaxTurn=abs(Turn);                          // the max. turn rate
     uint16_t Turn2=abs(Target.Turn); if(Turn2>MaxTurn) MaxTurn=Turn2;
      int16_t MaxStepTime = 32;                           // [0.5s] max. allowed stpping time period
     if(MaxTurn>=0x100) MaxStepTime=0x2000/MaxTurn;       // [0.5s] maximup step time (for sharp turns)
     if(MaxStepTime<2) MaxStepTime=2;                     // [0.5s] but don't do smaller steps than 1sec
     uint16_t TotSpeed = FastDistance(Speed, Climb) + FastDistance(Target.Speed, Target.Climb); // [0.5m/s] "total" speed, thus the sum of the two speeds magnitudes
#ifdef DEBUG_PRINT
     printf("StepTillMinSepar( , MinSepar=%3.1fm, MaxTime=%3.1fs)\n", 0.5*MinSepar, 0.5*MaxTime);
     Print(); Target.Print();
#endif
     for( ; ; )
     { if(MaxTime==0) return PredTime;                   // if max. prediction time reached: stop
       int32_t DistMargin  = FastDistance(Target);       // [0.5m]   Distance margin to the target
       if(DistMargin<=MinSepar) return PredTime;         //          If distance margin already below minimum
       int16_t  dT = Target.T-T;                         // [0.5sec] Target may not be exact same time
       int32_t  dS = (dT*TotSpeed)>>1;                   // [0.5m]   thus some extra distance margin
       DistMargin -= abs(dS);                            // [0.5m]   subtract margin due to time difference
       if(DistMargin<=MinSepar) return PredTime;         // [0.5m]   if distance margin below minimum
       DistMargin -= MinSepar;                           // [0.5m]   subtract minimum separation from the distance margin
       if((TotSpeed*MaxTime) < (2*DistMargin) )          //          If plenty enough margin given the speed
       { uint16_t TimeMargin=MaxTime+2;
         if((2*DistMargin)<(TotSpeed*TimeMargin)) TimeMargin=2*DistMargin/TotSpeed+1;
         TimeMargin+=PredTime; // if(TimeMargin>240) TimeMargin=240;
         return TimeMargin; }
       int16_t StepTime = (2*DistMargin)/TotSpeed+1;      // [0.5s]   Time margin given distance margin and speed
#ifdef DEBUG_PRINT
       printf("DistMargin=%3.1fm => StepTime=%+4.1fs\n", 0.5*DistMargin, 0.5*StepTime);
#endif
       // if(StepTime==0) return PredTime;
       // if(StepTime<1) StepTime=1;                          // minimum step time for prediction
       if(StepTime>MaxStepTime) StepTime=MaxStepTime;      // [0.5s] maximum step time for prediction
       if(StepTime>MaxTime) StepTime=MaxTime;
       int16_t NextTime = T+StepTime;                      // [0.5s] time
       StepFwd(StepTime);
       int16_t TgtStepTime = NextTime-Target.T;
       if(TgtStepTime>0) Target.StepFwd(TgtStepTime);
#ifdef DEBUG_PRINT
       Print(); Target.Print();
#endif
       PredTime+=StepTime;                                 // [0.5s] count time by which we predict
       MaxTime-=StepTime; }                                // [0.5s] decrement the max. time we should predict
     return MaxTime+1; }                                   // return estimated time margin before the separation could fall below minimum

   // predict the minimum distance: does not take turn into account thus must not be used on significant distances
   int16_t MissTime(Acft_RelPos &Target, int16_t MaxTime=40) // [0.5s]
   { int32_t dX  = Target.X;                                // [0.5m] Target distance vector
     int32_t dY  = Target.Y;
     int32_t dZ  = Target.Z;
     int32_t tVx, tVy; Target.getSpeedVector(tVx, tVy);     // [0.5m/s] Target speed vector
     int32_t tVz = Target.Climb;
     int16_t dT = Target.T - T;                         // [0.5sec] time difference betwen target and me
     if(dT)
     { dX -= (dT*tVx)>>1;                               // adjust target position for the time diff.
       dY -= (dT*tVy)>>1;
       dZ -= (dT*tVz)>>1; }
     dX -= X;                                           // [0.5m] Target relative position
     dY -= Y;
     dZ -= Z;
     int32_t dVx, dVy; getSpeedVector(dVx, dVy);        // [0.5m/s] Target relative speed
     dVx = tVx-dVx;
     dVy = tVy-dVy;
     int32_t dVz = tVz - Climb;

     int32_t DistSqrRate = dVx*dX + dVy*dY + dVz*dZ;      // [0.25m2/s] 3-D scalar product: rel. distance x rel. speed
     // if(DistSqrRate==0) return MaxTime;                   // if target neither moving away nor closing
     // if(DistSqrRate>0) return MaxTime;                    // if target moving away from me

     int32_t RelVelSqr  = dVx*dVx + dVy*dVy + dVz*dVz;             // [0.25m2/s2] 3-D relative velocity square
     // printf("MissTime() DistSqrRate = %+4.1fm^2/s RelVelSqr = %3.1f(m/s)^2\n", 0.25*DistSqrRate, 0.5*RelVelSqr);
     if( ( RelVelSqr*MaxTime) <= (-2*DistSqrRate) ) return  MaxTime;  // if min. approach time is longer than maximum
     if( (-RelVelSqr*MaxTime) >= (-2*DistSqrRate) ) return -MaxTime;  // if min. approach time is longer than maximum
     return (-2*DistSqrRate+(RelVelSqr/2))/RelVelSqr; }               // [0.5sec] time of the closest approach

   void calcDir(void)                                  // calculate the direction unity vector
   { Dx = Icos(Heading);                               // DirX = cos(Heading)
     Dy = Isin(Heading); }                             // DirY = sin(Heading)

   // void calcAccel(void)
   // { int32_t A = ((int32_t)Turn*Speed*201+0x8000)>>16; // [1/64m/s^2]
   //   Ax = (-A*Dy+0x2000)>>14;                          // [1/16m/s^2]
   //   Ay = (+A*Dx+0x2000)>>14; }

   // void calcTurn(int16_t MaxRadius=10000) // calculate the turning radius and turning circle center
   // { R=getTurnRadius(MaxRadius); if(R==0) return;
   //   Ox = X - ((Dy*R)>>12);
   //   Oy = Y + ((Dx*R)>>12); }

   int16_t getTurnDev(int16_t dT)                       // approx. horizontal deviation from straight line due to the turn
   { int32_t A = ((int32_t)Turn*Speed*201+0x8000)>>16;  // [1/64m/s^2] acceleration
     // printf("getTurnDev() A=%6.3fm/s^2\n", A/64.0);
     return (A*dT*dT+0x80)>>8; }                        // [0.5m]

   int16_t getTurnRadius(int32_t MaxRadius=0x7FFF) const
   { int32_t Radius = (int32_t)Speed*10436;
     if(Turn==0) return 0;                              // return zero for radius larger than maximum defined
     Radius/=Turn;
     if(abs(Radius)>MaxRadius) return 0;
     return Radius; }                                   // return turning radius [0.5m] = turning circle diameter [m]

   void getSpeedVector(int32_t &Vx, int32_t &Vy)        // [0.5m/s] [0.5m/s]
   { Vx = ((int32_t)Dx*Speed+0x800)>>12;                // Vx = DirX * Speed
     Vy = ((int32_t)Dy*Speed+0x800)>>12; }              // Vy = DirY * Speed

   void getSpeedVector(int16_t &Vx, int16_t &Vy)        // [0.5m/s] [0.5m/s]
   { Vx = ((int32_t)Dx*Speed+0x800)>>12;                // Vx = DirX * Speed
     Vy = ((int32_t)Dy*Speed+0x800)>>12; }              // Vy = DirY * Speed

   void StepFwd(int16_t Time)                           // [0.5s] predict the position into the future
   { int16_t Vx, Vy;
     int16_t Time1 = Time>>1;
     int16_t Time2 = Time-Time1;
     getSpeedVector(Vx, Vy);
     int16_t dX  = Time1*Vx; int16_t dY  = Time1*Vy;
     Heading += (Time*Turn)>>1;
     calcDir(); // calcAccel();
     getSpeedVector(Vx, Vy);
             dX += Time2*Vx;         dY += Time2*Vy;
     X += dX/2;
     Y += dY/2;
     Z += (Time*Climb)>>1;
     T += Time; }

   void StepFwdSecs(int16_t Secs)                       // [sec] predict the position Secs into the future
   { int16_t Vx, Vy;
     int16_t Secs1=Secs>>1;
     int16_t Secs2=Secs-Secs1;
     getSpeedVector(Vx, Vy);
     X += Secs1*Vx; Y += Secs1*Vy;
     Heading += Secs*Turn;
     calcDir(); // calcAccel();
     getSpeedVector(Vx, Vy);
     X += Secs2*Vx; Y += Secs2*Vy;
     Z += Secs*Climb;
     T += 2*Secs; }

   void StepFwd4secs(void)                             // predict the position four second into the future
   { int16_t Vx, Vy;
     getSpeedVector(Vx, Vy);
     X += 2*Vx; Y += 2*Vy;
     Heading += 4*Turn;
     calcDir(); // calcAccel();
     getSpeedVector(Vx, Vy);
     X += 2*Vx; Y += 2*Vy;
     Z += 4*Climb;
     T += 8; }

   void StepFwd2secs(void)                              // predict the position two seconds into the future
   { int16_t Vx, Vy;
     getSpeedVector(Vx, Vy);                            // get hor. speed vector from speed and dir. vector
     X += Vx; Y += Vy;                                  // incr. the hor. coordinates by half the speed
     Heading += 2*Turn;                                 // increment the heading by the turning rate
     calcDir(); // calcAccel();                         // recalc. direction and acceleration
     getSpeedVector(Vx, Vy);                            // recalc. the speed vector
     X += Vx; Y += Vy;                                  // incr. horizotal coord. by half the speed vector
     Z += 2*Climb;                                      // increment the rel. altitude by the climb rate
     T += 4; }                                          // increment time by 2sec

   void StepFwd1sec(void)                               // predict the position one second into the future
   { int16_t Vx, Vy;
     getSpeedVector(Vx, Vy);
     X += Vx/2; Y += Vy/2;
     Heading += Turn;
     calcDir(); // calcAccel();
     getSpeedVector(Vx, Vy);
     X += Vx/2; Y += Vy/2;
     Z += Climb;
     T += 2; }

   template <class OGNx_Packet>                          // read position from an OGN packet, use provided reference
    int32_t Read(OGNx_Packet &Packet, uint8_t RefTime, int32_t RefLat, int32_t RefLon, int32_t RefAlt, uint16_t LatCos=3000, int32_t MaxDist=10000)
   { T = (int16_t)Packet.Position.Time-(int16_t)RefTime;
     if(T<=(-30)) T+=60; else if(T>30) T-=60;
     T<<=1;
     int32_t LatDist, LonDist;
     if(Packet.calcDistanceVector(LatDist, LonDist, RefLat, RefLon, LatCos, MaxDist)<0) return -1;
     X = LatDist<<1;                                    // [m]      => [0.5m]
     Y = LonDist<<1;                                    // [m]      => [0.5m]
     Z = (Packet.DecodeAltitude()-RefAlt)<<1;           // [m]      => [0.5m]
     Speed = (Packet.DecodeSpeed()+2)/5;                // [0.1m/s] => [0.5m/s]
     Heading = Packet.getHeadingAngle();                // [360/0x10000deg]
     Climb = Packet.DecodeClimbRate()/5;                // [0.1m/s] => [0.5m/s]
     Turn = ((int32_t)Packet.DecodeTurnRate()*1165+32)>>6; // [0.1deg/s] => [360/0x10000deg/s] 
     calcDir();
     Error = (2*Packet.DecodeDOP()+22)/5;
     // calcAccel();
     // calcTurn();
     // printf("Read: "); Packet.Print();
     // Print();
     return 1; }

   template <class OGNx_Packet>                         // write position into the OGN packet, using given reference
    void Write(OGNx_Packet &Packet, uint8_t RefTime, int32_t RefLat, int32_t RefLon, int32_t RefAlt, uint16_t LatCos=3000)
   { int16_t Time=RefTime+(T>>1);
     // if(Time<0) Time+=60; else if(Time>=60) Time-=60;
     Packet.Position.Time = Time%60;
     Packet.setDistanceVector(X>>1, Y>>1, RefLat, RefLon, LatCos);
     Packet.EncodeAltitude(RefAlt+(Z>>1));              //
     Packet.clrBaro();                                  // don't know the standard pressure altitude
     Packet.EncodeSpeed(Speed*5);                       // [0.5m/s] => [0.1m/s]
     Packet.setHeadingAngle(Heading);                   //
     Packet.EncodeClimbRate(Climb*5);                   // [0.5m/s] => [0.1m/s]
     Packet.EncodeTurnRate((Turn*7+64)>>7);             // [360/0x10000deg/s] => [0.1deg/s]
     Packet.EncodeDOP((5*Error)/2-10);
   }

} ;

// =======================================================================================================

