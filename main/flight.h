#include <stdio.h>
#include <stdint.h>

#include "ogn.h"

// ===========================================================================================

class FlightMonitor
{ public:
   GPS_Position Takeoff;                // est. takeoff position
   GPS_Position Landing;                // est. landing position
   GPS_Position MaxAltitude;            // est. positon at peak altitude
   // GPS_Position Recent;
   // const char *IGCpath;
   static const uint8_t  MinHold  =  5; // [sec] minimum hold time before takeoff declared
   static const uint16_t MinSpeed = 50; // [0.1m/s] minimum speed to trigger the takeoff
   uint8_t HoldTime;
   // uint8_t TakeoffCount;                // count take-off/landing cycles for the IGC file name

  public:

   void Clear(void)
   { Takeoff.Clear();
     Landing.Clear();
     MaxAltitude.Clear();
     // Recent.Clear();
     // IGCpath=0;
     HoldTime=0;
     // TakeoffCount=0;
   }

   static char Code36(int Num)       // coding of numbers in IGC file names
   { if(Num<=0) return '0';
     if(Num<10) return '0'+Num;
     if(Num<36) return 'A'+(Num-10);
     return '_'; }

   int ShortName(char *Name, uint8_t TakeoffNum, const char *Serial) const // produce short IGC file name (a three-character Serial)
   { return ShortName(Name, Takeoff, TakeoffNum, Serial); }

   static int ShortName(char *Name, const GPS_Position &Takeoff, uint8_t TakeoffNum, const char *Serial)
   { int Len=0;
     Name[Len++]='0'+Takeoff.Year%10;                  // Year (last digit)
     Name[Len++]=Code36(Takeoff.Month);                // encoded month
     Name[Len++]=Code36(Takeoff.Day);                  // encoded day
     Name[Len++]='O';                                  // OGN
     Len+=Format_String(Name+Len, Serial);             // three-letter serial
     Name[Len++]=Code36(TakeoffNum);                   // flight of the day
     Len+=Format_String(Name+Len, ".IGC");             // extension
     Name[Len]=0;
     // printf("ShortName[%d]: %s\n", Len, Name);
     return Len; }

   int LongName(char *Name, const char *Serial) const // produce short IGC file name
   { int Len=0;
     Name[Len]=0; return 0; }

   static int FlightThresh(const GPS_Position &Position, uint16_t MinSpeed) // does the GPS position meed the  in-flight criteria ?
   { if(!Position.isValid()) return -1;
     if(Position.Altitude>20000) return 1;                     // [0.1] Altitude above 2000m implies a flight
     uint16_t Speed=Position.Speed;                            // [0.1m/s]  Horizontal speed
     int16_t Climb=Position.ClimbRate;                         // [0.1m/s]  Vertical speed
     uint8_t DOP=Position.PDOP; if(DOP==0) DOP=Position.HDOP;  // [0.1]     Dilution of Precision
     Speed += 4*abs(Climb);                                    // [0.1m/s] take horizontal speed and four times the (absolute) vertical speed
     if(DOP>10) { Speed = (Speed*10)/DOP; }
     return Speed>=MinSpeed; }

   bool inFlight(void) const { return Takeoff.isValid() && !Landing.isValid(); }

   int Process(const GPS_Position &Position)                   // precess the GPS positions
   { // Position.Print();
     if(inFlight())                                            // if already in flight
     { int Det=FlightThresh(Position, MinSpeed/2);             // check in-flight criteria with half the limit
       if(Det<=0)                                              // if fail
       { HoldTime++;                                           // count the holding time
         if(HoldTime>=2*MinHold)                               // if over twice the limit
         { Landing=Position; HoldTime=0;                       // then declare landing, store landing position
           // char Name[16]; ShortName(Name, "XXX");
           // printf("Landing #%d: %s\n", TakeoffCount, Name);
         }
       }
       else HoldTime=0;                                        // if in-flight criteria satisfied then clear the holding time
     }
     else                                                      // if not in flight yet
     { int Det=FlightThresh(Position, MinSpeed);               // check in-flight criteria with normal limits
       if(Det>0)                                               // if criteria satisfied
       { HoldTime++;                                           // count the holding time
         if(HoldTime>=MinHold)                                 // if enough
         { Takeoff=Position; // TakeoffCount++;                // declare takeoff
           Landing.Clear(); HoldTime=0;                        // clear the landing position
           // char Name[16]; ShortName(Name, "XXX");
           // printf("Takeoff #%d: %s\n", TakeoffCount, Name);
         }
       }
       else HoldTime=0;                                        // if takeoff criteria not met, then clear the holding time and wait
     }
     return inFlight(); }

} ;

// ===========================================================================================
