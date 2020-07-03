#include <stdio.h>
#include <stdint.h>

#include "ogn.h"

// ===========================================================================================

class FlightMonitor
{ public:
   GPS_Position Takeoff;
   GPS_Position Landing;
   GPS_Position MaxAltitude;
   // GPS_Position Recent;
   // const char *IGCpath;
   static const uint8_t MinHold = 5;   // [sec]
   static const uint16_t MinSpeed = 60; // [0.1m/s]
   uint8_t HoldTime;
   uint8_t TakeoffCount;

  public:

   void Clear(void)
   { Takeoff.Clear();
     Landing.Clear();
     MaxAltitude.Clear();
     // Recent.Clear();
     // IGCpath=0;
     HoldTime=0;
     TakeoffCount=0; }

   static char NameCode(int Num)
   { if(Num<=0) return '0';
     if(Num<10) return '0'+Num;
     if(Num<36) return 'A'+(Num-10);
     return '_'; }

   int ShortName(char *Name, const char *Serial) const
   { int Len=0;
     Name[Len++]='0'+Takeoff.Year%10;
     Name[Len++]=NameCode(Takeoff.Month);
     Name[Len++]=NameCode(Takeoff.Day);
     Name[Len++]='O';
     Len+=Format_String(Name+Len, Serial);
     Name[Len++]=NameCode(TakeoffCount);
     Len+=Format_String(Name+Len, ".IGC");
     Name[Len]=0;
     // printf("ShortName[%d]: %s\n", Len, Name);
     return Len; }

   int LongName(char *Name, const char *Serial) const
   { int Len=0;
     Name[Len]=0; return 0; }

   static int FlightThresh(const GPS_Position &Position, uint16_t MinSpeed)
   { if(!Position.isValid()) return -1;
     uint16_t Speed=Position.Speed;                            // [0.1m/s]
     int16_t Climb=Position.ClimbRate;                         // [0.1m/s]
     uint8_t DOP=Position.PDOP; if(DOP==0) DOP=Position.HDOP;  // [0.1]
     Speed+=4*abs(Climb);
     if(DOP>10) { Speed = (Speed*10)/DOP; }
     return Speed>=MinSpeed; }

   bool inFlight(void) const { return Takeoff.isValid() && !Landing.isValid(); }

   int Process(const GPS_Position &Position)
   { Position.Print();
     if(inFlight())
     { int Det=FlightThresh(Position, MinSpeed/2); // printf("FlightThres() => %d\n", Det);
       if(Det<=0)
       { HoldTime++;
         if(HoldTime>=2*MinHold)
         { Landing=Position; HoldTime=0;
           char Name[16]; ShortName(Name, "XXX");
           printf("Landing #%d: %s\n", TakeoffCount, Name);
         }
       }
       else HoldTime=0;
     }
     else
     { int Det=FlightThresh(Position, MinSpeed); // printf("FlightThres() => %d\n", Det);
       if(Det>0)
       { HoldTime++;
         if(HoldTime>=MinHold)
         { Takeoff=Position; TakeoffCount++;
           Landing.Clear(); HoldTime=0;
           char Name[16]; ShortName(Name, "XXX");
           printf("Takeoff #%d: %s\n", TakeoffCount, Name);
         }
       }
       else HoldTime=0;
     }
     return inFlight(); }

} ;

// ===========================================================================================
