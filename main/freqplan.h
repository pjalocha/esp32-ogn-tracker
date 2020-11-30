#ifndef __FREQPLAN_H__
#define __FREQPLAN_H__

#include <stdint.h>

class FreqPlan
{ public:
   uint8_t       Plan;  // 1=Europe, 2=USA/Canada, 3=Australia/Chile, 4=New Zeeland
   // char      Name[16];
   uint8_t   Channels;  // number of channels
   uint32_t  BaseFreq;  // [Hz] base channel (#0) frequency
   uint32_t ChanSepar;  // [Hz] channel spacing
   static const uint8_t MaxChannels=65;

  public:
   void setPlan(uint8_t NewPlan=0) // preset for a given frequency plan
   { Plan=NewPlan;
          if(Plan==2) { BaseFreq=902200000; ChanSepar=400000; Channels=65; } // USA
     else if(Plan==3) { BaseFreq=917000000; ChanSepar=400000; Channels=24; } // Australia and South America
     else if(Plan==4) { BaseFreq=869250000; ChanSepar=200000; Channels= 1; } // New Zeeland
     else if(Plan==5) { BaseFreq=916200000; ChanSepar=200000; Channels= 1; } // Israel
     else if(Plan==6) { BaseFreq=433200000; ChanSepar=200000; Channels= 8; } // Europe/Africa 434MHz
     else             { BaseFreq=868200000; ChanSepar=200000; Channels= 2; } // Europe/Africa 868MHz
   }

   void setPlan(int32_t Latitude, int32_t Longitude)
   { setPlan(calcPlan(Latitude, Longitude)); }

   const char *getPlanName(void) { return getPlanName(Plan); }                 // get the name of the given frequency plan
   uint32_t getCenterFreq(void) { return BaseFreq + ChanSepar/2*(Channels-1); } // get the center frequency for the given frequency plan

   static const char *getPlanName(uint8_t Plan)
   { static const char *Name[7] = { "Default", "EU/Africa", "USA/Canada", "Australia/Chile", "New Zeeland", "Israel", "EU/Africa 434MHz" } ;
     if(Plan>=7) return 0;
     return Name[Plan]; }

   uint8_t getChannel  (uint32_t Time, uint8_t Slot=0, uint8_t OGN=1) const // OGN-tracker or FLARM, UTC time, slot: 0 or 1
   { if(Channels<=1) return 0;                                              // if single channel (New Zeeland) return channel #0
     if(Plan>=2)                                                            // if USA/Canada or Australia/South America
     { uint8_t Channel = FreqHopHash((Time<<1)+Slot) % Channels;            // Flarm hopping channel
       if(OGN)                                                              // OGN Tracker
       { if(Slot)                                                           // for 2nd slot
         { uint8_t Channel2 = FreqHopHash((Time<<1)) % Channels;            // use same as Flarm in the 1st slot
           if(Channel2==Channel) { Channel++; if(Channel>=Channels) Channel-=2; } // but if same then Flarm in the 2nd slot
                            else { Channel=Channel2; }
         }
         else { Channel++; if(Channel>=Channels) Channel-=2; }              // for 1st slot choose a higher channel (unless already highest, then choose a lower one)
       }
       return Channel; }                                                    // return 0..Channels-1 for USA/CA or Australia.
     return Slot^OGN; }                                                     // if Europe/South Africa: return 0 or 1 for EU freq. plan

   uint32_t getChanFrequency(int Channel) const { return BaseFreq+ChanSepar*Channel; }

   uint32_t getFrequency(uint32_t Time, uint8_t Slot=0, uint8_t OGN=1) const
   { uint8_t Channel=getChannel(Time, Slot, OGN); return BaseFreq+ChanSepar*Channel; } // return frequency [Hz] for given UTC time and slot

   uint32_t getFreqPAW(uint32_t Time)
   { if(Plan<=1) return 869525000;
     return 0; }

   uint32_t getFreqFNT(uint32_t Time)
   { if(Plan<=1) return BaseFreq;                                          // Europe and default is 868.2MHz
     uint32_t Freq1 = getFrequency(Time, 0, 0);
     if(Plan==5) return Freq1;                                             // for 434MHz is same as "FLARM", which is never used there
     uint32_t Freq2 = getFrequency(Time, 0, 1);
     return (Freq1+Freq2)/2; }                                             // other hopping systems is half-way between FLARM and OGN

   uint8_t static calcPlan(int32_t Latitude, int32_t Longitude) // get the frequency plan from Lat/Lon: 1 = Europe + Africa, 2 = USA/CAnada, 3 = Australia + South America, 4 = New Zeeland
   { if( (Longitude>=(-20*600000)) && (Longitude<=(60*600000)) ) return 1; // between -20 and 60 deg Lat => Europe + Africa: 868MHz band
     if( Latitude<(20*600000) )                                            // below 20deg latitude
     { if( ( Longitude>(164*600000)) && (Latitude<(-30*600000)) && (Latitude>(-48*600000)) ) return 4;  // => New Zeeland
       return 3; }                                                         // => Australia + South America: upper half of 915MHz band
     return 2; }                                                           // => USA/Canada: full 915MHz band

  private:
   static uint32_t FreqHopHash(uint32_t Time)
   { Time  = (Time<<15) + (~Time);
     Time ^= Time>>12;
     Time += Time<<2;
     Time ^= Time>>4;
     Time *= 2057;
     return Time ^ (Time>>16); }

} ;

#endif // __FREQPLAN_H__
