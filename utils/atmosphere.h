#ifndef __ATMOSPHERE_H__
#define __ATMOSPHERE_H__

#include <math.h>
#include <stdint.h>

class Atmosphere
{ public:
   // int32_t Pressure;    // [    Pa  ]
   // int32_t Altitude;    // [0.1 m   ]
   // int32_t Temperature; // [0.1 degC]

   // altitude vs. pressure              //       20000, 30000, 40000, 50000, 60000, 70000, 80000, 90000, 100000, 110000     // [   Pa]
   static const int32_t StdAltTable[10]; //  = { 117764, 91652, 71864, 55752, 42070, 30126, 19493,  9886,   1109,  -6984 } ; // [0.1 m]

  public:
   // dH/dP = -R/g*T => R = 287.04m^2/K/sec^2, g = 9.80655m/s^2

    static int32_t PressureLapseRate(int32_t Pressure, int32_t Temperature=150) // [Pa], [0.1 degC] => [0.0001 m/Pa]
   { return -((int32_t)29270*(Temperature+2732)+Pressure/2)/Pressure; }
    // int32_t PressureLapseRate(void) { return PressureLapseRate(Pressure, Temperature); }

   static const int32_t StdPressureAtSeaLevel    = 101325; // [Pa]
   static const int32_t StdPressureAt11km        =  22632; // [Pa]
   static const int32_t StdPressureAt20km        =   5475; // [Pa]
   static const int32_t StdTemperatureLapseRate  =    -65; // [0.1 degC/1000m] valid till 11km
   static const int32_t StdTemperatureAtSeaLevel =    150; // [0.1 degC      ]
   static const int32_t StdTemperatureAt11km     =   -565; // [0.1 degC      ]

   static int32_t StdTemperature(int32_t Altitude)         // [0.1 m         ] valid till 20km
   { if(Altitude>110000) return StdTemperatureAt11km;
     return StdTemperatureAtSeaLevel+(StdTemperatureLapseRate*Altitude-5000)/10000; }

   static int32_t AltitudeDelta(int32_t PressureDelta, int32_t PressureLapseRate) // [Pa], [0.0001 m/Pa]
   { return (PressureDelta*PressureLapseRate)/100; }                              // [0.01m]

   static int32_t AltitudeDelta(int32_t PressureDelta, int32_t Pressure, int32_t Temperature) // [Pa], [Pa], [0.1degC]
   { int32_t PLR=PressureLapseRate(Pressure, Temperature); return AltitudeDelta(PressureDelta, PLR); } // [0.01m]

   static int32_t StdAltitude(int32_t Pressure, int32_t PressStep=100)            // [Pa]
   { int32_t Idx=(Pressure+5000)/10000; Idx-=2;
     if(Idx<0) Idx=0; else if(Idx>9) Idx=9;
     int32_t Press    = 10000*(Idx+2);
     int32_t Altitude = 10*StdAltTable[Idx];
     for( ; ; )
     { int32_t Temp=StdTemperature(Altitude/10);
       int32_t Delta=Pressure-Press; if(Delta==0) break;
       if(Delta>PressStep) Delta=PressStep;
       else if(Delta<(-PressStep)) Delta=(-PressStep);
       Altitude+=AltitudeDelta(Delta, Press, Temp);
       Press+=Delta;
     }
     return Altitude/10; }                                                          // [0.1m]

#ifdef NO_RTOS
   static int32_t StdAltitude_float(int32_t Pressure)
   { return floor(443300*(1-powf((float)Pressure/(float)101325.0, (float)0.190295))+0.5); }
#endif


} ;

#endif // __ATMOSPHERE_H__
