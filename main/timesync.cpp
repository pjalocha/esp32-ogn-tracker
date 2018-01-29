#include "timesync.h"

static TickType_t TimeSync_RefTick;    // reference point on the system tick
static uint32_t   TimeSync_RefTime;    // Time which corresponds to the above reference point

void TimeSync_HardPPS(TickType_t Tick)                                     // [ms] hardware PPS at the give system tick
{ TickType_t Incr = (Tick-TimeSync_RefTick+500)/1000;                      // [sec] home many full seconds to step forward
  TimeSync_RefTime += Incr;                                                // [sec] new time ref.
  TimeSync_RefTick = Tick; }                                               // [ms]  new tick ref.

void TimeSync_HardPPS(void) { TimeSync_HardPPS(xTaskGetTickCount()); }     //

void TimeSync_SoftPPS(TickType_t Tick, uint32_t Time, int32_t msOfs)       // [ms], [sec], [ms] software PPS: from GPS burst start or from MAV
{ Tick-=msOfs;                                                             // [ms]
  TickType_t Incr=(Tick-TimeSync_RefTick+500)/1000;                        // [sec]
  TimeSync_RefTime  = Time;                                                // [sec]
  TimeSync_RefTick += Incr*1000;                                           // [ms]
  // if(Tick>TimeSync_RefTick) TimeSync_RefTick++;
  // else if(Tick<TimeSync_RefTick) TimeSync_RefTick--;
  int32_t Diff = Tick-TimeSync_RefTick;
  TimeSync_RefTick += (Diff+8)>>4; }

void TimeSync_CorrRef(int16_t Corr)
{ TimeSync_RefTick += Corr; }

TickType_t TimeSync_msTime(TickType_t Tick)                                // [ms] get fractional time which corresponds to given system tick
{ TickType_t msTime=Tick-TimeSync_RefTick;
  if(msTime<1000) return msTime;
  if(msTime<2000) return msTime-1000;
  return msTime%1000; }

TickType_t TimeSync_msTime(void)                                           // [msec] get fractional time now
{ return TimeSync_msTime(xTaskGetTickCount()); }

uint32_t TimeSync_Time(TickType_t Tick)                                    // [sec] get Time which  corresponds to given system tick
{ int32_t Time=Tick-TimeSync_RefTick;
  // if(Time<0) return TimeSync_RefTime - ();
  if(Time<1000) return TimeSync_RefTime;
  if(Time<2000) return TimeSync_RefTime+1;
  return TimeSync_RefTime + (Time/1000); }

uint32_t TimeSync_Time(void)                                               // [sec] get Time now
{ return TimeSync_Time(xTaskGetTickCount()); }

