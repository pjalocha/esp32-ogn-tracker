#include "timesync.h"

static TickType_t TimeSync_RefTick;    // reference point on the system tick
static uint32_t   TimeSync_RefTime;    // Time which corresponds to the above reference point

void TimeSync_HardPPS(TickType_t Tick)                                     // [ms] hardware PPS at the give system tick
{ TickType_t Incr = (Tick-TimeSync_RefTick+500)/1000;                      // [sec] home many full seconds to step forward
  TimeSync_RefTime += Incr;                                                // [sec] new time ref.
  TimeSync_RefTick = Tick; }                                               // [ms]  new tick ref.

void TimeSync_HardPPS(void) { TimeSync_HardPPS(xTaskGetTickCount()); }     //

void TimeSync_SoftPPS(TickType_t Tick, uint32_t Time, int32_t msOfs)       // [ms], [sec], [ms] software PPS: from GPS burst start or from MAV
{
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "TimeSync_SoftPPS(");
  Format_UnsDec(CONS_UART_Write, Tick);
  Format_String(CONS_UART_Write, ", ");
  Format_UnsDec(CONS_UART_Write, Time);
  Format_String(CONS_UART_Write, ", ");
  Format_SignDec(CONS_UART_Write, msOfs);
  Format_String(CONS_UART_Write, ") => ");
  xSemaphoreGive(CONS_Mutex);
#endif
  Tick-=msOfs;                                                             // [ms]
  TickType_t Incr=(Tick-TimeSync_RefTick+500)/1000;                        // [sec]
  TimeSync_RefTime  = Time;                                                // [sec]
  TimeSync_RefTick += Incr*1000;                                           // [ms]
  // if(Tick>TimeSync_RefTick) TimeSync_RefTick++;
  // else if(Tick<TimeSync_RefTick) TimeSync_RefTick--;
  int32_t Diff = Tick-TimeSync_RefTick;
  TimeSync_RefTick += (Diff+8)>>4;
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_UnsDec(CONS_UART_Write, TimeSync_RefTick);
  Format_String(CONS_UART_Write, ", ");
  Format_UnsDec(CONS_UART_Write, TimeSync_RefTime);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
}

void TimeSync_CorrRef(int16_t Corr)
{ TimeSync_RefTick += Corr; }

TickType_t TimeSync_msTime(TickType_t Tick)                                // [ms] get fractional time which corresponds to given system tick
{ TickType_t msTime = Tick+1000-TimeSync_RefTick;
  if(msTime<1000) return msTime;
  if(msTime<2000) return msTime-1000;
  if(msTime<3000) return msTime-2000;
  return msTime%1000; }

TickType_t TimeSync_msTime(void)                                           // [msec] get fractional time now
{ return TimeSync_msTime(xTaskGetTickCount()); }

uint32_t TimeSync_Time(TickType_t Tick)                                    // [sec] get Time which  corresponds to given system tick
{ TickType_t msTime = Tick+1000-TimeSync_RefTick;
  if(msTime<1000) return TimeSync_RefTime-1;
  if(msTime<2000) return TimeSync_RefTime;
  if(msTime<3000) return TimeSync_RefTime+1;
  return TimeSync_RefTime-1 + (msTime/1000); }

uint32_t TimeSync_Time(void)                                               // [sec] get Time now
{ return TimeSync_Time(xTaskGetTickCount()); }

void TimeSync_Time(uint32_t &Time, TickType_t &msTime, TickType_t Tick)
{ TickType_t diffTime=Tick+1000-TimeSync_RefTick;
  // if(diffTime<   0) { Time = TimeSync_RefTime-1; msTime=diffTime+1000; return; }
  if(diffTime<1000) { Time = TimeSync_RefTime-1; msTime=diffTime     ; return; }
  if(diffTime<2000) { Time = TimeSync_RefTime  ; msTime=diffTime-1000; return; }
  if(diffTime<3000) { Time = TimeSync_RefTime+1; msTime=diffTime-2000; return; }
  TickType_t Diff = diffTime/1000; diffTime -= Diff*1000;
  Time = TimeSync_RefTime+Diff-1; msTime=diffTime; }

void TimeSync_Time(uint32_t &Time, TickType_t &msTime)
{ TimeSync_Time(Time, msTime, xTaskGetTickCount()); }
