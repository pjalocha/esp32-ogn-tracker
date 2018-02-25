#ifndef __TIMESYNC_H__
#define __TIMESYNC_H__

#include <stdint.h>

#include "hal.h"

void TimeSync_HardPPS(TickType_t Tick);                                     // hardware PPS at the give system tick
void TimeSync_HardPPS(void);

void TimeSync_SoftPPS(TickType_t Tick, uint32_t Time, int32_t msOfs=100);   // software PPS: from GPS burst start or from MAV

TickType_t TimeSync_msTime(TickType_t Tick);                                // [ms] get fractional time which corresponds to given system tick
TickType_t TimeSync_msTime(void);

uint32_t TimeSync_Time(TickType_t Tick);                                    // [sec] get Time which  corresponds to given system tick
uint32_t TimeSync_Time(void);

void TimeSync_CorrRef(int16_t Corr);                                        // [ms] correct the time reference [RTOS tick]

#endif // __TIMESYNC_H__
