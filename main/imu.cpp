#include "hal.h"
#include "sens.h"
#include "imu.h"

#include "timesync.h"

#include "parameters.h"

#include "proc.h"
#include "ctrl.h"
#include "gps.h"

#ifdef WITH_BMX055

#include "bmx055.h"

BMX055_MAG MAG;
BMX055_ACC ACC;
BMX055_GYR GYR;

static uint8_t InitIMU(void)
{ MAG.Bus=BARO_I2C;
  ACC.Bus=BARO_I2C;
  GYR.Bus=BARO_I2C;
  uint8_t  Err  = MAG.CheckID(); // Err<<=1;
  // if(!Err) Err |= ACC.CheckID(); Err<<=1;
  // if(!Err) Err |= GYR.CheckID();
  return Err; }

static void ProcIMU(void)
{
  int16_t Sec  = 10*(TimeSync_Time()%60);                               // [0.1sec]
  uint16_t Phase = TimeSync_msTime();                                   // sync to the GPS PPS
  if(Phase>=500) { Sec+=10; vTaskDelay(1000-Phase); }                   // wait till start of the measuring period
            else { Sec+= 5; vTaskDelay( 500-Phase); }
  if(Sec>=600) Sec-=600;                                                // [0.1sec] measurement time

}

#endif

extern "C"
void vTaskIMU(void* pvParameters)
{ vTaskDelay(100);   // ?

#ifdef WITH_BMX055
  uint8_t Detected = InitIMU();
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "TaskIMU: ");
  Format_Hex(CONS_UART_Write, Detected); CONS_UART_Write(':'); CONS_UART_Write(' ');
  Format_Hex(CONS_UART_Write, MAG.ADDR); CONS_UART_Write(',');
  Format_Hex(CONS_UART_Write, ACC.ADDR); CONS_UART_Write(',');
  Format_Hex(CONS_UART_Write, GYR.ADDR);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif

  while(1)
  {
#ifdef WITH_BMX055
    if(PowerMode) ProcIMU();
             else vTaskDelay(100);
#else
    vTaskDelay(1000);
#endif
  }
}
