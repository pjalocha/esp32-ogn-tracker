#include <stdlib.h>

#include "hal.h"

#include "knob.h"

#include "format.h"

#ifdef WITH_KNOB

volatile uint8_t KNOB_Tick=15;

#ifdef __cplusplus
  extern "C"
#endif
void vTaskKNOB(void* pvParameters)
{

  uint8_t Tick=KNOB_Tick;
  for( ; ; )
  { vTaskDelay(40);
#ifdef WITH_STM32
    xSemaphoreTake(ADC1_Mutex, portMAX_DELAY);            // request
    uint16_t Knob     = ADC_Read_Knob();                  // read knob position: 0..2047
    xSemaphoreGive(ADC1_Mutex);
#endif
#ifdef WITH_ESP32
    uint16_t Knob = KnobSense();
#endif
    uint16_t PrevKnob = ((uint16_t)Tick<<8)+0x80;
     int16_t Err      = Knob-PrevKnob;
    if(abs(Err)>=(0x80+0x20))                              // 0x20 is the histeresis to avoid noisy input
    { KNOB_Tick = (Tick = (Knob>>8)); Play(Play_Vol_1 | Play_Oct_1 | Tick, 5);
      // if(KNOB_Tick&1) LED_BAT_On();
      //           else LED_BAT_Off();
      // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      // Format_UnsDec(CONS_UART_Write, (uint16_t)Tick);
      // CONS_UART_Write('\r'); CONS_UART_Write('\n');
      // xSemaphoreGive(CONS_Mutex);
    }

  }

}

#endif // WITH_KNOB
