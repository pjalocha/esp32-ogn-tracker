#include <stdint.h>
#include <stdlib.h>

#include "hal.h"
#include "aero.h"

class AEROmsg
{ public:

   static const uint8_t MaxLen=96;   // maximum length
   static const uint8_t MaxParms=16; // maximum number of parameters (commas)

   uint8_t Data[MaxLen];             // the message itself
   uint8_t Len;                      // number of bytes
   uint8_t Parms;                    // number of commas
   uint8_t Parm[MaxParms];           // offset to each comma
   uint8_t State;                    // bits: 0:loading, 1:complete, 2:locked,

  public:
   void Clear(void)                          // Clear the frame: discard all data, ready for next message
   { State=0; Len=0; Parms=0; }

   void ProcessByte(uint8_t Byte)            // pass all bytes through this call and it will build the frame
   {
     if(isComplete()) return;                // if already a complete frame, ignore extra bytes
     if(Len==0)                              // if data is empty
     { if(Byte!='#') return;                 // then ignore all bytes but '#'
       Data[Len++]=Byte;                     // start storing the frame
       setLoading(); Parms=0;                // set state to "isLoading", clear checksum
     } else                                  // if not empty (being loaded)
     { if((Byte=='\r')||(Byte=='\n'))        // if CR (or NL ?) then frame is complete
       { setComplete(); if(Len<MaxLen) Data[Len]=0;
         return; }
       else if(Byte<=' ')                    // other control bytes treat as errors
       { Clear(); return; }                  // and drop the frame
       else if(Byte==',')                    // save comma positions to later get back to the fields
       { if(Parms<MaxParms) Parm[Parms++]=Len+1; }
       if(Len<MaxLen) { Data[Len++]=Byte; }  // store data but if too much then treat as an error
                 else Clear();               // if too long, then drop the frame completely
     }
     return; }

   uint8_t isLoading(void) const  { return State &0x01; }
   void   setLoading(void)        {        State|=0x01; }

   uint8_t isComplete(void) const { return State &0x02; }
   void   setComplete(void)       {        State|=0x02; }

   uint8_t isLocked(void) const   { return State&0x04; }

   uint8_t isEmpty(void) const    { return Len==0; }

   uint8_t isChecked(void) const    // is the AERO checksum OK ?
   { if(Data[Len-5]!=',') return 0;
     uint16_t CRC; if(Read_Hex(CRC, (const char *)(Data+(Len-4)))!=4) return 0;
     return CRC == CRC16(Data, Len-5); }

   static uint16_t CRC16(uint16_t CRC, uint8_t Byte)
   { uint8_t X = (CRC>>8) ^ Byte; X ^= X>>4;
     CRC = (CRC<<8) ^ ((uint16_t)(X<<12)) ^ ((uint16_t)(X <<5)) ^ ((uint16_t)X);
     return CRC; }

   static uint16_t CRC16(const uint8_t *Data, uint8_t Len)
   { uint16_t CRC = 0xFFFF;
     while(Len--)
     { CRC = CRC16(CRC, *Data++); }
     return (CRC>>8) | (CRC<<8); }

} ;

static AEROmsg AERO;

static void ADSB_AERO(bool GoodCRC)
{
// #ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  // Format_UnsDec(CONS_UART_Write, TimeSync_Time()%60);
  // CONS_UART_Write('.');
  // Format_UnsDec(CONS_UART_Write, TimeSync_msTime(),3);
  // Format_String(CONS_UART_Write, " -> ");
  Format_String(CONS_UART_Write, (const char *)AERO.Data, AERO.Len, 0);
  if(!GoodCRC) Format_String(CONS_UART_Write, " <- bad !");
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
// #endif

}

#ifdef __cplusplus
  extern "C"
#endif
void vTaskAERO(void* pvParameters)
{

  vTaskDelay(5);                                                         // put some initial delay for lighter startup load

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "TaskAERO:");
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);

  for( ; ; )                                                              // main task loop: every milisecond (RTOS time tick)
  { vTaskDelay(1);                                                        //
    for( ; ; )                                                            // loop over bytes in the AERO UART buffer
    { uint8_t Byte; int Err=AERO_UART_Read(Byte); if(Err<=0) break;       // get Byte from serial port, if no bytes then break this l$
      AERO.ProcessByte(Byte);                                             // process through the AERO interpreter
      if(AERO.isComplete())                                               // AERO completely received ?
      { ADSB_AERO(AERO.isChecked());
        AERO.Clear(); break; }
    }

  }

}

