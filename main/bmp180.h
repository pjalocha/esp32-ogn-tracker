#include <stdint.h>
#include <string.h>
#include <math.h>

#include "hal.h"

class BMP180
{ private:
  public:
   static const uint8_t ADDR         = 0x77; // BMP180 I2C address
  private:
   static const uint8_t REG_CALIB    = 0xAA; // calibration register: 11x16bit (MSB first)
   static const uint8_t REG_ID       = 0xD0; // ID register: always reads 0x55
   static const uint8_t REG_RESET    = 0xE0; // write 0xB6 to perform soft-reset
   static const uint8_t REG_MEAS     = 0xF4; // measurement control: SSCMMMMM SS=oversampling, C=conversion, MMMMM=mode
   static const uint8_t MEAS_TEMP    = 0x2E; // measure temperature
   static const uint8_t MEAS_PRESS   = 0x34; // measure pressure
   static const uint8_t MEAS_BUSY    = 0x20; // measurement-busy flag
   static const uint8_t REG_ADC      = 0xF6; // ADC result: 2 or 3 bytes
   static const uint8_t REG_ADC_MSB  = 0xF6; // ADC result: MSB
   static const uint8_t REG_ADC_LSB  = 0xF7; // ADC result: LSB
   static const uint8_t REG_ADC_XLSB = 0xF8; // ADC result: more LSB
   static const uint8_t MEAS_OSS     = 0x03; // oversampling factor: 0=single, 1=two, 2=four, 3=eight samples
  public:
   uint8_t Bus;         // which I2C bus
  private:
   union
   { int16_t Calib[11];
     struct
     { int16_t AC1, AC2, AC3;    // 11 calibration values from EEPROM
      uint16_t AC4, AC5, AC6;
       int16_t  B1,  B2;
       int16_t  MB,  MC,  MD;
     } ;
   } ;
   int32_t  B5;              // temperature compensation for pressure ?
  public:
     uint8_t Error;          //      error on the I2C bus (0=no error)
    uint16_t RawTemp;        //      raw temperature - to be processed
     int16_t Temperature;    // [0.1 degC] after processing
    uint32_t RawPress;       //      raw pressure - to be processed
    uint32_t Pressure;       // [    Pa  ] after processing

  private:
   static uint16_t SwapBytes(uint16_t Word) { return (Word>>8) | (Word<<8); }
   static uint16_t SwapBytes( int16_t Word) { return SwapBytes((uint16_t)Word); }
   static     void SwapBytes(uint16_t *Word, uint8_t Bytes)
   { uint8_t Words=Bytes>>1;
     for(uint8_t Idx=0; Idx<Words; Idx++)
       Word[Idx]=SwapBytes(Word[Idx]);
   }

  public:

  void DefaultCalib(void)    // set default calibration constants
  { AC1 =    408;
    AC2 =    -72;
    AC3 = -14383;
    AC4 =  32741;
    AC5 =  32757;
    AC6 =  23153;
    B1  =   6190;
    B2  =      4;
    MB  = -32768;
    MC  =  -8711;
    MD  =   2868; }

  uint8_t CheckID(void) // check ID, to make sure the BMP180 is connected and works correctly
   { uint8_t ID;
     Error=I2C_Read(Bus, ADDR, REG_ID, ID); if(Error) return Error;
     return ID!=0x55; } // 0 => no error and correct ID

  uint8_t ReadCalib(void) // read the calibration constants from the EEPROM
  { Error=I2C_Read(Bus, ADDR, REG_CALIB, (uint8_t *)Calib, 2*11); if(Error) return Error;
    SwapBytes((uint16_t *)Calib, 2*11 ); return Error; }

  uint8_t ReadReady(void) // check if temperature or pressure conversion is done
  { uint8_t MeasCtrl;
    Error=I2C_Read(Bus, ADDR, REG_MEAS, MeasCtrl); if(Error) return Error;
    return (MeasCtrl&MEAS_BUSY)==0; } // 0 = busy, 1 => ready

  uint8_t WaitReady(uint8_t Timeout=4, uint8_t Wait=4) // wait for the conversion to be ready
  { vTaskDelay(Wait);
    for(; Timeout; Timeout--)
    { uint8_t Err=ReadReady(); if(Err>1) return Err;
      if(Err==1) return 0;
      vTaskDelay(1); }
    return 0xFF; }

  uint8_t TriggRawTemperature(void)                                    // start a temperature measurement
  { uint8_t Ctrl=MEAS_TEMP;
    Error=I2C_Write(Bus, ADDR, REG_MEAS, Ctrl); return Error; }

  uint8_t ReadRawTemperature(void)
  { Error=I2C_Read(Bus, ADDR, REG_ADC, RawTemp); if(Error) return Error;
    RawTemp=SwapBytes(RawTemp); return 0; }

  uint8_t AcquireRawTemperature(void)                                  // 
  { uint8_t Err=TriggRawTemperature(); if(Err) return Err;
            Err=WaitReady(4, 4); if(Err) return Err;
    return ReadRawTemperature(); }

  uint8_t TrigRawPressure(void)                                        // start a pressure measurement
  { uint8_t Ctrl=(MEAS_OSS<<6) | MEAS_PRESS;
    Error=I2C_Write(Bus, ADDR, REG_MEAS, Ctrl); return Error; }

  uint8_t ReadRawPressure(void)
  { RawPress=0;
    Error=I2C_Read(Bus, ADDR, REG_ADC, (uint8_t *)(&RawPress), 3); if(Error) return Error;
    RawPress = ((RawPress<<16)&0xFF0000) | (RawPress&0x00FF00) | ((RawPress>>16)&0x0000FF);
    RawPress>>=(8-MEAS_OSS);
    return 0; }

  uint8_t AcquireRawPressure(void)
  { uint8_t Err=TrigRawPressure(); if(Err) return Err;
    uint8_t Timeout = (((uint8_t)1<<MEAS_OSS)+2)<<1;
    Err=WaitReady(Timeout, Timeout); if(Err) return Err;
    return ReadRawPressure(); }

// temperature and pressure calculation routines takes from the datasheet

  void CalcTemperature(void)   // calculate the temperature from raw readout and calibration values
  { int32_t X1 = ( (( (int32_t)RawTemp - (int32_t)AC6 )*(int32_t)AC5) )>>15;
    int32_t X2 = ((int32_t)MC<<11)/(X1+(int32_t)MD);
            B5 = X1+X2;
    Temperature = (B5+8)>>4; }

  void CalcPressure(void)      // calculate the pressure - must calculate the temperature first !
  { int32_t B6 = B5-4000;
    int32_t X1 = ((int32_t)B2*((B6*B6)>>12))>>11;
    int32_t X2 = ((int32_t)AC2*B6)>>11;
    int32_t X3 = X1+X2;
    int32_t B3 = (((((int32_t)AC1<<2)+X3)<<MEAS_OSS)+2)>>2;
            X1 = ((int32_t)AC3*B6)>>13;
            X2 = ((int32_t)B1*((B6*B6)>>12))>>16;
            X3 = ((X1+X2)+2)>>2;
   uint32_t B4 = ((uint32_t)AC4*(uint32_t)(X3+32768))>>15;
   uint32_t B7 = (RawPress-B3)*((uint32_t)50000>>MEAS_OSS);
    if(B7&0x8000000) { Pressure = (B7/B4)<<1; }
                else { Pressure = (B7<<1)/B4; }
            X1 = (Pressure>>8)*(Pressure>>8);
            X1 = (X1*3038)>>16;
            X2 = (-7357*(int32_t)Pressure)>>16;
            Pressure += (X1+X2+3791)>>4; }

} ;
