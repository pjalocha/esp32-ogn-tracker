#include <stdint.h>
#include <string.h>
#include <math.h>

#include "hal.h"

class BMP280
{ private:
   static const uint8_t ADDR0        = 0x76; // possible I2C addresses
   static const uint8_t ADDR1        = 0x77;

   static const uint8_t REG_CALIB    = 0x88; // calibration register:
   static const uint8_t REG_ID       = 0xD0; // ID register: always reads 0x58
   static const uint8_t REG_RESET    = 0xE0; // write 0xB6 to perform soft-reset
   static const uint8_t REG_STATUS   = 0xF3; // status: ____C__I C=conversion in progress
   static const uint8_t REG_CTRL     = 0xF4; // control: TTTPPPMM TTT=temp. oversampling, PPP=pressure oversampling, MM=power mode
   static const uint8_t REG_CONFIG   = 0xF5; // config: TTTFFF_S TTT=? FFF=Filter S=3-wire SPI

   static const uint8_t REG_PRESS      = 0xF7; // Pressure result:
   static const uint8_t REG_PRESS_MSB  = 0xF7; // Pressure result: MSB
   static const uint8_t REG_PRESS_LSB  = 0xF8; // Pressure result: LSB
   static const uint8_t REG_PRESS_XLSB = 0xF9; // Pressure result: more LSB

   static const uint8_t REG_TEMP      = 0xFA; // Temperature result:
   static const uint8_t REG_TEMP_MSB  = 0xFA; // Temperature result: MSB
   static const uint8_t REG_TEMP_LSB  = 0xFB; // Temperature result: LSB
   static const uint8_t REG_TEMP_XLSB = 0xFC; // Temperature result: more LSB
  public:
   uint8_t Bus;                              // which I2C bus
   uint8_t ADDR;                             // detected I2C address
  private:
   union
   { int16_t Calib[13];
     struct
     { uint16_t T1;                // 13 calibration values from EEPROM
        int16_t T2, T3;
       uint16_t P1;
        int16_t P2, P3, P4, P5, P6, P7, P8, P9;
        int16_t D;
     } ;
   } ;
  public:
     uint8_t Error;          //      error on the I2C bus (0=no error)
     int32_t RawTemp;        //      raw temperature - to be processed
     int32_t FineTemp;       //      for pressure calc.
     int16_t Temperature;    // [0.1 degC] after processing
     int32_t RawPress;       //      raw pressure - to be processed
    uint32_t Pressure;       // [0.25Pa  ] after processing

  public:

  void DefaultCalib(void)    // set default calibration constants
  { T1 =   27504;
    T2 =   26435;
    T3 =   -1000;
    P1 =   36477;
    P2 =  -10685;
    P3 =    3024;
    P4  =   2855;
    P5  =    140;
    P6  =     -7;
    P7  =  15500;
    P8  = -14600;
    P9  =   6000;
    D   =      0; }

  uint8_t CheckID(void) // check ID, to make sure the BMP280 is connected and works correctly
   { uint8_t ID;
     ADDR=0;
     Error=I2C_Read(Bus, ADDR0, REG_ID, ID);
     if( (!Error) && ( (ID==0x58) || (ID==0x60) ) ) { ADDR=ADDR0; return 0; }
     Error=I2C_Read(Bus, ADDR1, REG_ID, ID);
     if( (!Error) && ( (ID==0x58) || (ID==0x60) ) ) { ADDR=ADDR1; return 0; }
     return 1; } // 0 => no error and correct ID

  uint8_t ReadCalib(void) // read the calibration constants from the EEPROM
  { Error=I2C_Read(Bus, ADDR, REG_CALIB, (uint8_t *)Calib, 2*13);
    return Error; }

  uint8_t ReadReady(void) // check if temperature and pressure conversion is done
  { uint8_t Status;
    Error=I2C_Read(Bus, ADDR, REG_STATUS, Status); if(Error) return Error;
    return (Status&0x09)==0; } // 1 = ready, 0 => busy

  uint8_t WaitReady(uint8_t Timeout=50, uint8_t Wait=30) // wait for the conversion to be ready
  { vTaskDelay(Wait);
    for(; Timeout; Timeout--)
    { uint8_t Err=ReadReady(); if(Err) return Err; // 1 is ready, >1 is an I2C bus error
      vTaskDelay(1); }
    return 0xFF; }            // return "timeout" error

  uint8_t Trigger(void)                                    // start a temperature+pressure measurement
  { uint8_t Data=0x00; Error=I2C_Write(Bus, ADDR, REG_CONFIG, Data); if(Error) return Error;
            Data=0x55; Error=I2C_Write(Bus, ADDR, REG_CTRL,   Data); return Error; } // P.osp=16x, T.osp=2x

  uint8_t ReadRawPress(void)
  { RawPress=0;
    Error=I2C_Read(Bus, ADDR, REG_PRESS, (uint8_t *)(&RawPress), 3); if(Error) return Error;
    RawPress = ((RawPress<<16)&0xFF0000) | (RawPress&0x00FF00) | ((RawPress>>16)&0x0000FF);
    RawPress>>=4;
    return 0; }

  uint8_t ReadRawTemp(void)
  { RawTemp=0;
    Error=I2C_Read(Bus, ADDR, REG_TEMP, (uint8_t *)(&RawTemp), 3); if(Error) return Error;
    RawTemp = ((RawTemp<<16)&0xFF0000) | (RawTemp&0x00FF00) | ((RawTemp>>16)&0x0000FF);
    RawTemp>>=4;
    return 0; }

  uint8_t Acquire(void)
  { if(Trigger()) return Error;
    if(WaitReady()!=1) return Error;
    if(ReadRawTemp()) return Error;
    return ReadRawPress(); }

  void Calculate(void)
  { CalcTemperature();
    CalcPressure(); }

  void CalcTemperature(void)   // calculate the temperature from raw readout and calibration values
  { int32_t var1 = ((((RawTemp>>3) - ((int32_t)T1<<1))) * ((int32_t)T2)) >> 11;
    int32_t var2 = ( ( (((RawTemp>>4) - (int32_t)T1) * ((RawTemp>>4) - (int32_t)T1) ) >> 12) * ((int32_t)T3) ) >> 14;
    FineTemp = var1 + var2;
    Temperature = ( (var1+var2) + 256) >> 9;      // [0.1degC]
    // Temperature = ( (var1+var2) * 5 + 128) >> 8; // [0.01degC]
  }

  void CalcPressure(void)      // calculate the pressure - must calculate the temperature first !
  {
    int64_t var1 = ((int64_t)FineTemp) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)P6;
            var2 = var2 + ((var1*(int64_t)P5)<<17);
            var2 = var2 + (((int64_t)P4)<<35);
            var1 = ((var1 * var1 * (int64_t)P3)>>8) + ((var1 * (int64_t)P2)<<12);
            var1 = (((((int64_t)1)<<47)+var1))*((int64_t)P1)>>33;
    if (var1 == 0) { Pressure=0; return; }
    int64_t p = 1048576-RawPress;
            p = (((p<<31)-var2)*3125)/var1;
            var1 = (((int64_t)P9) * (p>>13) * (p>>13)) >> 25;
            var2 = (((int64_t)P8) * p) >> 19;
            p = ((p + var1 + var2) >> 8) + (((int64_t)P7)<<4);
    Pressure = ((p+32)>>6);
  }

} ;
