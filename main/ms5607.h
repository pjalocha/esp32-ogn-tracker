#include <stdint.h>
#include <string.h>
#include <math.h>

#include "hal.h"

class MS5607
{ private:
   static const uint8_t ADDR0         = 0x76; // possible I2C addresses
   static const uint8_t ADDR1         = 0x77;

   static const uint8_t CMD_RESET     = 0x1E; // ADC reset command
   static const uint8_t CMD_ADC_READ  = 0x00; // ADC read command
   static const uint8_t CMD_ADC_CONV  = 0x40; // ADC conversion command
   static const uint8_t CMD_ADC_D1    = 0x00; // ADC D1 conversion
   static const uint8_t CMD_ADC_D2    = 0x10; // ADC D2 conversion
   static const uint8_t CMD_ADC_256   = 0x00; // ADC OSR=256
   static const uint8_t CMD_ADC_512   = 0x02; // ADC OSR=512
   static const uint8_t CMD_ADC_1024  = 0x04; // ADC OSR=1024
   static const uint8_t CMD_ADC_2048  = 0x06; // ADC OSR=2048
   static const uint8_t CMD_ADC_4096  = 0x08; // ADC OSR=4096
   static const uint8_t CMD_PROM_READ = 0xA0; // Prom read command
  public:
   uint8_t Bus;                              // which I2C bus
   uint8_t ADDR;                             // detected I2C address
  private:
   union
   { int16_t Calib[8];
     struct
     { uint16_t C0, C1, C2, C3, C4, C5, C6, C7; };               // 6 calibration values from EEPROM
   } ;
  public:
     uint8_t Error;          //      error on the I2C bus (0=no error)
     int32_t RawTemp;        //      raw temperature - to be processed
     int32_t RawPress;       //      raw pressure - to be processed
     int32_t Temperature;    // [0.1 degC] temperature after correction
    uint32_t Pressure;       // [0.25Pa  ] pressure after correction

  public:

  void DefaultCalib(void)    // set default calibration constants
  { C0 =       0;
    C1 =   46372;
    C2 =   43981;
    C3 =   29059;
    C4 =   27842;
    C5 =   31553;
    C6 =   28165;
    C7 =       0; }

  uint8_t Reset(uint8_t Addr)                           // RESET: takes 2.8ms
  { Error=I2C_Write(Bus, Addr, CMD_RESET, 0, 0);
    return Error; }

  uint8_t CheckID(void)
  { ADDR=0;
    Error=Reset(ADDR0); if(!Error) { vTaskDelay(3); ADDR=ADDR0; return 0; }
    Error=Reset(ADDR1); if(!Error) { vTaskDelay(3); ADDR=ADDR1; return 0; }
    return 1; }

  uint16_t SwapBytes(uint16_t Word)
  { return (Word<<8) | (Word>>8); }

  uint8_t ReadCalib(void)                         // read the calibration constants from the PROM
  { for(uint8_t Idx=0; Idx<8; Idx++)
    { Error=I2C_Read(Bus, ADDR, CMD_PROM_READ + 2*Idx, (uint8_t *)(Calib+Idx), 2); if(Error) break;
      Calib[Idx]=SwapBytes(Calib[Idx]); }
    return 0; }

  uint8_t ReadRawPress(void)                     // convert and read raw pressure
  { RawPress=0;
    Error=I2C_Write(Bus, ADDR, CMD_ADC_CONV+CMD_ADC_D1+CMD_ADC_4096, 0, 0); if(Error) return Error;
    vTaskDelay(12);
    Error=I2C_Read(Bus, ADDR, CMD_ADC_READ, (uint8_t *)(&RawPress), 3); if(Error) return Error;
    RawPress = ((RawPress<<16)&0xFF0000) | (RawPress&0x00FF00) | ((RawPress>>16)&0x0000FF); // swap bytes
    return 0; }

  uint8_t ReadRawTemp(void)                     // convert and read raw temperature
  { RawTemp=0;
    Error=I2C_Write(Bus, ADDR, CMD_ADC_CONV+CMD_ADC_D2+CMD_ADC_4096, 0, 0); if(Error) return Error;
    vTaskDelay(12);
    Error=I2C_Read(Bus, ADDR, CMD_ADC_READ, (uint8_t *)(&RawTemp), 3); if(Error) return Error;
    RawTemp = ((RawTemp<<16)&0xFF0000) | (RawTemp&0x00FF00) | ((RawTemp>>16)&0x0000FF); // swap bytes
    return 0; }

  uint8_t Acquire(void)                        // convert and read raw pressure then temperature
  { if(ReadRawPress()) return Error;
    return ReadRawTemp(); }

  void Calculate(void)                             // process temperature and pressure with the calibration constants
  { int32_t dT = RawTemp - ((int32_t)C5<<8);
    int32_t TEMP = 2000 + (((int64_t)dT*C6)>>23);  // [0.01degC]
    int64_t OFF = ((uint64_t)C2<<17) + (((int64_t)dT*C4)>>6);
    int64_t SENS = ((uint32_t)C1<<16) + (((int64_t)dT*C3)>>7);
    if(TEMP<2000)
    { int32_t dT2 = ((int64_t)dT*dT)>>31;
      int32_t OFF2 = (61*(TEMP-2000)*(TEMP-2000))>>4;
      int32_t SENS2 = 2*(TEMP-2000)*(TEMP-2000);
      if(TEMP<(-1500))
      { OFF2  += 15*(TEMP+1500)*(TEMP+1500);
        SENS2 += 8*(TEMP+1500)*(TEMP+1500); }
      TEMP -= dT2;
      OFF  -= OFF2;
      SENS -= SENS2; }
    Temperature = (TEMP+5)/10;                      // [0.1degC]
    Pressure = (((SENS*RawPress)>>21) - OFF)>>13; } // [0.25Pa]

} ;

