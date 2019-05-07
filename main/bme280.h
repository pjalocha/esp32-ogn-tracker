#ifndef __BME280_H__
#define __BME280_H__

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "nmea.h"
#include "format.h"

#include "bmp280.h"

class BME280: public BMP280
{ private:
    static uint16_t SwapBytes(uint16_t Word) { return (Word>>8) | (Word<<8); }

    static const uint8_t ADDR0        = 0x76; // possible I2C addresses
    static const uint8_t ADDR1        = 0x77;

    static const uint8_t REG_CALIB_A1 = 0xA1; // calibration register:
    static const uint8_t REG_CALIB_HUM= 0xE1; // calibration register:
    static const uint8_t REG_CTRL_HUM = 0xF2; // control: _____HHH HHH=humidity oversampling

    static const uint8_t REG_HUM        = 0xFD; // Humidity result:
    static const uint8_t REG_HUM_MSB    = 0xFD; // Humidity result: MSB
    static const uint8_t REG_HUM_LSB    = 0xFE; // Humidity result: LSB

    uint8_t  H1;
    int16_t  H2;
    uint8_t  H3;
    int16_t  H4;
    int16_t  H5;
    int8_t   H6;

  public:
    uint16_t RawHum;        //      raw humidity - to be processed
    int32_t Humidity;       // [0.1 %] relative humidity after processing
/*
  uint8_t CheckID(void) // check ID, to make sure the BME280 is connected and works correctly
   { uint8_t ID;
     ADDR=0;
     Error=I2C_Read(Bus, ADDR0, REG_ID, ID);
     if( (!Error) && (ID==0x60) ) { ADDR=ADDR0; return 0; }
     Error=I2C_Read(Bus, ADDR1, REG_ID, ID);
     if( (!Error) && (ID==0x60) ) { ADDR=ADDR1; return 0; }
     return 1; } // 0 => no error and correct ID
*/
  uint8_t ReadCalib(void) // read the calibration constants from the EEPROM
  { Error=BMP280::ReadCalib(); if(Error) return Error;
    if(hasHumidity())
    { Error=I2C_Read(Bus, ADDR, REG_CALIB_A1, (uint8_t *)(&H1), 1); if(Error) return Error;
      uint8_t buf[7];
      Error=I2C_Read(Bus, ADDR, REG_CALIB_HUM, buf, 7); if(Error) return Error;
      parse_humidity_calib_data(buf); }
    return 0; }

  void parse_humidity_calib_data(uint8_t *s)
  {
  int16_t lsb, msb;

  H2 = (int16_t)(((uint16_t)s[1] << 8) | (uint16_t)s[0]);
  H3 = s[2];

  msb = (int16_t)(int8_t)s[3] * 16;
  lsb = (int16_t)(s[4] & 0x0F);
  H4 = msb | lsb;

  msb = (int16_t)(int8_t)s[5] * 16;
  lsb = (int16_t)(s[4] >> 4);
  H5 = msb | lsb;
  H6 = (int8_t)s[6];
  }

  uint8_t Trigger(void)                                    // start  measurement
  { if(hasHumidity())
    { uint8_t Data=0x03; Error=I2C_Write(Bus, ADDR, REG_CTRL_HUM, Data); if(Error) return Error; } // H.osp=3x
    return BMP280::Trigger(); }

  uint8_t ReadRawHum(void)
  { RawHum=0;
    Error=I2C_Read(Bus, ADDR, REG_HUM, (uint8_t *)(&RawHum), 2); if(Error) return Error;
    RawHum = SwapBytes(RawHum);
    return 0; }

  // uint8_t Acquire(void)
  // { Error=BMP280::Acquire(); if(Error) return Error;
  //   if(hasHumidity()) return ReadRawHum();
  //   return 0; }

  uint8_t Acquire(void)
  { if(Trigger()) return Error;
    if(WaitReady()!=1) return Error;
    if(ReadRawTemp()) return Error;
    if(ReadRawPress()) return Error;
    if(hasHumidity()) return ReadRawHum();
    return 0; }

  void Calculate(void)
  { BMP280::Calculate();
    if(hasHumidity()) calcHumidity(); }

  void calcHumidity(void)   // calculate the humidity from raw readout and calibration values
  {
    int32_t var1, var2, var3, var4, var5;
    int32_t humidity_max = 100000;

    var1 = FineTemp - ((int32_t)76800);
    var2 = (int32_t)(((uint32_t)RawHum & 0xFFFF) << 14);
    var3 = (int32_t)(((int32_t)H4) << 20);
    var4 = ((int32_t)H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) >> 15;
    var2 = (var1 * ((int32_t)H6)) >> 10;
    var3 = (var1 * ((int32_t)H3)) >> 11;
    var4 = ((var2 * (var3 + (int32_t)32768)) >> 10) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)H2)) + 8192) >> 14;
    var3 = var5 * var2;
    var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
    var5 = var3 - ((var4 * ((int32_t)H1)) >> 4);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    Humidity = (uint32_t)(var5 >> 12);
    if (Humidity > humidity_max) Humidity = humidity_max;
    Humidity /= 100;
  }

} ;

#endif // __BME280_H__
