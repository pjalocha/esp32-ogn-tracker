#include <stdint.h>
#include <string.h>
#include <math.h>

#include "hal.h"

class BMX055_ACC
{ private:
   static const uint8_t ADDR0        = 0x18; // possible I2C addresses
   static const uint8_t ADDR1        = 0x19;

   static const uint8_t REG_ID       = 0x00;

  public:
   uint8_t Bus;                              // which I2C bus
   uint8_t ADDR;                             // detected I2C address
   uint8_t ID;                               // 0x58 for BMP280, 0x60 for BME280

  public:
   uint8_t Error;                            // error on the I2C bus (0=no error)

  uint8_t CheckID(void)                      // check ID
   { ADDR=0;
     Error=I2C_Read(Bus, ADDR0, REG_ID, ID);
     if( (!Error) && (ID==0xFA) ) { ADDR=ADDR0; return 0; }
     Error=I2C_Read(Bus, ADDR1, REG_ID, ID);
     if( (!Error) && (ID==0xFA) ) { ADDR=ADDR1; return 0; }
     return 1; } // 0 => no error and correct ID

} ;

class BMX055_GYR
{ private:
   static const uint8_t ADDR0        = 0x68; // possible I2C addresses
   static const uint8_t ADDR1        = 0x69;

   static const uint8_t REG_ID       = 0x00;

  public:
   uint8_t Bus;                              // which I2C bus
   uint8_t ADDR;                             // detected I2C address
   uint8_t ID;                               // 0x58 for BMP280, 0x60 for BME280

  public:
   uint8_t Error;                            // error on the I2C bus (0=no error)

  uint8_t CheckID(void)                      // check ID
   { ADDR=0;
     Error=I2C_Read(Bus, ADDR0, REG_ID, ID);
     if( (!Error) && (ID==0x0F) ) { ADDR=ADDR0; return 0; }
     Error=I2C_Read(Bus, ADDR1, REG_ID, ID);
     if( (!Error) && (ID==0x0F) ) { ADDR=ADDR1; return 0; }
     return 1; } // 0 => no error and correct ID

} ;

class BMX055_MAG
{ private:
   static const uint8_t ADDR0        = 0x10; // possible I2C addresses
   static const uint8_t ADDR1        = 0x11;
   static const uint8_t ADDR2        = 0x12;
   static const uint8_t ADDR3        = 0x13;

   static const uint8_t REG_ID       = 0x40;
   static const uint8_t REG_PWR      = 0x4B;

  public:
   uint8_t Bus;                              // which I2C bus
   uint8_t ADDR;                             // detected I2C address
   uint8_t ID;                               // 0x58 for BMP280, 0x60 for BME280

  public:
     uint8_t Error;                          // error on the I2C bus (0=no error)

  uint8_t CheckID(void)                      // power-up and check ID
   { ADDR=0;
     Error=I2C_Write(Bus, ADDR0, REG_PWR, 0x03);
     if(!Error) Error=I2C_Read(Bus, ADDR0, REG_ID, ID);
     if( (!Error) && (ID==0x32) ) { ADDR=ADDR0; return 0; }
     Error=I2C_Write(Bus, ADDR1, REG_PWR, 0x03);
     if(!Error) Error=I2C_Read(Bus, ADDR1, REG_ID, ID);
     if( (!Error) && (ID==0x32) ) { ADDR=ADDR1; return 0; }
     Error=I2C_Write(Bus, ADDR2, REG_PWR, 0x03);
     if(!Error) Error=I2C_Read(Bus, ADDR2, REG_ID, ID);
     if( (!Error) && (ID==0x32) ) { ADDR=ADDR2; return 0; }
     Error=I2C_Write(Bus, ADDR3, REG_PWR, 0x03);
     if(!Error) Error=I2C_Read(Bus, ADDR3, REG_ID, ID);
     if( (!Error) && (ID==0x32) ) { ADDR=ADDR3; return 0; }
     return 1; } // 0 => no error and correct ID

} ;

