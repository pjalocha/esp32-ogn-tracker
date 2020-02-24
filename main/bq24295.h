#ifndef __BQ24295_H__
#define __BQ24295_H__

#include <stdint.h>

class BQ24295
{ private:
   static const uint8_t BQ24295_ADDR     = 0x6B; // possible I2C addresses

   static const uint8_t REG_SOURCE       = 0x00;
   static const uint8_t REG_POWERON      = 0x01;
   static const uint8_t REG_CHG_CURR     = 0x02;  // charging current
   static const uint8_t REG_PRE_CHG      = 0x03;  // pre-/post- charge current
   static const uint8_t REG_CHG_VOLT     = 0x04;
   static const uint8_t REG_TIMER_CTRL   = 0x05;
   static const uint8_t REG_BOOST        = 0x06;
   static const uint8_t REG_MISC         = 0x07;
   static const uint8_t REG_STATUS       = 0x08;  // 
   static const uint8_t REG_FAULT        = 0x09;
   static const uint8_t REG_ID           = 0x0A;  // reads 0x23 but should read 0xC0

   static const uint8_t BQ24295_ID       = 0xC0; // ID for the BQ24295, but the chip returns 0x23 ?

  public:
   uint8_t Bus;                              // which I2C bus
   uint8_t ADDR;                             // detected I2C address
   uint8_t ID;

  public:
   uint8_t Error;                            // error on the I2C bus (0=no error)

   uint8_t readReg(uint8_t Reg)
   { uint8_t Byte=0; Error=I2C_Read(Bus, ADDR, Reg, Byte); return Byte; }

   uint8_t readStatus(void) { return readReg(REG_STATUS); }  // VVCCDPTS  VV=Vbus status, CC=Charge status, D=DPM, P=Power-Good, T=Thermal reg., S=Vsys-min reg.
   uint8_t readFault (void) { return readReg(REG_FAULT); }   // WOCCBrNN  W=Watchdog, O=OTG, CC=charge fault, NN=NTC(temperature)
   uint8_t readSource(void) { return readReg(REG_SOURCE); }  //
   uint8_t readID    (void) { return readReg(REG_ID); }

   uint8_t checkID(void) // check ID, to make sure the BQ24295 is reachable
   { ADDR=0;
     Error=I2C_Read(Bus, BQ24295_ADDR, REG_ID, ID);
     if( (!Error) /* && (ID==BQ24295_ID) */ ) { ADDR=BQ24295_ADDR; return 0; }
     return 1; } // 0 => no error and correct ID

   uint8_t writeSource(uint8_t Byte=0x05)  // disable, 3.88V, 1.5A
   { Error=I2C_Write(Bus, ADDR, REG_SOURCE, Byte);
     return Error; }

   uint8_t writePowerON(uint8_t Byte=0x3F) // OTG enable, charge enable, 3.7V
   { Error=I2C_Write(Bus, ADDR, REG_POWERON, Byte);
     return Error; }

   uint8_t writeChargeCurr(uint8_t Byte=0x00) // 512mA
   { Error=I2C_Write(Bus, ADDR, REG_CHG_CURR, Byte);
     return Error; }

   uint8_t writePreCharge(uint8_t Byte=0x00) // 128mA, 128mA
   { Error=I2C_Write(Bus, ADDR, REG_PRE_CHG, Byte);
     return Error; }

   uint8_t writeChargeVolt(uint8_t Byte=0x9A) // 4.112V, 3.0V, 100mV
   { Error=I2C_Write(Bus, ADDR, REG_CHG_VOLT, Byte);
     return Error; }

   uint8_t writeTimerCtrl(uint8_t Byte=0x8C)      // disable watchdog
   { Error=I2C_Write(Bus, ADDR, REG_TIMER_CTRL, Byte);
     return Error; }

} ;

#endif //  __BQ24295_H__
