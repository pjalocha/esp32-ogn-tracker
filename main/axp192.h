#ifndef __AXP192_H__
#define __AXP192_H__

#include <stdint.h>

class AXP192
{ private:
   static const uint8_t AXP192_ADDR     = 0x34; // possible I2C addresses

   // registers
   static const uint8_t REG_STATUS          = 0x00; // bit #2 = charge/discharge, bit #0 = power-on triggered by Vbus/ACin
   static const uint8_t REG_MODE_CHGSTATUS  = 0x01; // charge mode and status
   static const uint8_t REG_ID              = 0x03;
   static const uint8_t REG_LDO234_DC23_CTL = 0x12; // [bit mask] control outputs ON/OFF
   static const uint8_t REG_DC1_VOLTAGE     = 0x26; // [25mV] controls DC1 voltage
   static const uint8_t REG_OFF_CTL         = 0x32; // power-off, battery check, LED control
   static const uint8_t REG_CHARGE_1        = 0x33; // target voltage, current
   static const uint8_t REG_CHARGE_2        = 0x34;
   static const uint8_t REG_BACKUP_CHG      = 0x35; // backup battery charge
   static const uint8_t REG_POK_SET         = 0x36; // press-key parameters

   static const uint8_t REG_INTEN1          = 0x40; // IRQ enable
   static const uint8_t REG_INTEN2          = 0x41;
   static const uint8_t REG_INTEN3          = 0x42;
   static const uint8_t REG_INTEN4          = 0x43;
   static const uint8_t REG_INTEN5          = 0x4A;

   static const uint8_t REG_INTSTS1         = 0x44; // IRQ status
   static const uint8_t REG_INTSTS2         = 0x45;
   static const uint8_t REG_INTSTS3         = 0x46;
   static const uint8_t REG_INTSTS4         = 0x47;
   static const uint8_t REG_INTSTS5         = 0x4D;

   static const uint8_t REG_VBUS_VOLT_H8    = 0x5A; // [1.7mV]
   static const uint8_t REG_VBUS_VOLT_L4    = 0x5B;
   static const uint8_t REG_VBUS_CURR_H8    = 0x5C; // [0.375mA]
   static const uint8_t REG_VBUS_CURR_L4    = 0x5D;
   static const uint8_t REG_TEMP_H8         = 0x5E; // [-144.7..264.8/0.1]degC
   static const uint8_t REG_TEMP_L4         = 0x5F;

   static const uint8_t REG_BAT_VOLT_H8     = 0x78; // [0.0..4504.5/1.1]mV
   static const uint8_t REG_BAT_VOLT_L4     = 0x79;
   static const uint8_t REG_BAT_INP_CURR_H8 = 0x7A; // [0.5mA]
   static const uint8_t REG_BAT_INP_CURR_L5 = 0x7B;
   static const uint8_t REG_BAT_OUT_CURR_H8 = 0x7C; // [0.5mA]
   static const uint8_t REG_BAT_OUT_CURR_L5 = 0x7D;

   static const uint8_t REG_ADC_EN1         = 0x82; // battery voltage enabled by default
   static const uint8_t REG_ADC_EN2         = 0x83; // internal temperature enabled by default
   static const uint8_t REG_ADC_SPEED       = 0x84; // 25Hz by default
   static const uint8_t REG_ADC_RANGE       = 0x85;

   static const uint8_t REG_BAT_INP_CHG     = 0xB0;
   static const uint8_t REG_BAT_OUT_CHG     = 0xB4;
   static const uint8_t REG_BAT_MON         = 0xB8;

   static const uint8_t AXP192_ID           = 0x03; // ID for the AXP192

  public:
   static const uint8_t OUT_DCDC1 = 0;              // supply outputs
   static const uint8_t OUT_DCDC3 = 1;
   static const uint8_t OUT_LDO2  = 2;
   static const uint8_t OUT_LDO3  = 3;
   static const uint8_t OUT_DCDC2 = 4;
   static const uint8_t OUT_EXTEN = 6;

   static const uint16_t ADC_TEMP      = 0x8000;    // ADC sampling inputs
   static const uint16_t ADC_BAT_VOLT  = 0x0080;
   static const uint16_t ADC_BAT_CURR  = 0x0040;
   static const uint16_t ADC_VBUS_VOLT = 0x0008;
   static const uint16_t ADC_VBUS_CURR = 0x0004;

   static const uint32_t AXP202_PEK_LONGPRESS_IRQ = 0x00010000;
   static const uint32_t AXP202_PEK_SHORTPRESS_IRQ = 0x00020000;

  public:
   uint8_t Bus;                              // which I2C bus
   uint8_t ADDR;                             // detected I2C address
   uint8_t ID;

  public:
   uint8_t Error;                            // error on the I2C bus (0=no error)

   uint8_t checkID(void) // check ID, to make sure the AXP192 is reachable
   { ADDR=0;
     Error=I2C_Read(Bus, AXP192_ADDR, REG_ID, ID);
     if( (!Error) && (ID==AXP192_ID) ) { ADDR=AXP192_ADDR; return 0; }
     return 1; } // 0 => no error and correct ID

   uint8_t readStatus(void)
   { uint8_t Byte=0; Error=I2C_Read(Bus, ADDR, REG_STATUS, Byte); return Byte; }

   uint8_t setPOK(uint8_t Byte=0xDC)
   { Error=I2C_Write(Bus, ADDR, REG_POK_SET, Byte);
     return Error; }

   uint8_t setPowerOutput(uint8_t Channel, bool ON=1)
   { uint8_t Byte;
     Error=I2C_Read(Bus, ADDR, REG_LDO234_DC23_CTL, Byte); if(Error) return Error;
     uint8_t Mask=1; Mask<<=Channel;
     if(ON) Byte |=  Mask;
       else Byte &= ~Mask;
     Error=I2C_Write(Bus, ADDR, REG_LDO234_DC23_CTL, Byte);
     return Error; }

   uint8_t setDCDC1(uint16_t mV) // [mV] set voltage on DCDC1 output
   { if(mV>3500) mV=3500;
     else if(mV<700) mV=700;
     uint8_t Byte = (mV-700+12)/25;
     Error=I2C_Write(Bus, ADDR, REG_DC1_VOLTAGE, Byte);
     return Error; }

   uint8_t ShutDown(void)
   { uint8_t Byte;
     Error=I2C_Read(Bus, ADDR, REG_OFF_CTL, Byte); if(Error) return Error;
     Byte |= 0x80;
     Error=I2C_Write(Bus, ADDR, REG_OFF_CTL, Byte);
     return Error; }

   uint8_t setLED(uint8_t Mode) // 0=OFF, 1=1Hz, 2=4Hz, 3=ON, 4=reflect charging status
   { uint8_t Byte;
     Error=I2C_Read(Bus, ADDR, REG_OFF_CTL, Byte); if(Error) return Error;
     if(Mode<4)
     { Byte |= 0x08;                         // control LED by software
       Byte &= 0xCF;
       Byte |= Mode<<4; }                    // set LED state: 0, 1, 2, 3
     else Byte &= 0xF7;                      // control LED by hardware
     Error=I2C_Write(Bus, ADDR, REG_OFF_CTL, Byte);
     return Error; }

   uint8_t enableADC(uint16_t Mask)
   { Error=I2C_Write(Bus, ADDR, REG_ADC_EN1, Mask   ); if(Error) return Error;
     Error=I2C_Write(Bus, ADDR, REG_ADC_EN2, Mask>>8); return Error; }

   uint16_t readH8L4(uint8_t Reg)                                            // read two-byte H8:L4 value
   { uint8_t Byte[2]; Error=I2C_Read(Bus, ADDR, Reg, Byte, 2); if(Error) return 0;
     uint16_t Word = Byte[0]; Word<<=4; Word|=Byte[1]&0x0F; return Word; }

   uint16_t readVbusVoltage(void)
   { uint16_t Volt=readH8L4(REG_VBUS_VOLT_H8); return (Volt*17+5)/10; }      // [1mV]

   uint16_t readVbusCurrent(void)
   { uint16_t Curr=readH8L4(REG_VBUS_CURR_H8); return (Curr*15+20)/40; }     // [mA]

   uint16_t readBatteryVoltage(void)
   { uint16_t Volt=readH8L4(REG_BAT_VOLT_H8); return (Volt*11+5)/10; }       // [mV]

   // uint16_t readVbusVoltage(void)
   // { uint8_t H8; Error=I2C_Read(Bus, ADDR, REG_VBUS_VOLT_H8, H8); if(Error) return 0;
   //   uint8_t L4; Error=I2C_Read(Bus, ADDR, REG_VBUS_VOLT_L4, L4); if(Error) return 0;
   //   uint16_t Volt=H8; Volt<<=4; Volt|=L4&0x0F; return (Volt*17+5)/10; }      // [mV]

   // uint16_t readVbusCurrent(void)
   // { uint8_t H8; Error=I2C_Read(Bus, ADDR, REG_VBUS_CURR_H8, H8); if(Error) return 0;
   //   uint8_t L4; Error=I2C_Read(Bus, ADDR, REG_VBUS_CURR_L4, L4); if(Error) return 0;
   //   uint16_t Curr=H8; Curr<<=4; Curr|=L4&0x0F; return (Curr*15+20)/40; }     // [1mA]

   // uint16_t readBatteryVoltage(void)
   // { uint8_t H8; Error=I2C_Read(Bus, ADDR, REG_BAT_VOLT_H8, H8); if(Error) return 0;
   //   uint8_t L4; Error=I2C_Read(Bus, ADDR, REG_BAT_VOLT_L4, L4); if(Error) return 0;
   //   uint16_t Volt=H8; Volt<<=4; Volt|=L4&0x0F; return (Volt*11+5)/10; }      // [1mV]

    int16_t readTemperature(void)
   { int16_t Temp=readH8L4(REG_TEMP_H8); return Temp-1447; }  // [0.1degC]

   //  int16_t readTemperature(void)
   // { uint8_t H8; Error=I2C_Read(Bus, ADDR, REG_TEMP_H8, H8); if(Error) return 0;
   //   uint8_t L4; Error=I2C_Read(Bus, ADDR, REG_TEMP_L4, L4); if(Error) return 0;
   //   int16_t Temp=H8; Temp<<=4; Temp|=L4&0x0F; return Temp-1447; }  // [0.1degC]

   uint16_t readH8L5(uint8_t Reg)
   { uint8_t Byte[2]; Error=I2C_Read(Bus, ADDR, Reg, Byte, 2); if(Error) return 0;
     uint16_t Word = Byte[0]; Word<<=5; Word|=Byte[1]&0x01F; return Word; }

   uint16_t readBatteryInpCurrent(void)
   { uint16_t Curr=readH8L5(REG_BAT_INP_CURR_H8); return Curr/2; }  // [mA]

   uint16_t readBatteryOutCurrent(void)
   { uint16_t Curr=readH8L5(REG_BAT_OUT_CURR_H8); return Curr/2; }  // [mA]

   // uint16_t readBatteryInpCurrent(void)
   // { uint8_t H8; Error=I2C_Read(Bus, ADDR, REG_BAT_INP_CURR_H8, H8); if(Error) return 0;
   //   uint8_t L5; Error=I2C_Read(Bus, ADDR, REG_BAT_INP_CURR_L5, L5); if(Error) return 0;
   //   uint16_t Curr=H8; Curr<<=5; Curr|=L5&0x1F; return Curr/2; }      // [1mA]

   // uint16_t readBatteryOutCurrent(void)
   // { uint8_t H8; Error=I2C_Read(Bus, ADDR, REG_BAT_OUT_CURR_H8, H8); if(Error) return 0;
   //   uint8_t L5; Error=I2C_Read(Bus, ADDR, REG_BAT_OUT_CURR_L5, L5); if(Error) return 0;
   //   uint16_t Curr=H8; Curr<<=5; Curr|=L5&0x1F; return Curr/2; }      // [1mA]

   uint16_t read32bit(uint8_t Reg)
   { uint8_t Byte[4]; Error=I2C_Read(Bus, ADDR, Reg, Byte, 4); if(Error) return 0;
     uint32_t Word=Byte[0];
     for(uint8_t Idx=1; Idx<4; Idx++)
     { Word<<=8; Word|=Byte[Idx]; }
     return Word; }

   uint32_t readBatteryInpCharge(void)                                // [0x8000/25 mAs]
   { uint32_t Charge = read32bit(REG_BAT_INP_CHG); return Charge; }

   uint32_t readBatteryOutCharge(void)                                // [0x8000/25 mAs]
   { uint32_t Charge = read32bit(REG_BAT_OUT_CHG); return Charge; }

   uint8_t setBatMon(uint8_t Byte=0x80)
   { Error=I2C_Write(Bus, ADDR, REG_BAT_MON, Byte); return Error; }

   uint8_t  enableBatMon(void) { return setBatMon(0x80); }
   uint8_t disableBatMon(void) { return setBatMon(0x00); }
   uint8_t    stopBatMon(void) { return setBatMon(0xC0); }
   uint8_t   clearBatMon(void) { return setBatMon(0xA0); }

   bool isBatteryConnected(void)
   { uint8_t Byte;
     Error=I2C_Read(Bus, ADDR, REG_MODE_CHGSTATUS, Byte); if(Error) return 0;
     return Byte&0x20; } // 1=battery connected, 0=no battery connected

   bool isBatteryCharging(void)
   { uint8_t Byte;
     Error=I2C_Read(Bus, ADDR, REG_MODE_CHGSTATUS, Byte); if(Error) return 0;
     return Byte&0x40; } // 1=charging, 0=charge finished or not charging

   bool isBatteryActive(void)
   { uint8_t Byte;
     Error=I2C_Read(Bus, ADDR, REG_MODE_CHGSTATUS, Byte); if(Error) return 0;
     return Byte&0x08; } // 1=active

   bool isChargeNotEnough(void)
   { uint8_t Byte;
     Error=I2C_Read(Bus, ADDR, REG_MODE_CHGSTATUS, Byte); if(Error) return 0;
     return Byte&0x04; } // 1=not enough charging current

   uint8_t enableIRQ(uint32_t IrqMask, bool Enable=1)
   { for(uint8_t Idx=0; Idx<4; Idx++)
     { uint8_t Mask = IrqMask;
       if(Mask)
       { uint8_t Byte;
         Error=I2C_Read(Bus, ADDR, REG_INTEN1+Idx, Byte); // if(Error) continue;
         if(Enable) Byte |=  Mask;
               else Byte &= ~Mask;
         Error=I2C_Write(Bus, ADDR, REG_INTEN1+Idx, Byte); // if(Error) continue;
       }
       IrqMask>>=8;
     }
     return Error; }

   bool readLongPressIRQ(void)
   { uint8_t Byte=0;
     Error=I2C_Read(Bus, ADDR, REG_INTSTS1+2, Byte); if(Error) return 0;
     return Byte&0x01; }

   bool readShortPressIRQ(void)
   { uint8_t Byte=0;
     Error=I2C_Read(Bus, ADDR, REG_INTSTS1+2, Byte); if(Error) return 0;
     return Byte&0x02; }

   uint8_t clearIRQ(void)
   { uint8_t Byte=0xFF;
     for(uint8_t Idx=0; Idx<4; Idx++)
     { Error=I2C_Write(Bus, ADDR, REG_INTSTS1+Idx, Byte); }
     Error=I2C_Write(Bus, ADDR, REG_INTSTS5, Byte);
     return Error; }

} ;

#endif // __AXP192_H__
