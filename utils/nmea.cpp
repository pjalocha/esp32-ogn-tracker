#include "nmea.h"

uint8_t NMEA_Check(uint8_t *NMEA, uint8_t Len) // NMEA check-sum
{ uint8_t Check=0;                             // to be calculated over characters between '$' and '*' but _excluding_ those.
  for(uint8_t Idx=0; Idx<Len; Idx++)
    Check^=NMEA[Idx];
  return Check; }

uint8_t NMEA_AppendCheck(uint8_t *NMEA, uint8_t Len)
{ uint8_t Check=NMEA_Check(NMEA+1, Len-1);
  NMEA[Len  ]='*';
  uint8_t Digit=Check>>4;  NMEA[Len+1] = Digit<10 ? Digit+'0':Digit+'A'-10;
          Digit=Check&0xF; NMEA[Len+2] = Digit<10 ? Digit+'0':Digit+'A'-10;
  return 3; }

uint8_t NMEA_AppendCheckCRNL(uint8_t *NMEA, uint8_t Len)
{ uint8_t CheckLen=NMEA_AppendCheck(NMEA, Len);
  Len+=CheckLen; NMEA[Len]='\n';
  return CheckLen+1; }
