#ifndef  __FORMAT_H__
#define  __FORMAT_H__

#include <stdint.h>

#define WITH_AUTOCR

char HexDigit(uint8_t Val);

       void Format_Bytes ( void (*Output)(char), const uint8_t *Bytes,  uint8_t Len);
inline void Format_Bytes ( void (*Output)(char), const    char *Bytes,  uint8_t Len) { Format_Bytes(Output, (const uint8_t *)Bytes,  Len); }

void Format_String( void (*Output)(char), const    char *String);
void Format_String( void (*Output)(char), const    char *String, uint8_t MinLen, uint8_t MaxLen);

void Format_Hex( void (*Output)(char), uint8_t  Byte );
void Format_Hex( void (*Output)(char), uint16_t Word );
void Format_Hex( void (*Output)(char), uint32_t Word );
void Format_MAC( void (*Output)(char), uint8_t *MAC, uint8_t Len=6);

void Format_UnsDec ( void (*Output)(char), uint16_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0);
void Format_SignDec( void (*Output)(char),  int16_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0, uint8_t NoPlus=0);

void Format_UnsDec ( void (*Output)(char), uint32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0);
void Format_SignDec( void (*Output)(char),  int32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0, uint8_t NoPlus=0);

void Format_UnsDec ( void (*Output)(char), uint64_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0);
void Format_SignDec( void (*Output)(char),  int64_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0, uint8_t NoPlus=0);

uint8_t Format_String(char *Out, const char *String);
uint8_t Format_String(char *Out, const char *String, uint8_t MinLen, uint8_t MaxLen);

uint8_t Format_UnsDec (char *Out, uint32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0);
uint8_t Format_SignDec(char *Out,  int32_t Value, uint8_t MinDigits=1, uint8_t DecPoint=0, uint8_t NoPlus=0);

uint8_t Format_Hex( char *Output, uint8_t  Byte );
uint8_t Format_Hex( char *Output, uint16_t Word );
uint8_t Format_Hex( char *Output, uint32_t Word );
uint8_t Format_Hex( char *Output, uint32_t Word, uint8_t Digits);
uint8_t Format_Hex( char *Output, uint64_t Word );
// uint8_t Format_Hex( char *Output, uint64_t Word, uint8_t Digits);

template <class Type>
 uint8_t Format_Hex( char *Output, Type Word, uint8_t Digits)
{ for(uint8_t Idx=Digits; Idx>0; )
  { Output[--Idx]=HexDigit(Word&0x0F);
    Word>>=4; }
  return Digits; }

uint8_t Format_HHcMMcSS(char *Out, uint32_t Time);
uint8_t Format_HHMMSS(char *Out, uint32_t Time);
void    Format_HHMMSS(void (*Output)(char), uint32_t Time);

uint8_t Format_Latitude (char *Out, int32_t Lat); // [1/600000deg] =>  DDMM.MMMMs
uint8_t Format_Longitude(char *Out, int32_t Lon); // [1/600000deg] => DDDMM.MMMMs

int8_t  Read_Hex1(char Digit);

int8_t  Read_Dec1(char Digit);                  // convert single digit into an integer
inline int8_t Read_Dec1(const char *Inp) { return Read_Dec1(Inp[0]); }
int8_t  Read_Dec2(const char *Inp);             // convert two digit decimal number into an integer
int16_t Read_Dec3(const char *Inp);             // convert three digit decimal number into an integer
int16_t Read_Dec4(const char *Inp);             // convert three digit decimal number into an integer

  template <class Type>
   int8_t Read_Hex(Type &Int, const char *Inp, uint8_t MaxDig=0) // convert variable number of digits hexadecimal number into an integer
   { if(Inp==0) return 0;
     if(MaxDig==0) MaxDig=2*sizeof(Type);
     Int=0; int8_t Len=0;
     for( ; MaxDig; MaxDig--)
     { int8_t Dig=Read_Hex1(Inp[Len]); if(Dig<0) break;
       Int = (Int<<4) + Dig; Len++; }
     return Len; }                                        // return number of characters read

template <class Type>
 int8_t Read_UnsDec(Type &Int, const char *Inp)         // convert variable number of digits unsigned decimal number into an integer
 { Int=0; int8_t Len=0;
   if(Inp==0) return 0;
   for( ; ; )
   { int8_t Dig=Read_Dec1(Inp[Len]); if(Dig<0) break;
     Int = 10*Int + Dig; Len++; }
   return Len; }                                        // return number of characters read

template <class Type>
 int8_t Read_SignDec(Type &Int, const char *Inp)        // convert signed decimal number into in16_t or int32_t
 { Int=0; int8_t Len=0;
   if(Inp==0) return 0;
   char Sign=Inp[0];
   if((Sign=='+')||(Sign=='-')) Len++;
   Len+=Read_UnsDec(Int, Inp+Len); if(Sign=='-') Int=(-Int);
   return Len; }                                        // return number of characters read

template <class Type>
 int8_t Read_Int(Type &Value, const char *Inp)
 { Value=0; int8_t Len=0;
   if(Inp==0) return 0;
   char Sign=Inp[0]; int8_t Dig;
   if((Sign=='+')||(Sign=='-')) Len++;
   if((Inp[Len]=='0')&&(Inp[Len+1]=='x'))
   { Len+=2; Dig=Read_Hex(Value, Inp+Len); }
   else
   { Dig=Read_UnsDec(Value, Inp+Len); }
   if(Dig<=0) return Dig;
   Len+=Dig;
   if(Sign=='-') Value=(-Value); return Len; }

template <class Type>
 int8_t Read_Float1(Type &Value, const char *Inp)       // read floating point, take just one digit after decimal point
 { Value=0; int8_t Len=0;
   if(Inp==0) return 0;
   char Sign=Inp[0]; int8_t Dig;
   if((Sign=='+')||(Sign=='-')) Len++;
   Len+=Read_UnsDec(Value, Inp+Len); Value*=10;
   if(Inp[Len]!='.') goto Ret;
   Len++;
   Dig=Read_Dec1(Inp[Len]); if(Dig<0) goto Ret;
   Value+=Dig; Len++;
   Dig=Read_Dec1(Inp[Len]); if(Dig>=5) Value++;
   Ret: if(Sign=='-') Value=(-Value); return Len; }

int8_t Read_LatDDMMSS(int32_t &Lat, const char *Inp);
int8_t Read_LonDDMMSS(int32_t &Lon, const char *Inp);

#endif //  __FORMAT_H__
