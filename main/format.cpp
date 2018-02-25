#include "format.h"

// ------------------------------------------------------------------------------------------

char HexDigit(uint8_t Val) { return Val+(Val<10?'0':'A'-10); }

// ------------------------------------------------------------------------------------------

void Format_Bytes( void (*Output)(char), const uint8_t *Bytes, uint8_t Len)
{ for( ; Len; Len--)
    (*Output)(*Bytes++);
}

void Format_String( void (*Output)(char), const char *String)
{ for( ; ; )
  { uint8_t ch = (*String++); if(ch==0) break;
    if(ch=='\n') (*Output)('\r');
    (*Output)(ch); }
}

uint8_t Format_String(char *Out, const char *String)
{ uint8_t OutLen=0;
  for( ; ; )
  { char ch = (*String++); if(ch==0) break;
    if(ch=='\n') Out[OutLen++]='\r';
    Out[OutLen++]=ch; }
  // Out[OutLen]=0;
  return OutLen; }

void Format_String( void (*Output)(char), const char *String, uint8_t MinLen, uint8_t MaxLen)
{ if(MaxLen<MinLen) MaxLen=MinLen;
  uint8_t Idx;
  for(Idx=0; Idx<MaxLen; Idx++)
  { char ch = String[Idx]; if(ch==0) break;
    if(ch=='\n') (*Output)('\r');
    (*Output)(ch); }
  for(    ; Idx<MinLen; Idx++)
    (*Output)(' ');
}

uint8_t Format_String(char *Out, const char *String, uint8_t MinLen, uint8_t MaxLen)
{ if(MaxLen<MinLen) MaxLen=MinLen;
  uint8_t OutLen=0;
  uint8_t Idx;
  for(Idx=0; Idx<MaxLen; Idx++)
  { char ch = String[Idx]; if(ch==0) break;
    if(ch=='\n') Out[OutLen++]='\r';
    Out[OutLen++]=ch; }
  for(    ; Idx<MinLen; Idx++)
    Out[OutLen++]=' ';
  // Out[OutLen++]=0;
  return OutLen; }

void Format_Hex( void (*Output)(char), uint8_t Byte )
{ (*Output)(HexDigit(Byte>>4)); (*Output)(HexDigit(Byte&0x0F)); }

void Format_Hex( void (*Output)(char), uint16_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>8)); Format_Hex(Output, (uint8_t)Word); }

void Format_Hex( void (*Output)(char), uint32_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>24)); Format_Hex(Output, (uint8_t)(Word>>16));
  Format_Hex(Output, (uint8_t)(Word>>8));  Format_Hex(Output, (uint8_t)Word); }

uint8_t Format_HHMMSS(char *Out, uint32_t Time)
{ uint32_t DayTime=Time%86400;
  uint32_t Hour=DayTime/3600; DayTime-=Hour*3600;
  uint32_t Min=DayTime/60; DayTime-=Min*60;
  uint32_t Sec=DayTime;
  uint32_t HHMMSS = 10000*Hour + 100*Min + Sec;
  return Format_UnsDec(Out, HHMMSS, 6); }

void Format_UnsDec( void (*Output)(char), uint16_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint16_t Base; uint8_t Pos;
  for( Pos=5, Base=10000; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) (*Output)('.');
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Output)('0'+Dig); MinDigits=Pos; }
  }
}

void Format_SignDec( void (*Output)(char), int16_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ if(Value<0) { (*Output)('-'); Value=(-Value); }
         else { (*Output)('+'); }
  Format_UnsDec(Output, (uint16_t)Value, MinDigits, DecPoint); }

void Format_UnsDec( void (*Output)(char), uint32_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint32_t Base; uint8_t Pos;
  for( Pos=10, Base=1000000000; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) (*Output)('.');
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Output)('0'+Dig); MinDigits=Pos; }
  }
}

void Format_SignDec( void (*Output)(char), int32_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ if(Value<0) { (*Output)('-'); Value=(-Value); }
         else { (*Output)('+'); }
  Format_UnsDec(Output, (uint32_t)Value, MinDigits, DecPoint); }

void Format_UnsDec( void (*Output)(char), uint64_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint64_t Base; uint8_t Pos;
  for( Pos=20, Base=10000000000000000000; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) (*Output)('.');
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Output)('0'+Dig); MinDigits=Pos; }
  }
}

void Format_SignDec( void (*Output)(char), int64_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ if(Value<0) { (*Output)('-'); Value=(-Value); }
         else { (*Output)('+'); }
  Format_UnsDec(Output, (uint32_t)Value, MinDigits, DecPoint); }

// ------------------------------------------------------------------------------------------

uint8_t Format_UnsDec(char *Out, uint32_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ uint32_t Base; uint8_t Pos, Len=0;
  for( Pos=10, Base=1000000000; Base; Base/=10, Pos--)
  { uint8_t Dig;
    if(Value>=Base)
    { Dig=Value/Base; Value-=Dig*Base; }
    else
    { Dig=0; }
    if(Pos==DecPoint) { (*Out++)='.'; Len++; }
    if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
    { (*Out++)='0'+Dig; Len++; MinDigits=Pos; }
    // (*Out)=0;
  }
  return Len; }

uint8_t Format_SignDec(char *Out, int32_t Value, uint8_t MinDigits, uint8_t DecPoint)
{ if(Value<0) { (*Out++)='-'; Value=(-Value); }
         else { (*Out++)='+'; }
  return 1+Format_UnsDec(Out, Value, MinDigits, DecPoint); }

uint8_t Format_Hex( char *Output, uint8_t Byte )
{ (*Output++) = HexDigit(Byte>>4); (*Output++)=HexDigit(Byte&0x0F); return 2; }

uint8_t Format_Hex( char *Output, uint16_t Word )
{ Format_Hex(Output, (uint8_t)(Word>>8)); Format_Hex(Output+2, (uint8_t)Word); return 4; }

uint8_t Format_Hex( char *Output, uint32_t Word )
{ Format_Hex(Output  , (uint8_t)(Word>>24)); Format_Hex(Output+2, (uint8_t)(Word>>16));
  Format_Hex(Output+4, (uint8_t)(Word>> 8)); Format_Hex(Output+6, (uint8_t) Word     ); return 8; }

uint8_t Format_Hex( char *Output, uint32_t Word, uint8_t Digits)
{ for(uint8_t Idx=Digits; Idx>0; )
  { Output[--Idx]=HexDigit(Word&0x0F);
    Word>>=4; }
  return Digits; }

// ------------------------------------------------------------------------------------------

int8_t Read_Hex1(char Digit)
{ int8_t Val=Read_Dec1(Digit); if(Val>=0) return Val; 
  if( (Digit>='A') && (Digit<='F') ) return Digit-'A'+10;
  if( (Digit>='a') && (Digit<='f') ) return Digit-'a'+10;
  return -1; }

int8_t Read_Dec1(char Digit)                   // convert single digit into an integer
{ if(Digit<'0') return -1;                     // return -1 if not a decimal digit
  if(Digit>'9') return -1;
  return Digit-'0'; }

int8_t Read_Dec2(const char *Inp)              // convert two digit decimal number into an integer
{ int8_t High=Read_Dec1(Inp[0]); if(High<0) return -1;
  int8_t Low =Read_Dec1(Inp[1]); if(Low<0)  return -1;
  return Low+10*High; }

int16_t Read_Dec3(const char *Inp)             // convert three digit decimal number into an integer
{ int8_t High=Read_Dec1(Inp[0]); if(High<0) return -1;
  int8_t Mid=Read_Dec1(Inp[1]);  if(Mid<0) return -1;
  int8_t Low=Read_Dec1(Inp[2]);  if(Low<0) return -1;
  return (int16_t)Low + (int16_t)10*(int16_t)Mid + (int16_t)100*(int16_t)High; }

int16_t Read_Dec4(const char *Inp)             // convert three digit decimal number into an integer
{ int16_t High=Read_Dec2(Inp  ); if(High<0) return -1;
  int16_t Low =Read_Dec2(Inp+2); if(Low<0) return -1;
  return Low + (int16_t)100*(int16_t)High; }

// ------------------------------------------------------------------------------------------

int8_t Read_Coord(int32_t &Lat, const char *Inp)
{ uint16_t Deg; int8_t Min, Sec;
  Lat=0;
  const char *Start=Inp;
  int8_t Len=Read_UnsDec(Deg, Inp); if(Len<0) return -1;
  Inp+=Len;
  Lat=(uint32_t)Deg*36000;
  if(Inp[0]!=(char)0xC2) return -1;
  if(Inp[1]!=(char)0xB0) return -1;
  Inp+=2;
  Min=Read_Dec2(Inp); if(Min<0) return -1;
  Inp+=2;
  Lat+=(uint32_t)Min*600;
  if(Inp[0]!=(char)'\'') return -1;
  Inp++;
  Sec=Read_Dec2(Inp); if(Sec<0) return -1;
  Inp+=2;
  Lat+=(uint32_t)Sec*10;
  if(Inp[0]=='.')
  { Sec=Read_Dec1(Inp+1); if(Sec<0) return -1;
    Inp+=2; Lat+=Sec; }
  if(Inp[0]==(char)'\"') { Inp++; }
  else if( (Inp[0]==(char)'\'') && (Inp[1]==(char)'\'') ) { Inp+=2; }
  else return -1;
  return Inp-Start; }

int8_t Read_LatDDMMSS(int32_t &Lat, const char *Inp)
{ Lat=0;
  const char *Start=Inp;
  int8_t Sign=0;
       if(Inp[0]=='N') { Sign=  1 ; Inp++; }
  else if(Inp[0]=='S') { Sign=(-1); Inp++; }
  int8_t Len=Read_Coord(Lat, Inp); if(Len<0) return -1;
  Inp+=Len;
  if(Sign==0)
  {      if(Inp[0]=='N') { Sign=  1 ; Inp++; }
    else if(Inp[0]=='S') { Sign=(-1); Inp++; }
  }
  if(Sign==0) return -1;
  if(Sign<0) Lat=(-Lat);
  return Inp-Start; }

int8_t Read_LonDDMMSS(int32_t &Lon, const char *Inp)
{ Lon=0;
  const char *Start=Inp;
  int8_t Sign=0;
       if(Inp[0]=='E') { Sign=  1 ; Inp++; }
  else if(Inp[0]=='W') { Sign=(-1); Inp++; }
  int8_t Len=Read_Coord(Lon, Inp); if(Len<0) return -1;
  Inp+=Len;
  if(Sign==0)
  {      if(Inp[0]=='E') { Sign=  1 ; Inp++; }
    else if(Inp[0]=='W') { Sign=(-1); Inp++; }
  }
  if(Sign==0) return -1;
  if(Sign<0) Lon=(-Lon);
  return Inp-Start; }

