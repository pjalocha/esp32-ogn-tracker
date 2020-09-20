#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "format.h"

static int APRS2IGC(char *Out, const char *Inp, int GeoidSepar)      // convert APRS positon message into IGC B-record
{ int Len=0;
  const char *Msg = strchr(Inp, ':'); if(Msg==0) return 0;           // colon: separates header and message
  Msg++;                                                             // where message starts
  if(Msg[0]!='/' || Msg[7]!='h') return 0;
  const char *Pos = Msg+8; if(Pos[4]!='.' || Pos[14]!='.') return 0; // where position starts
  const char *ExtPos = strstr(Pos+18, " !W"); if(ExtPos[5]=='!') ExtPos+=3; else ExtPos=0;
  Out[Len++]='B';                                                    // B-record
  memcpy(Out+Len, Msg+1, 6); Len+=6;                                 // copy UTC time
  memcpy(Out+Len, Pos, 4); Len+=4;                                   // copy DDMM
  memcpy(Out+Len, Pos+5, 2); Len+=2;                                 // copy fractional MM
  Out[Len++] = ExtPos?ExtPos[0]:'0';                                 // extended precision
  Out[Len++] = Pos[7];                                               // copy N/S sign
  memcpy(Out+Len, Pos+9, 5); Len+=5;                                 // copy DDMM
  memcpy(Out+Len, Pos+15,2); Len+=2;                                 // copy fractional MM
  Out[Len++] = ExtPos?ExtPos[1]:'0';                                 // extended precision
  Out[Len++] = Pos[17];                                              // copy E/W sign
  Out[Len++] = 'A';
  memcpy(Out+Len, "          ", 10);
  const char *FL = strstr(Pos+18, " FL");
  if(FL)                                                             // pressure altitude
  { float PressAlt=atof(FL+3); PressAlt*=30.4; int32_t Alt=floor(PressAlt+0.5);
    if(Alt<0) { Alt = (-Alt); Out[Len] = '-'; Format_UnsDec(Out+Len+1, (uint32_t)Alt, 4); }
         else { Format_UnsDec(Out+Len, (uint32_t)Alt, 5); }
  }
  Len+=5;
  if(Pos[27]=='A' && Pos[28]=='=')                                   // geometrical altitude
  { int32_t Alt=atol(Pos+29); Alt=(Alt*3+5)/10; Alt+=GeoidSepar;     // convert to meters and add GeoidSepar for HAE
    if(Alt<0) { Alt = (-Alt); Out[Len] = '-'; Format_UnsDec(Out+Len+1, (uint32_t)Alt, 4); }
         else { Format_UnsDec(Out+Len, (uint32_t)Alt, 5); }
  }
  Len+=5;
  Out[Len]=0; return Len; }

static int Verbose    =  1;
static int GeoidSepar = 40;

static FILE *OutFile = 0;

int main(int argc, char *argv[])
{ if(argc<3)
  { printf("Usage: %s <own-aircraft-APRS-call> <input-file.aprs>\n", argv[0]);
    return 0; }

  const char *OwnAcft = argv[1]; int OwnAcftLen = strlen(OwnAcft);
  char OutFileName[32]; strcpy(OutFileName, OwnAcft); strcat(OutFileName, ".IGC");
  OutFile=fopen(OutFileName, "wt");
  if(OutFile==0) { printf("Cannot open %s for write\n", OutFileName); return 0; }

  const char *InpFileName = argv[2];

  FILE *InpFile=fopen(InpFileName, "rt");
  if(InpFile==0) { printf("Cannot open %s for read\n", InpFileName); return 0; }

  char InpLine[256];
  char OutLine[256];
  int InpLines=0;
  int OutLines=0;
  for( ; ; )
  { if(fgets(InpLine, 256, InpFile)==0) break;
    char *EOL = strchr(InpLine, '\n'); if(EOL==0) break;
    *EOL = 0;
    InpLines++;
    if(memcmp(InpLine, OwnAcft, OwnAcftLen)) continue;
    int OutLen=APRS2IGC(OutLine, InpLine, GeoidSepar);
    if(OutLen>0)
    { if(Verbose) printf("%s => %s [%d]\n", InpLine, OutLine, OutLen);
      fprintf(OutFile, "%s\n", OutLine); OutLines++; }
  }
  fclose(InpFile);
  printf("%d lines from %s\n", InpLines, InpFileName);
  fclose(OutFile);
  printf("%d lines to %s\n", OutLines, OutFileName);

  return 0; }
