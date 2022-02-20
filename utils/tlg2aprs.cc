#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ogn1.h"
#include "ogn.h"

static char Line[160];

static int ProcessFile(const char *FileName)
{ uint32_t FileTime=0;
  const char *ShortName=FileName;
  for( ; ; )
  { const char *Slash=strchr(ShortName, '/'); if(Slash==0) break;
    ShortName=Slash+1; }
  if(Read_Hex(FileTime, ShortName)!=8) printf("Not a TLG file: %s\n", ShortName);
  FILE *File = fopen(FileName, "rb"); if(File==0) { printf("Cannot open %s for read\n", FileName); return 0; }
  OGN_LogPacket<OGN1_Packet> Packet;
  int Packets=0;
  for( ; ; )
  { if(fread(&Packet, Packet.Bytes, 1, File)!=1) break;          // read the next packet from the file
    if(!Packet.isCorrect()) continue;                            //
    uint32_t Time=Packet.getTime(FileTime);                      // [sec] get exact time from short time in the packet and the file start time
    int Len=Packet.Packet.WriteAPRS(Line, Time);
    if(Len==0) continue;
    printf("%s\n", Line); }
  fclose(File); return Packets; }

int main(int argc, char *argv[])
{

  for(int arg=1; arg<argc; arg++)
  { ProcessFile(argv[arg]); }

  return 0; }

