#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <math.h>

#include "serial.h"
#include "ogn.h"

static double getTime(void)                                           // read the system time at this very moment
{ struct timespec now; clock_gettime(CLOCK_REALTIME, &now); return now.tv_sec + 1e-9*now.tv_nsec; }

SerialPort Port;

int main(int argc, char *argv[])
{ const int MaxLineLen = 256;
  char Line[MaxLineLen];
  int LineIdx;

  const char *PortName = "/dev/ttyUSB0";                              // default serial port name
        int   BaudRate = 115200;                                      // default baud rate on the serial port

  if(argc>=2)
  { char *Colon = strchr(argv[1],':');
    if(Colon) { Colon[0]=0; BaudRate=atoi(Colon+1); }
    PortName = argv[1]; }

  if(Port.Open(PortName, BaudRate)<0) { printf("Can't open %s at %dbps\n", PortName, BaudRate); return -1; }

  uint8_t Index[20];
  LineIdx=0;                                                          // line byte index/counter
  double Start=getTime();                                             // line start time
  for( ; ; )
  { char Byte;
    if(Port.Read(Byte)<=0) { usleep(1000); continue; }                // if no new bytes on the serial port sleep a little
    if(LineIdx==0) Start=getTime();                                   // if this is the first byte on the line: note the time
    if(Byte<' ')                                                      // if a control (non-printable) character
    { if(LineIdx==0) { printf("<%02X>", Byte); continue; }                                    // if this is the first byte on the line
      printf("\n");
      Line[LineIdx]=0;                                                // if more bytes on this line: add NL at the end of the line
      int8_t Err=GPS_Position::IndexNMEA(Index, Line);
      uint32_t Time=(uint32_t)floor(Start);
      double Frac=Start-Time;
      printf("%06.3f [%3d] [%2d] %s<%02X>", (Time%60)+Frac, LineIdx, Err, Line, Byte);     // print the time, line length and the line itself
      LineIdx=0;                                                      // reset line byte index/counter
      continue; }
    if(LineIdx<MaxLineLen) Line[LineIdx++]=Byte;
  }

  Port.Close();
  return 0; }
