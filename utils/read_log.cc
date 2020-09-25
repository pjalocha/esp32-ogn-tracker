#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <math.h>

#include <vector>

#include "serial.h"
#include "ogn.h"

static double getTime(void)                                           // read the system time at this very moment
{ struct timespec now; clock_gettime(CLOCK_REALTIME, &now); return now.tv_sec + 1e-9*now.tv_nsec; }

static const char *PortName = "/dev/ttyUSB0";                              // default serial port name
static int         BaudRate = 115200;                                      // default baud rate on the serial port
static int         ListOnly = 0;
static uint32_t    SelectFile  = 0;
static int         Help = 0;

static SerialPort Port;

const int MaxLineLen = 512;
static char Line[MaxLineLen];

static std::vector<uint32_t> LogFileList;                                  // list of log file start times

static int List(void)
{
  sleep(1);
  Port.Write(12);                                                     // send Ctrl-L to list the log files

  uint8_t Index[32];
  int LineIdx=0;
  double Start=getTime();                                             // sart counting the time
  for( ; ; )
  { char Byte;
    if(Port.Read(Byte)<=0)                                            // get a byte from the serial port
    { double Now=getTime();                                           // if non, then check time
      if((Now-Start)>=5.0) break;                                     // if idle for more than 4 sec then stop recording the log files
      usleep(1000); continue; }                                       // if no new bytes on the serial port sleep a little
    // printf("%3d: %02X %c\n", LineIdx, Byte, Byte<=' '?' ':Byte);
    if(Byte<' ')                                                      // if a control (non-printable) character
    { Line[LineIdx]=0;                                                // terminate the line and process it
      if(LineIdx<7) { LineIdx=0; continue; }
      // printf("%s\n", Line);
      if(memcmp(Line, "$POGNL,", 7)!=0) { LineIdx=0; continue; }      // only take POGNL sentences
      int8_t Err=GPS_Position::IndexNMEA(Index, Line);                // index the parameters
      if(Err<2) { LineIdx=0; continue; }                              // if less than two parameters, then skip this message
      if((Index[1]-Index[0])!=13) { LineIdx=0; continue; }            // filename must be 13 characters long
      uint32_t UnixTime=0;
      if(Read_Hex(UnixTime, Line+Index[0])!=8) { LineIdx=0; continue; }   // get the starting time: 8 hex characters
      time_t Time = UnixTime;
      struct tm *TM = gmtime(&Time);
      printf("%s (%d,%d) %02d:%02d:%02d %02d.%02d.%04d\n",
             Line, Err, Index[1]-Index[0], TM->tm_hour, TM->tm_min, TM->tm_sec, TM->tm_mday, TM->tm_mon+1, TM->tm_year+1900);
      Start=getTime();                                                // set new start time counter
      LogFileList.push_back(Time);
      LineIdx=0; continue; }
    if(LineIdx<MaxLineLen) Line[LineIdx++]=Byte;                      // add the byte to the line, keep collecting more bytes
  }
  printf("%d log files\n", LogFileList.size());

  return LogFileList.size(); }

static int Download(uint32_t LogFile)
{ char Cmd[32];

  sleep(1);
  sprintf(Cmd, "$POGNL,%08X.TLG\n", LogFile);
  Port.Write(Cmd);
  printf("Port <- %s", Cmd);

  int Records=0;
  int LineIdx=0;
  double Start=getTime();
  for( ; ; )
  { char Byte;
    if(Port.Read(Byte)<=0)
    { double Now=getTime();
      if((Now-Start)>=4.0) break;
      usleep(1000); continue; }                                       // if no new bytes on the serial port sleep a little
    if(Byte<' ')                                                      // if a control (non-printable) character
    { Line[LineIdx]=0;
      if(Line[0]=='$') { LineIdx=0; continue; }
      if(LineIdx<12) { LineIdx=0; continue; }
      printf("%s\n", Line);
      Start=getTime();
      Records++;
      LineIdx=0; continue; }
    if(LineIdx<MaxLineLen) Line[LineIdx++]=Byte;
  }

  printf("%d log records for %08X.TLG\n", Records, LogFile);
  return Records; }

int main(int argc, char *argv[])
{

  int arg=1;
  for( ; arg<argc; arg++)
  { const char *Val = argv[arg]; if(Val[0]!='-') break;
    switch(Val[1])
    { case 'h': Help=1; break;
      case 'l': ListOnly=1; break;
      case 'f': SelectFile=strtol(Val+2, 0, 16); break;
      default: Help=1; break;
    }
  }

  if(Help)
  { printf("Usage: %s [options] <serial-port:baudrate>\n\
Options: -h                this help\n\
         -l                only list the log files stored in the OGN-Tracker\n\
         -f<UTC-hex-time>  download the selected file: use the UTC time\n\
", argv[0]);
    return 0; }

  if(arg<argc)
  { char *Colon = strchr(argv[arg],':');
    if(Colon) { Colon[0]=0; BaudRate=atoi(Colon+1); }
    PortName = argv[arg]; }

  if(Port.Open(PortName, BaudRate)<0) { printf("Can't open %s at %dbps\n", PortName, BaudRate); return -1; }
  // printf("Open %s at %dbps\n", PortName, BaudRate);

  if(SelectFile==0) List();

  if(ListOnly) { Port.Close(); return 0; }

  if(SelectFile)
  { Download(SelectFile); }
  else
  { for( size_t File=0; File<LogFileList.size(); File++)
      Download(LogFileList[File]); }

  Port.Close();
  return 0; }

