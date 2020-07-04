#include <stdint.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <utime.h>
#include <unistd.h>

#include "hal.h"                      // Hardware Abstraction Layer

#include "log.h"                      // LOG task: log the own position and other positions heard

#include "gps.h"
#include "ogn.h"                      // OGN packet structures, encoding/decoding/etc.
#include "timesync.h"

// #define DEBUG_PRINT

static void AddPath(char *Name, const char *FileName, const char *Path)
{ if(Path==0) { strcpy(Name, FileName); return; }
  strcpy(Name, Path);
  int Len=strlen(Name); if(Name[Len-1]!='/') Name[Len++]='/';
  strcpy(Name+Len, FileName); }

#ifdef WITH_SPIFFS

static const char *SDcard_Path = "/sdcard/TLG";
static const char *SPIFFSlog_Path = "/spiffs";     // path to log files
static const char *SPIFFSlog_Ext  = ".TLG";        // extension for log files, could be as well .TLA
static const uint32_t SPIFFSlog_MaxTime = 7200;    // 2 hour max. per single log file
static const uint32_t SPIFFSlog_MaxSize = 0x20000; // 128KB max. per single log file
// static uint32_t SPIFFSlogOldestTime;
uint32_t SPIFFSlog_FileTime=0;
char SPIFFSlog_FileName[32];                       // current log file name if open
static FILE *SPIFFSlog_File=0;                     // current log file if open

FIFO<OGN_LogPacket<OGN_Packet>, 32> LOG_FIFO;

int SPIFFSlog_ShortFileName(char *FileName, uint32_t Time)     // make just the short (without path) file name for given start date
{ int Len = Format_Hex(FileName, Time);                        // Time in %08X format
  strcpy(FileName+Len, SPIFFSlog_Ext); Len+=strlen(SPIFFSlog_Ext); // add extension
  FileName[Len]=0; return Len; }                               // return the name length, should be 12 characters

int SPIFFSlog_FullFileName(char *FileName, uint32_t Time)      // make the full (long) log file name for given start time
{ strcpy(FileName, SPIFFSlog_Path);                            // copy the path
  int Len = strlen(FileName);
  if(FileName[Len-1]!='/') FileName[Len++]='/';
  return Len += SPIFFSlog_ShortFileName(FileName+Len, Time); } // return the length of the file name

uint32_t SPIFFSlog_ReadShortFileTime(const char *FileName, int Len) // extract the time from the short file name
{ if(Len!=12) return 0;                                        // file name must be 12 char long
  if(memcmp(FileName+8, SPIFFSlog_Ext, 4)!=0) return 0;        // extension must be .TLx (x is normally G or A)
  uint32_t Time=0;
  if(Read_Hex(Time, FileName)!=8) return 0;                    // read start time, give up if other format
  return Time; }                                               // return the extracted time

uint32_t SPIFFSlog_ReadShortFileTime(const char *FileName)     //
{ return SPIFFSlog_ReadShortFileTime(FileName, strlen(FileName)); }

int SPIFFSlog_CopyToSD(void)                                   // copy log files to SD card
{ int Files=0;
  struct stat DstStat;
  struct utimbuf DstTime;
  char SrcName[32];
  char DstName[32];
  const int BuffSize=2048;
  uint8_t Buffer[BuffSize];
  DIR *Dir=opendir(SPIFFSlog_Path); if(!Dir) return -1;                                 // open directory, give up if not possible
  for( ; ; )
  { vTaskDelay(1);                                                                      // give some time to parallel tasks
    struct dirent *Ent = readdir(Dir); if(!Ent) break;                                  // read next directory entry, break if all read
    if(Ent->d_type != DT_REG) continue;                                                 // skip non-regular files
    char *Name = Ent->d_name;
    uint32_t Time=SPIFFSlog_ReadShortFileTime(Name);                                    // read time from the file name
    if(Time==0) continue;                                                               // skip if not .TLG format
    AddPath(SrcName, Name, SPIFFSlog_Path);
    // strcpy(SrcName, SPIFFSlog_Path); strcat(SrcName, Name);                             // form full source file name
    AddPath(DstName, Name, SDcard_Path);
    // strcpy(DstName, SDcard_Path); strcat(DstName, Name);                                // form full destination file name
    FILE *SrcFile = fopen(SrcName, "rb"); if(SrcFile==0) continue;                      // open source file
    FILE *DstFile = fopen(DstName, "wb");
    { if(mkdir(SDcard_Path, 0777)<0) continue;
      DstFile = fopen(DstName, "wb"); }
    if(DstFile==0) { fclose(SrcFile); continue; }                                       // open destination file
    for( ; ; )
    { int Read=fread(Buffer, 1, BuffSize, SrcFile); if(Read<=0) break;                  // keep copying
      int Write=fwrite(Buffer, 1, Read, DstFile); if(Read<BuffSize || Write<Read) break; } // until EOF
    fclose(SrcFile);                                                                    // close source file
    fclose(DstFile);                                                                    // close destination file
    if(stat(DstName, &DstStat)>=0)                                                      // get file attributes (maybe not really needed)
    { DstTime.actime  = Time;                                                           // set access and modification times of the dest. file
      DstTime.modtime = Time;
      utime(DstName, &DstTime); }                                                       // write to the FAT
    Files++; }
  closedir(Dir);                                                                        // close directory (for searching of log files)
  return Files; }

int SPIFFSlog_FindOldestFile(uint32_t &Oldest, uint32_t After) // find the oldest log file, but not older than given time
{ int Files=0;
  Oldest=0xFFFFFFFF;                                           // possibly oldest time
  DIR *Dir=opendir(SPIFFSlog_Path); if(!Dir) return -1;        // open directory, give up if not possible
  for( ; ; )
  { vTaskDelay(1);
    struct dirent *Ent = readdir(Dir); if(!Ent) break;         // read next directory entry, break if all read
    if(Ent->d_type != DT_REG) continue;                        // skip non-regular files
    char *Name = Ent->d_name;
    uint32_t Time=SPIFFSlog_ReadShortFileTime(Name);           // read time from the file name, skip if other format
    if(Time<=After) continue;                                  // but not older than
    if(Time<Oldest) Oldest=Time;                               // search for oldest start time
    Files++; }
  closedir(Dir);
  return Files; }                                              // return number of log files

int SPIFFSlog_ListFiles(void)                                  // list log files sorted by time
{ int Files=0;
  char Line[64];
  char FullName[32];
  // char HHMMSS[8];
  struct stat Stat;
  uint32_t PrevTime=0;
  for( ; ; )
  { vTaskDelay(1);                                             // not to overload the priority level
    uint32_t Time = 0;
    SPIFFSlog_FindOldestFile(Time, PrevTime);                  // find the next oldest file
    if(Time==0xFFFFFFFF) break;                                // if none found then stop the list
    PrevTime=Time;
    SPIFFSlog_FullFileName(FullName, Time);
    if(stat(FullName, &Stat)<0) continue;                      // get file info
    int Size = Stat.st_size;                                   // if above minimum size: skip
    strcpy(Line, "$POGNL,");
    uint8_t Len=7;
    // strcpy(Line+Len, FullName+strlen(SPIFFSlog_Path)); Len+=12; // print the short name only
    const char *Name = strrchr(FullName, '/'); Name++;
    strcpy(Line+Len, Name); Len+=strlen(Name);                 // print the short name only
    Line[Len++]=',';
    Len+=Format_HHMMSS(Line+Len, Time);                        // print the time-of-day
    Line[Len++]=',';
    Len+=Format_UnsDec(Line+Len, (uint32_t)Size/OGN_LogPacket<OGN_Packet>::Bytes); // number of packets stored
    Len+=NMEA_AppendCheckCRNL(Line, Len);
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, Line, 0, Len);
    xSemaphoreGive(CONS_Mutex);
    Files++; }
  return Files; }

/*
int SPIFFSlog_ListFiles(void)                            //
{ int Files=0;
  char FullName[32]; strcpy(FullName, SPIFFSlogPath); int PathLen=strlen(SPIFFSlogPath);
  char HHMMSS[8];
  struct stat Stat;
  DIR *Dir=opendir(SPIFFSlogPath); if(!Dir) return -1;         // open the file directory
  for( ; ; )                                                   // run through the directory
  { struct dirent *Ent = readdir(Dir); if(!Ent) break;         // read next directory entry, break if all read
    if(Ent->d_type != DT_REG) continue;                        // skip non-regular files
    char *Name = Ent->d_name;
    uint32_t Time = SPIFFSlog_ReadShortFileTime(Name); if(Time==0) continue;
    strcpy(FullName+PathLen, Name);
    if(stat(FullName, &Stat)<0) continue;                      // get file info
    int Size = Stat.st_size;                                   // if above minimum size: skip
    Format_String(CONS_UART_Write, "$POGNL,");
    Format_String(CONS_UART_Write, Name);
    CONS_UART_Write(',');
    Format_HHMMSS(HHMMSS, Time); HHMMSS[6]=0;
    Format_String(CONS_UART_Write, HHMMSS);
    CONS_UART_Write(',');
    Format_UnsDec(CONS_UART_Write, (uint32_t)Size/OGN_LogPacket::Bytes);
    Format_String(CONS_UART_Write, "\n");
    Files++; }
  closedir(Dir);
  return Files; }
*/

int SPIFFSlog_ListFile(const char *FileName, uint32_t FileTime)    // print the content of the given log file
{ char Line[128];
  // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  // Format_String(CONS_UART_Write, "SPIFFSlog_ListFile(");
  // Format_String(CONS_UART_Write, FileName);
  // Format_String(CONS_UART_Write, ")\n");
  // xSemaphoreGive(CONS_Mutex);
  FILE *File=fopen(FileName, "rb"); if(File==0) return -1;
  OGN_LogPacket<OGN_Packet> Packet;
  int Packets=0;
  for( ; ; )
  { if(fread(&Packet, Packet.Bytes, 1, File)!=1) break;
    if(!Packet.isCorrect()) continue;
    if(Packet.Packet.Header.NonPos) continue;
    uint32_t Time = Packet.getTime(FileTime);
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    uint8_t Len=Packet.Packet.WriteAPRS(Line, Time);
    Format_String(CONS_UART_Write, Line, 0, Len);
    xSemaphoreGive(CONS_Mutex);
    vTaskDelay(10);
    Packets++; }
  fclose(File);
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, FileName);
  Format_String(CONS_UART_Write, " => ");
  Format_UnsDec(CONS_UART_Write, (uint32_t)Packets);
  Format_String(CONS_UART_Write, " packets\n");
  xSemaphoreGive(CONS_Mutex);
  return Packets; }

int SPIFFSlog_ListFile(uint32_t FileTime)                   // 
{ char FileName[32];
  SPIFFSlog_FullFileName(FileName, FileTime);
  return SPIFFSlog_ListFile(FileName, FileTime); }

static int SPIFFSlog_CleanEmpty(int MinSize=0)                 // delete empty files or below certain minimum size
{ const int MaxDelFiles = 4;
  uint32_t DelFile[MaxDelFiles];
  int DelFiles=0;
  char FullName[32];
  strcpy(FullName, SPIFFSlog_Path); int PathLen=strlen(FullName);
  if(FullName[PathLen-1]!='/') FullName[PathLen++]='/';
  struct stat Stat;
  DIR *Dir=opendir(SPIFFSlog_Path); if(!Dir) return -1;         // open the file directory
  for( ; ; )                                                   // run through the directory
  { struct dirent *Ent = readdir(Dir); if(!Ent) break;         // read next directory entry, break if all read
    if(Ent->d_type != DT_REG) continue;                        // skip non-regular files
    char *Name = Ent->d_name;
    uint32_t Time = SPIFFSlog_ReadShortFileTime(Name); if(Time==0) continue;
    strcpy(FullName+PathLen, Name);
    if( Time>=0x10000000 && Time<0x80000000 )                    // heuristic data selection
    { if(stat(FullName, &Stat)<0) continue;                      // get file info
      if(Stat.st_size>=MinSize) continue; }                      // if above minimum size: skip
    DelFile[DelFiles++]=Time; if(DelFiles==MaxDelFiles) break;
    vTaskDelay(1); }
  closedir(Dir);
  for( int File=0; File<DelFiles; File++)
  { SPIFFSlog_FullFileName(FullName, DelFile[File]);
    unlink(FullName);
    vTaskDelay(1); }
  return DelFiles; }

static int SPIFFSlog_Clean(size_t MinFree)                          // clean oldest file when running short in space
{ size_t Total, Used;
  if(SPIFFS_Info(Total, Used)!=0) return -1;                        // check SPIFFS status, give up if not possible
  size_t Free = Total-Used;                                         // [B] amount of free space
  if(MinFree) { if(Free>= MinFree ) return 0; }                     // give up if enough space
         else { if(Free>=(Total/2)) return 0; }                     // if MinFree not specified, take Total/4
  uint32_t Oldest=0xFFFFFFFF;
  int Files=SPIFFSlog_FindOldestFile(Oldest, 0);                    // find the oldest file
  if(Files<0) return Files;
  if(Files<=2) return 0;                                            // if two or less files give up
  char FullName[32];
  SPIFFSlog_FullFileName(FullName, Oldest);                         // oldest file name
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "SPIFFSlog_Clean() ");
  Format_String(CONS_UART_Write, FullName);
  CONS_UART_Write(' ');
  Format_UnsDec(CONS_UART_Write, (uint32_t)Files);
  Format_String(CONS_UART_Write, " files\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  if(unlink(FullName)<0) return -1;                                  // remove the oldest file
  return 1; }

static int SPIFFSlog_Clean(size_t MinFree, int Loops)                // repeat the clean procedure several times
{ int Count=0;
  for( ; Loops>0; Loops--)
  { if(SPIFFSlog_Clean(MinFree)<=0) break;
    vTaskDelay(1); Count++; }
  return Count; }

static int SPIFFSlog_Open(uint32_t Time)                             // open a new log file for given start time
{ if(SPIFFSlog_File) { fclose(SPIFFSlog_File); SPIFFSlog_File=0; }   // if a file open already, close it
  SPIFFSlog_CleanEmpty(32);                                          // remove empty files or shorter than 32 bytes
  SPIFFSlog_Clean(2*SPIFFSlog_MaxSize, 8);                           // clean files to get free space at least twice the max. file sie
  SPIFFSlog_FullFileName(SPIFFSlog_FileName, Time);                  // name of the new log file
  SPIFFSlog_FileTime=Time;                                           // record the time of the log file
  SPIFFSlog_File = fopen(SPIFFSlog_FileName, "wb");                  // open the new file
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "SPIFFSlog_Open() ");
  Format_String(CONS_UART_Write, SPIFFSlog_FileName);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  if(SPIFFSlog_File==0) SPIFFSlog_Clean(0, 8);                       // if the file cannot be open clean again
  return SPIFFSlog_File!=0; }                                        // 1=success, 0=failure: new log file could not be open

static int SPIFFSlog(OGN_LogPacket<OGN_Packet> *Packet, int Packets, uint32_t Time)      // log a batch of OGN packets
{ if(SPIFFSlog_File)
  { uint32_t TimeSinceStart = Time-SPIFFSlog_FileTime;                       // [sec] for how long this file is open already ?
    if( (TimeSinceStart>=SPIFFSlog_MaxTime) || (ftell(SPIFFSlog_File)>=SPIFFSlog_MaxSize) ) // is it too long in time or in size ?
    { fclose(SPIFFSlog_File); SPIFFSlog_File=0; }                              // decide to close the current log file
  }
  if(SPIFFSlog_File==0) SPIFFSlog_Open(Time);                                 // if file closed, then attempt to open a new one
  if(SPIFFSlog_File==0) return -1;                                            // if file still not open, then give up
  if(fwrite(Packet, Packet->Bytes, Packets, SPIFFSlog_File)!=Packets)         // write the packet to the log file
  { fclose(SPIFFSlog_File); SPIFFSlog_File=0; SPIFFSlog_Clean(0, 8); return -1; } // if failure then close the log file and report error
  return Packets; }                                                          // report success
#endif // WITH_SPIFFS

static int Copy(void)                                              // copy the packets from the LOG_FIFO to the log file
{ OGN_LogPacket<OGN_Packet> *Packet;
  size_t Packets = LOG_FIFO.getReadBlock(Packet);                  // ask for a block o packets
  if(Packets==0) return 0;                                         // if none: give up
  uint32_t Time = TimeSync_Time();                                 // Time is to create new log file
#ifdef WITH_SPIFFS
  int Err=SPIFFSlog(Packet, Packets, Time);                        // log the batch of packets
  if(Err<0) { SPIFFSlog_Clean(0, 8); Err=SPIFFSlog(Packet, Packets, Time); } // if failed: give it another try
  // if(Err<0) SPIFFSlog_Clean(0, 8);
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "vTaskLOG() ");
  Format_UnsDec(CONS_UART_Write, Packets);
  Format_String(CONS_UART_Write, " packets => ");
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
#endif
  LOG_FIFO.flushReadBlock(Packets);                                // remove the copied packets from the LOG_FIFO
#ifdef WITH_SPIFFS
  return Err;
#else
  return 0;
#endif
}

#ifdef __cplusplus
  extern "C"
#endif
void vTaskLOG(void* pvParameters)
{
  LOG_FIFO.Clear();

#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "TaskLOG() ");
#ifdef WITH_SPIFFS
  { size_t Total, Used;
    if(SPIFFS_Info(Total, Used)==0)                            // get the SPIFFS usage summary
    { Format_UnsDec(CONS_UART_Write, Used/1024);
      Format_String(CONS_UART_Write, "kB used, ");
      Format_UnsDec(CONS_UART_Write, Total/1024);
      Format_String(CONS_UART_Write, "kB total, "); }
  }
#endif
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif

#ifdef WITH_SD
#ifdef WITH_SPIFFS
      SPIFFSlog_CopyToSD();                                   // copy all flash log files to the SD card
#endif
#endif

  TickType_t PrevTick = 0;
  for( ; ; )
  { // vTaskDelay(200);                                           // wait idle 0.2sec
    TickType_t Tick=xTaskGetTickCount();                       //
    size_t Packets = LOG_FIFO.Full();                          // how many packets in the queue ?
// #ifdef DEBUG_PRINT
//     xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
//     Format_String(CONS_UART_Write, "TaskLOG() ");
//     Format_UnsDec(CONS_UART_Write, Tick, 4, 3);
//     Format_String(CONS_UART_Write, "(");
//     Format_UnsDec(CONS_UART_Write, PrevTick, 4, 3);
//     Format_String(CONS_UART_Write, ")s: ");
//     Format_UnsDec(CONS_UART_Write, Packets);
//     Format_String(CONS_UART_Write, " packets\n");
//     xSemaphoreGive(CONS_Mutex);
// #endif
    if(Packets==0) { PrevTick=Tick; vTaskDelay(100); continue; } // if none: then give up
    if(Packets>=8) { Copy(); PrevTick=Tick; continue; }          // if 8 or more packets then copy them to the log file
    TickType_t Diff = Tick-PrevTick;                             // time since last log action
    if(Diff>=8000) { Copy(); PrevTick=Tick; continue; }          // if more than 4.0sec than copy the packets
    vTaskDelay(100); }

}
