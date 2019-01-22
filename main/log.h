#include "fifo.h"
#include "ogn.h"

extern FIFO<OGN_LogPacket<OGN_Packet>, 32> LOG_FIFO;

extern uint32_t SPIFFSlog_FileTime;                            // [sec] start time of the current log file
extern char SPIFFSlogFileName[32];                             // current log file name if open
int  SPIFFSlog_FullFileName(char *FileName, uint32_t Time);    // create full name (including hte path) of the log file corresponding to Time
int  SPIFFSlog_ShortFileName(char *FileName, uint32_t Time);
uint32_t SPIFFSlog_ReadShortFileTime(const char *FileName, int Len);
uint32_t SPIFFSlog_ReadShortFileTime(const char *FileName);
int  SPIFFSlog_FindOldestFile(uint32_t &Oldest, uint32_t After=0);        // find the oldest log file
int  SPIFFSlog_ListFiles(void);                                // list the log files on the console
int  SPIFFSlog_ListFile(uint32_t FileTime);                    //
int  SPIFFSlog_ListFile(const char *FileName, uint32_t FileTime); //
// int  SPIFFSlog_ListFile(const char *FileName, int Len);        //

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskLOG(void* pvParameters);

