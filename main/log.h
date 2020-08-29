#include "fifo.h"
#include "ogn.h"

extern FIFO<OGN_LogPacket<OGN_Packet>, 32> FlashLog_FIFO;

extern uint32_t FlashLog_FileTime;                            // [sec] start time of the current log file
extern char FlashLog_FileName[32];                             // current log file name if open
int  FlashLog_FullFileName(char *FileName, uint32_t Time);    // create full name (including hte path) of the log file corresponding to Time
int  FlashLog_ShortFileName(char *FileName, uint32_t Time);
uint32_t FlashLog_ReadShortFileTime(const char *FileName, int Len);
uint32_t FlashLog_ReadShortFileTime(const char *FileName);
int FlashLog_CopyToSD(bool Remove=0);
int  FlashLog_FindOldestFile(uint32_t &Oldest, uint32_t After=0);        // find the oldest log file
int  FlashLog_ListFiles(void);                                // list the log files on the console
int  FlashLog_ListFile(uint32_t FileTime);                    //
int  FlashLog_ListFile(const char *FileName, uint32_t FileTime); //
// int  FlashLog_ListFile(const char *FileName, int Len);        //

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskLOG(void* pvParameters);

