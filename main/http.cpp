#include <ctype.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>

#include "hal.h"

// #include "tcpip_adapter.h"
// #include "esp_wifi.h"
// #include "esp_event_loop.h"
#include "esp_http_server.h"

#include "format.h"
// #include "fifo.h"
// #include "socket.h"
#include "proc.h"
// #include "wifi.h"
#include "log.h"
#include "http.h"

// #define DEBUG_PRINT

#ifdef WITH_HTTP

// ============================================================================================================

static void SelectList(httpd_req_t *Req, const char *Name, const char **List, int Size, int Sel=0)
{ char Line[8];
  httpd_resp_sendstr_chunk(Req, "<select name=\"");
  httpd_resp_sendstr_chunk(Req, Name);
  httpd_resp_sendstr_chunk(Req, "\">\n");
  for(int Idx=0; Idx<Size; Idx++)
  { httpd_resp_sendstr_chunk(Req, "<option value=\"");
    int Len=Format_UnsDec(Line, (uint16_t)Idx);
    httpd_resp_send_chunk(Req, Line, Len);
    httpd_resp_sendstr_chunk(Req, "\"");
    if(Idx==Sel) httpd_resp_sendstr_chunk(Req, " selected>");
            else httpd_resp_sendstr_chunk(Req, ">");
    httpd_resp_sendstr_chunk(Req, List[Idx]);
    httpd_resp_sendstr_chunk(Req, "</option>\n");
  }
  httpd_resp_sendstr_chunk(Req, "</select>\n"); }

static void ParmForm_Info(httpd_req_t *Req)  // produce HTML form for aircraft parameters
{
  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"Info\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Info</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Pilot</td><td><input type=\"text\" name=\"Pilot\" size=\"10\" value=\"");
  if(Parameters.Pilot[0]) httpd_resp_sendstr_chunk(Req, Parameters.Pilot);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Crew</td><td><input type=\"text\" name=\"Crew\" size=\"10\" value=\"");
  if(Parameters.Crew[0]) httpd_resp_sendstr_chunk(Req, Parameters.Crew);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Base airfield</td><td><input type=\"text\" name=\"Base\" size=\"10\" value=\"");
  if(Parameters.Base[0]) httpd_resp_sendstr_chunk(Req, Parameters.Base);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Registration</td><td><input type=\"text\" name=\"Reg\" size=\"10\" value=\"");
  if(Parameters.Reg[0]) httpd_resp_sendstr_chunk(Req, Parameters.Reg);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Manufacturer</td><td><input type=\"text\" name=\"Manuf\" size=\"10\" value=\"");
  if(Parameters.Manuf[0]) httpd_resp_sendstr_chunk(Req, Parameters.Manuf);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Model</td><td><input type=\"text\" name=\"Model\" size=\"10\" value=\"");
  if(Parameters.Model[0]) httpd_resp_sendstr_chunk(Req, Parameters.Model);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Type</td><td><input type=\"text\" name=\"Type\" size=\"10\" value=\"");
  if(Parameters.Type[0]) httpd_resp_sendstr_chunk(Req, Parameters.Type);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }

static void ParmForm_Acft(httpd_req_t *Req)  // produce HTML form for aircraft parameters
{ char Line[16];

  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"Acft\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Aircraft</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Address</td><td><input type=\"text\" name=\"Address\" size=\"10\" value=\"0x");
  Format_Hex(Line, (uint8_t)(Parameters.Address>>16)); Format_Hex(Line+2, (uint16_t)Parameters.Address);
  httpd_resp_send_chunk(Req, Line, 6);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  const char *AddrTypeTable[4] = { "Random", "ICAO", "FLARM", "OGN" } ;
  httpd_resp_sendstr_chunk(Req, "<tr><td>Addr-Type</td><td>\n");
  SelectList(Req, "AddrType", AddrTypeTable, 4, Parameters.AddrType);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n");

  const char *AcftTypeTable[16] = { "Unknown", "(moto)Glider", "Tow-plane", "Helicopter", "Parachute", "Drop-plane", "Hang-glider", "Para-glider",
                                    "Powered-aircraft", "Jet-aircraft", "UFO", "Balloon", "Airship", "UAV/drone", "Ground support", "Static object" } ;
  httpd_resp_sendstr_chunk(Req, "<tr><td>Acft-Type</td><td>\n");
  SelectList(Req, "AcftType", AcftTypeTable, 16, Parameters.AcftType);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }

static void ParmForm_Other(httpd_req_t *Req)  // produce HTML form for aircraft parameters
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"Other\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Other</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");

  const char *FreqPlanTable[16] = { "Auto", "Europe/Africa", "USA/Canada", "Australia/Chile", "New Zeeland", "Izrael" };
  httpd_resp_sendstr_chunk(Req, "<tr><td>Freq. plan</td><td>\n");
  SelectList(Req, "FreqPlan", FreqPlanTable, 6, Parameters.FreqPlan);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Tx power [dBm]</td><td><input type=\"text\" name=\"TxPower\" size=\"10\" value=\"");
  Len=Format_SignDec(Line, (int16_t)Parameters.getTxPower());
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Freq.corr. [ppm]</td><td><input type=\"text\" name=\"RFchipFreqCorr\" size=\"10\" value=\"");
  Len=Format_SignDec(Line, Parameters.RFchipFreqCorr, 2, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Console baud</td><td><input type=\"text\" name=\"CONbaud\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.CONbaud);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Verbose</td><td><input type=\"text\" name=\"Verbose\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, (uint16_t)Parameters.Verbose);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Page sel. mask</td><td><input type=\"text\" name=\"PageMask\" size=\"10\" value=\"0x");
  Len=Format_Hex(Line, Parameters.PageMask);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }

#ifdef WITH_AP
static void ParmForm_AP(httpd_req_t *Req) // Wi-Fi access point parameters { char Line[16]; int Len;
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"AP\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Wi-Fi</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>SSID</td><td><input type=\"text\" name=\"APname\" size=\"10\" value=\"");
  if(Parameters.APname[0]) httpd_resp_sendstr_chunk(Req, Parameters.APname);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Password</td><td><input type=\"text\" name=\"APpass\" size=\"10\" value=\"");
  if(Parameters.APpass[0]) httpd_resp_sendstr_chunk(Req, Parameters.APpass);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Data port</td><td><input type=\"text\" name=\"APport\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.APport);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "<tr><td>Tx [dBm]</td><td><input type=\"text\" name=\"APtxPwr\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, (10*Parameters.APtxPwr+2)>>2, 2, 1);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }
#endif

static esp_err_t parm_get_handler(httpd_req_t *Req)
{ char Line[32]; int Len;
  uint16_t URLlen=httpd_req_get_url_query_len(Req);
  if(URLlen)
  { char *URL = (char *)malloc(URLlen+1);
    httpd_req_get_url_query_str(Req, URL, URLlen+1);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "parm_get_handler() => [");
    Format_SignDec(CONS_UART_Write, URLlen);
    Format_String(CONS_UART_Write, "] ");
    Format_String(CONS_UART_Write, URL);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    char *Line=URL;
    for( ; ; )
    { Parameters.ReadLine(Line);
      Line = strchr(Line, '&'); if(Line==0) break;
      Line++; }
    free(URL);
    Parameters.WriteToNVS(); }
  httpd_resp_sendstr_chunk(Req, "\
<!DOCTYPE html>\n\
<html>\n\
<head>\n\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n\
</head>\n\
<body>\n\
<title>OGN-Tracker configuration</title>\n\
");

  httpd_resp_sendstr_chunk(Req, "<h2>OGN-Tracker configuration</h2>\n");
  httpd_resp_sendstr_chunk(Req, "<b>CPU ID: ");
  Len=Format_Hex(Line, getUniqueID());
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "</b><br />\n");

  httpd_resp_sendstr_chunk(Req, "<table>\n<tr><td>\n");
  ParmForm_Acft(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
  ParmForm_Info(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
#ifdef WITH_AP
  ParmForm_AP(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
#endif
  ParmForm_Other(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\">\n");
  httpd_resp_sendstr_chunk(Req, "<input type=\"submit\" value=\"Reset to defaults\">\n");
  httpd_resp_sendstr_chunk(Req, "<input type=\"hidden\" name=\"Defaults\" value=\"1\">\n");
  httpd_resp_sendstr_chunk(Req, "</form>\n");
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n</table>\n");
  httpd_resp_sendstr_chunk(Req, "</body>\n</html>\n");

/*
  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\">\n");
  httpd_resp_sendstr_chunk(Req, "<input type=\"submit\" value=\"Restart\">\n");
  httpd_resp_sendstr_chunk(Req, "<input type=\"hidden\" name=\"Restart\" value=\"1\">");
  httpd_resp_sendstr_chunk(Req, "</form>\n");
*/
  // httpd_resp_sendstr_chunk(Req, 0);
  httpd_resp_send_chunk(Req, 0, 0);
  return ESP_OK; }

static esp_err_t top_get_handler(httpd_req_t *Req)
{
  httpd_resp_set_status(Req, "307 Temporary Redirect");
  httpd_resp_set_hdr(Req, "Location", "/parm.html");
  httpd_resp_send(Req, 0, 0);
/*
  httpd_resp_sendstr_chunk(Req, "\
<!DOCTYPE html>\n\
<html><body>\n\
<title>OGN-Tracker status</title>\n\
");

  httpd_resp_sendstr_chunk(Req, "<h1>OGN-Tracker</h1>\n");
  httpd_resp_sendstr_chunk(Req, "<a href=\"parm.html\">Configuration page</a>\n");

  httpd_resp_sendstr_chunk(Req, "</body></html>\n");
  httpd_resp_sendstr_chunk(Req, 0);
*/
  return ESP_OK; }

static int Format_DateTime(char *Out, time_t Time)
{ struct tm *TM = gmtime(&Time);
  int Len=Format_UnsDec(Out, (uint16_t)1900+TM->tm_year, 4);
  Out[Len++]='.';
  Len+=Format_UnsDec(Out+Len, (uint16_t)1+TM->tm_mon, 2);
  Out[Len++]='.';
  Len+=Format_UnsDec(Out+Len, (uint16_t)TM->tm_mday, 2);
  Out[Len++]=' ';
  Len+=Format_UnsDec(Out+Len, (uint16_t)TM->tm_hour, 2);
  Out[Len++]=':';
  Len+=Format_UnsDec(Out+Len, (uint16_t)TM->tm_min, 2);
  Out[Len++]=':';
  Len+=Format_UnsDec(Out+Len, (uint16_t)TM->tm_sec, 2);
  return Len; }

static int LogFileName(char *Name, time_t Time, const char *Ext=0)
{ int Len=Parameters.getAprsCall(Name);
  Name[Len++]='_';
  struct tm *TM = gmtime(&Time);
  Len+=Format_UnsDec(Name+Len, (uint16_t)1900+TM->tm_year, 4);
  Name[Len++]='.';
  Len+=Format_UnsDec(Name+Len, (uint16_t)1+TM->tm_mon, 2);
  Name[Len++]='.';
  Len+=Format_UnsDec(Name+Len, (uint16_t)TM->tm_mday, 2);
  Name[Len++]='_';
  Len+=Format_UnsDec(Name+Len, (uint16_t)TM->tm_hour, 2);
  Len+=Format_UnsDec(Name+Len, (uint16_t)TM->tm_min, 2);
  Len+=Format_UnsDec(Name+Len, (uint16_t)TM->tm_sec, 2);
  if(Ext) Len+=Format_String(Name+Len, Ext);
  Name[Len]=0; return Len; }

static esp_err_t SendLog_APRS(httpd_req_t *Req, const char *FileName, uint32_t FileTime)
{ char ContDisp[64];
  char Line[128]; int Len;
  Len=Format_String(ContDisp, "attachement; filename=\""); Len+=LogFileName(ContDisp+Len, FileTime, ".aprs"); ContDisp[Len++]='\"'; ContDisp[Len]=0;
  httpd_resp_set_hdr(Req, "Content-Disposition", ContDisp);
  httpd_resp_set_type(Req, "text/plain");
  FILE *File = fopen(FileName, "rb"); if(File==0) { httpd_resp_send_chunk(Req, 0, 0); return ESP_OK; }
  OGN_LogPacket<OGN_Packet> Packet;
  for( ; ; )
  { if(fread(&Packet, Packet.Bytes, 1, File)!=1) break;          // read the next packet
    if(!Packet.isCorrect()) continue;
    uint32_t Time = Packet.getTime(FileTime);                    // [sec] get exact time from short time in the packet and the file start time
    Len=Packet.Packet.WriteAPRS(Line, Time);                     // print the packet in the APRS format
    if(Len==0) continue;                                         // if cannot be printed for whatever reason
    httpd_resp_send_chunk(Req, Line, Len);
    vTaskDelay(1); }
  fclose(File);
  httpd_resp_send_chunk(Req, 0, 0);
  return ESP_OK; }

static esp_err_t SendLog_TLG(httpd_req_t *Req, const char *FileName, uint32_t FileTime)
{ char ContDisp[64];
  char Line[512]; int Len;
  Len=Format_String(ContDisp, "attachement; filename=\"");
  Len+=Parameters.getAprsCall(ContDisp+Len); ContDisp[Len++]='_';
  Len+=Format_Hex(ContDisp+Len, FileTime);
  Len+=Format_String(ContDisp+Len, ".TLG"); ContDisp[Len++]='\"'; ContDisp[Len]=0;
  httpd_resp_set_hdr(Req, "Content-Disposition", ContDisp);
  httpd_resp_set_type(Req, "application/octet-stream");
  FILE *File = fopen(FileName, "rb"); if(File==0) { httpd_resp_send_chunk(Req, 0, 0); return ESP_OK; }
  for( ; ; )
  { Len=fread(Line, 1, 512, File); if(Len<=0) break;
    httpd_resp_send_chunk(Req, Line, Len);
    vTaskDelay(1); }
  fclose(File);
  httpd_resp_send_chunk(Req, 0, 0);
  return ESP_OK; }

static esp_err_t log_get_handler(httpd_req_t *Req)
{ char FullName[32]; char Line[32]; struct stat Stat;
  // httpd_resp_set_status(Req, "307 Temporary Redirect");
  // httpd_resp_set_hdr(Req, "Location", "/parm.html");
  const char *Path= "/spiffs";

  uint16_t URLlen=httpd_req_get_url_query_len(Req);
  if(URLlen)
  { char *URL = (char *)malloc(URLlen+1);
    httpd_req_get_url_query_str(Req, URL, URLlen+1);
    char Name[16]; bool SendFile = httpd_query_key_value(URL, "File", Name, 16)==ESP_OK;
    char Format[8] = { 0 }; httpd_query_key_value(URL, "Format", Format, 8);
    free(URL);
    if(SendFile)
    { AddPath(FullName, Name, Path);
      uint32_t Time=FlashLog_ReadShortFileTime(Name);
      if(Time)
      { if(strcmp(Format, "APRS")==0) return SendLog_APRS(Req, FullName, Time);
        return SendLog_TLG(Req, FullName, Time); }
    }
  }
  httpd_resp_sendstr_chunk(Req, "\
<!DOCTYPE html>\n\
<html>\n\
<head>\n\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n\
</head>\n\
<body>\n\
<title>OGN-Tracker log files</title>\n\
");

  httpd_resp_sendstr_chunk(Req, "<h2>OGN-Tracker log files</h2>\n");

  DIR *Dir=opendir(Path);
  if(Dir)
  { httpd_resp_sendstr_chunk(Req, "<table>\n<tr><th>File</th><th></th><th>[KB]</th><th>Date</th></tr>\n");
    for( ; ; )
    { struct dirent *Ent = readdir(Dir); if(!Ent) break;        // read next directory entry, break if all read
      if(Ent->d_type != DT_REG) continue;                       // skip non-regular files
      char *Name = Ent->d_name;
      uint32_t Time=FlashLog_ReadShortFileTime(Name);           // read time from the file name
      if(Time==0) continue;                                     // skip if not .TLG format
      AddPath(FullName, Name, Path);
      uint32_t Size=0;
      if(stat(FullName, &Stat)>=0)                              // get file info
        Size = Stat.st_size;
      httpd_resp_sendstr_chunk(Req, "<tr><td><a href=\"/log.html?File=");
      httpd_resp_sendstr_chunk(Req, Name);
      httpd_resp_sendstr_chunk(Req, "\">");
      httpd_resp_sendstr_chunk(Req, Name);
      httpd_resp_sendstr_chunk(Req, "</a></td><td><a href=\"/log.html?Format=APRS&File=");
      httpd_resp_sendstr_chunk(Req, Name);
      httpd_resp_sendstr_chunk(Req, "\">A</a></td><td align=\"center\">");
      int Len=Format_UnsDec(Line, (Size+512)>>10);
      httpd_resp_send_chunk(Req, Line, Len);
      httpd_resp_sendstr_chunk(Req, "</td><td>");
      Len=Format_DateTime(Line, Time);
      httpd_resp_send_chunk(Req, Line, Len);
      httpd_resp_sendstr_chunk(Req, "</td></tr>\n");
      vTaskDelay(1); }
    httpd_resp_sendstr_chunk(Req, "</table>\n");
    closedir(Dir);
  } else httpd_resp_sendstr_chunk(Req, "<p>Cannot open the log directory !</p>\n");
  httpd_resp_sendstr_chunk(Req, "</body>\n</html>\n");
  httpd_resp_send_chunk(Req, 0, 0);
  return ESP_OK; }

static esp_err_t logo_get_handler(httpd_req_t *Req)
{ extern const uint8_t OGN_logo_jpg[]   asm("_binary_OGN_logo_240x240_jpg_start");
  extern const uint8_t OGN_logo_end[]   asm("_binary_OGN_logo_240x240_jpg_end");
  const int OGN_logo_size = OGN_logo_end-OGN_logo_jpg;
  httpd_resp_set_type(Req, "image/jpeg");
  httpd_resp_send(Req, (const char *)OGN_logo_jpg, OGN_logo_size);
  return ESP_OK; }

static const httpd_uri_t HTTPtop =
{ .uri       = "/",
  .method    = HTTP_GET,
  .handler   = top_get_handler,
  .user_ctx  = 0 };

static const httpd_uri_t HTTPlogo =
{ .uri       = "/logo.jpeg",
  .method    = HTTP_GET,
  .handler   = logo_get_handler,
  .user_ctx  = 0 };

static const httpd_uri_t HTTPparm =
{ .uri       = "/parm.html",
  .method    = HTTP_GET,
  .handler   = parm_get_handler,
  .user_ctx  = 0 };

static const httpd_uri_t HTTPlog =
{ .uri       = "/log.html",
  .method    = HTTP_GET,
  .handler   = log_get_handler,
  .user_ctx  = 0 };

static httpd_handle_t HTTPserver = 0;

esp_err_t HTTP_Start(int MaxSockets, int Port)
{ httpd_config_t Config = HTTPD_DEFAULT_CONFIG();
  Config.server_port      = Port;
  Config.task_priority    = tskIDLE_PRIORITY+3;
  Config.max_open_sockets = MaxSockets;
  esp_err_t Err=httpd_start(&HTTPserver, &Config); if(Err!=ESP_OK) return Err;
  httpd_register_uri_handler(HTTPserver, &HTTPtop);  // top URL
  httpd_register_uri_handler(HTTPserver, &HTTPparm); // parameters URL
  httpd_register_uri_handler(HTTPserver, &HTTPlog);  // log files URL
  httpd_register_uri_handler(HTTPserver, &HTTPlogo); // OGN logo
  return Err; }

void HTTP_Stop(void)
{ if(HTTPserver) httpd_stop(HTTPserver); HTTPserver=0; }

// ============================================================================================================

#endif // WITH_HTTP
