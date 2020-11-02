#include <ctype.h>

#include "hal.h"

#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_http_server.h"

#include "format.h"
#include "fifo.h"
#include "socket.h"
#include "proc.h"
#include "wifi.h"

// #define DEBUG_PRINT

#ifdef WITH_AP

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

  httpd_resp_sendstr_chunk(Req, "<tr><td>OLED page sel.</td><td><input type=\"text\" name=\"PageMask\" size=\"10\" value=\"0x");
  Len=Format_Hex(Line, Parameters.PageMask);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }

static void ParmForm_AP(httpd_req_t *Req) // Wi-Fi access point parameters { char Line[16]; int Len;
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"AP\">\n<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">\n");
  httpd_resp_sendstr_chunk(Req, "<tr><th><b>Wi-Fi AP</th><td><input type=\"submit\" value=\"Apply\"></td></tr>\n");

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
  Len=Format_UnsDec(Line, Parameters.APtxPwr);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\"></td></tr>\n");

  httpd_resp_sendstr_chunk(Req, "</table></form>\n"); }

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

  httpd_resp_sendstr_chunk(Req, "<h1>OGN-Tracker configuration</h1>\n");
  httpd_resp_sendstr_chunk(Req, "<b>CPU ID: ");
  Len=Format_Hex(Line, getUniqueID());
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "</b><br />\n");

  httpd_resp_sendstr_chunk(Req, "<table>\n<tr><td>\n");
  ParmForm_Acft(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
  ParmForm_Other(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
  ParmForm_Info(Req);
  httpd_resp_sendstr_chunk(Req, "</td></tr>\n<tr><td>\n");
  ParmForm_AP(Req);
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
  httpd_resp_sendstr_chunk(Req, 0);
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

static const httpd_uri_t HTTPtop =
{ .uri       = "/",
  .method    = HTTP_GET,
  .handler   = top_get_handler,
  .user_ctx  = 0 };

static const httpd_uri_t HTTPparm =
{ .uri       = "/parm.html",
  .method    = HTTP_GET,
  .handler   = parm_get_handler,
  .user_ctx  = 0 };

static httpd_handle_t HTTPserver = 0;

static esp_err_t HTTP_Start(int MaxSockets=4, int Port=80)
{ httpd_config_t Config = HTTPD_DEFAULT_CONFIG();
  Config.server_port      = Port;
  Config.task_priority    = tskIDLE_PRIORITY+3;
  Config.max_open_sockets = MaxSockets;
  esp_err_t Err=httpd_start(&HTTPserver, &Config); if(Err!=ESP_OK) return Err;
  httpd_register_uri_handler(HTTPserver, &HTTPtop);
  httpd_register_uri_handler(HTTPserver, &HTTPparm);
  return Err; }

static void HTTP_Stop(void)
{ if(HTTPserver) httpd_stop(HTTPserver); HTTPserver=0; }

// ============================================================================================================

DataServer PortServer;

static FIFO<char, 1024> AP_TxFIFO;
static FIFO<uint8_t, 256> AP_RxFIFO;

void AP_Write(char Byte) { AP_TxFIFO.Write(Byte); }

static int AP_TxPush(size_t MaxLen=256)                           // transmit part of the TxFIFO to the TCP clients
{ char *Data; size_t Len=AP_TxFIFO.getReadBlock(Data);            // see how much data is there in the queue for transmission
  if(Len==0) return 0;                                            // if block is empty then give up
  if(Len>MaxLen) Len=MaxLen;                                      // limit the block size
  int Ret=PortServer.Send(Data, Len);                             // write the block to the Stratux socket
  if(Ret<0) return -1;                                            // if an error then give up
  AP_TxFIFO.flushReadBlock(Len);                                  // remove the transmitted block from the FIFO
  return Len; }                                                   // return number of transmitted bytes

// ============================================================================================================

extern "C"
void vTaskAP(void* pvParameters)
{ esp_err_t Err;
  AP_TxFIFO.Clear();
  AP_RxFIFO.Clear();
  vTaskDelay(1000);

  WIFI_State.Flags=0;
  Err=WIFI_Init();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "WIFI_Init() => ");
  if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif

  Err=WIFI_StartAP(Parameters.APname, Parameters.APpass);
  WIFI_setTxPower(Parameters.APtxPwr);
  WIFI_setPowerSave(1);

  Err=PortServer.Listen(Parameters.APport);
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "PortServer.Listen() => ");
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif

  Err=HTTP_Start();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "HTTP_Start() => ");
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  // vTaskDelay(1000);

  for( ; ; )                                                       // main (endless) loop
  { Err=AP_TxPush();
    if(Err>0) { vTaskDelay(1); continue; }
    vTaskDelay(50);
    Err = PortServer.Accept();
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "PortServer.Accept() => ");
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif


  }

}

#endif // WITH_AP
