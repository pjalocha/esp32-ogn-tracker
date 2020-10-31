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

static void ParmForm(httpd_req_t *Req)  // produce HTML form for parameters input
{ char Line[16]; int Len;

  httpd_resp_sendstr_chunk(Req, "<form action=\"/parm.html\" method=\"get\" id=\"Parms\">\n");

  httpd_resp_sendstr_chunk(Req, "<br /><label>Address:</label><input type=\"text\" name=\"Address\" size=\"6\" value=\"");
  strcpy(Line, "0xXXXXXX\">\n"); Format_Hex(Line+2, (uint8_t)(Parameters.Address>>16)); Format_Hex(Line+4, (uint16_t)Parameters.Address);
  httpd_resp_sendstr_chunk(Req, Line);

  httpd_resp_sendstr_chunk(Req, "<br /><label>Pilot:</label><input type=\"text\" name=\"Pilot\" size=\"10\" value=\"");
  if(Parameters.Pilot[0]) httpd_resp_sendstr_chunk(Req, Parameters.Pilot);
  httpd_resp_sendstr_chunk(Req, "\">\n");

  httpd_resp_sendstr_chunk(Req, "<br /><label>AP SSID:</label><input type=\"text\" name=\"APname\" size=\"10\" value=\"");
  if(Parameters.APname[0]) httpd_resp_sendstr_chunk(Req, Parameters.APname);
  httpd_resp_sendstr_chunk(Req, "\">\n");

  httpd_resp_sendstr_chunk(Req, "<br /><label>AP pass:</label><input type=\"text\" name=\"APpass\" size=\"10\" value=\"");
  if(Parameters.APpass[0]) httpd_resp_sendstr_chunk(Req, Parameters.APpass);
  httpd_resp_sendstr_chunk(Req, "\">\n");

  httpd_resp_sendstr_chunk(Req, "<br /><label>AP port:</label><input type=\"text\" name=\"APport\" size=\"10\" value=\"");
  Len=Format_UnsDec(Line, Parameters.APport);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">\n");

  httpd_resp_sendstr_chunk(Req, "<br /><label>Verbose:</label><input type=\"text\" name=\"Verbose\" size=\"6\" value=\"");
  Len=Format_UnsDec(Line, Parameters.Verbose);
  httpd_resp_send_chunk(Req, Line, Len);
  httpd_resp_sendstr_chunk(Req, "\">\n");

  httpd_resp_sendstr_chunk(Req, "<br /><label>PageMask:</label><input type=\"text\" name=\"PageMask\" size=\"10\" value=\"");
  strcpy(Line, "0xXXXXXXXX\">\n"); Format_Hex(Line+2, Parameters.PageMask);
  httpd_resp_sendstr_chunk(Req, Line);

  httpd_resp_sendstr_chunk(Req, "<br /><input type=\"submit\" value=\"Save\">\n</form>\n");
}

static esp_err_t parm_get_handler(httpd_req_t *Req)
{ // char Line[64];
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
    free(URL); }
  httpd_resp_sendstr_chunk(Req, "\
<!DOCTYPE html>\n\
<html><body>\n\
<title>OGN-Tracker status</title>\n\
");
  // int Len=Parameters.Print(Line);
  // httpd_resp_send_chunk(Req, Line, Len);
  ParmForm(Req);
  httpd_resp_sendstr_chunk(Req, "</body></html>\n");
  httpd_resp_sendstr_chunk(Req, 0);
  return ESP_OK; }

/*
static esp_err_t parm_get_handler(httpd_req_t *Req)
{ char Line[32];
  // int ContLen=Req->content_len;
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "parm_get_handler() => [");
  // Format_SignDec(CONS_UART_Write, ContLen, 1, 0, 1);
  Format_String(CONS_UART_Write, "] ");
  // for( ; ; )
  // { int Ret = httpd_req_recv(Req, Line, 31); if(Len<=0) break;
  // }
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
  return ESP_OK; }
*/

static const httpd_uri_t HTTPparm =
{ .uri       = "/parm.html",
  .method    = HTTP_GET,
  .handler   = parm_get_handler,
  .user_ctx  = 0 };

/*
static const httpd_uri_t HTTPsave =
{ .uri       = "/parm.html",
  .method    = HTTP_GET,
  .handler   = parm_get_handler,
  .user_ctx  = 0 };
*/

static httpd_handle_t HTTPserver = 0;

static esp_err_t HTTP_Start(int MaxSockets=4, int Port=80)
{ httpd_config_t Config = HTTPD_DEFAULT_CONFIG();
  Config.server_port      = Port;
  Config.task_priority    = tskIDLE_PRIORITY+3;
  Config.max_open_sockets = MaxSockets;
  esp_err_t Err=httpd_start(&HTTPserver, &Config); if(Err!=ESP_OK) return Err;
  httpd_register_uri_handler(HTTPserver, &HTTPparm);
  // httpd_register_uri_handler(HTTPserver, &HTTPsave);
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
