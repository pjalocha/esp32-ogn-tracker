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

static void RespChunk(httpd_req_t *Req, const char *Resp, int Len) { httpd_resp_send_chunk(Req, Resp, Len); }
static void RespChunk(httpd_req_t *Req, const char *Resp) { httpd_resp_send_chunk(Req, Resp, strlen(Resp)); }
static void RespEnd(httpd_req_t *Req) { httpd_resp_send_chunk(Req, 0, 0); };

static esp_err_t status_get_handler(httpd_req_t *Req)
{ char Line[64];
  RespChunk(Req, "\
<!DOCTYPE html>\r\n\
<html>\r\n\
<title>OGN-Tracker status</title>\r\n\
");
  int Len=Parameters.Print(Line);
  RespChunk(Req, Line, Len);
  RespChunk(Req, "</html>\r\n");
  RespEnd(Req);
  return ESP_OK; }

static const httpd_uri_t HTTPstatus =
{ .uri       = "/status.html",
  .method    = HTTP_GET,
  .handler   = status_get_handler,
  .user_ctx  = 0 };

static httpd_handle_t HTTPserver = 0;

static esp_err_t HTTP_Start(int MaxSockets=4, int Port=80)
{ httpd_config_t Config = HTTPD_DEFAULT_CONFIG();
  Config.server_port      = Port;
  Config.task_priority    = tskIDLE_PRIORITY+3;
  Config.max_open_sockets = MaxSockets;
  esp_err_t Err=httpd_start(&HTTPserver, &Config); if(Err!=ESP_OK) return Err;
  httpd_register_uri_handler(HTTPserver, &HTTPstatus);
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
