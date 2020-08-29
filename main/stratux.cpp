#include "hal.h"

#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include <ctype.h>
#include "format.h"

#include "socket.h"

#include "proc.h"

#include "stratux.h"

#define DEBUG_PRINT

#ifdef WITH_STRATUX

// ==============================================================================================

wifi_config_t WIFI_Config;   // WIFI config: ESSID, etc.
uint32_t WIFI_LocalIP = 0;   // WIFI local IP address
bool WIFI_isConnected(void) { return WIFI_LocalIP!=0; }

static esp_err_t WIFI_event_handler(void *ctx, system_event_t *event)
{
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "WIFI_event_handler => ");
  Format_SignDec(CONS_UART_Write, event->event_id);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif

  switch (event->event_id)
  { case SYSTEM_EVENT_STA_START:        // #2
      // esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_STOP:         // #3
      break;
    case SYSTEM_EVENT_STA_CONNECTED:    // #4
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED: // #5
      // esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:       // #7
      // ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip)
      break;
    case SYSTEM_EVENT_STA_LOST_IP:      // #8
      break;
    default:
      break;
  }
  return ESP_OK; }

static esp_err_t WIFI_Init(void)
{ esp_err_t Err;
  tcpip_adapter_init();
  Err = esp_event_loop_init(WIFI_event_handler, NULL); if(Err!=ESP_OK) return Err;
  wifi_init_config_t Config = WIFI_INIT_CONFIG_DEFAULT();
  Err = esp_wifi_init(&Config); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  return Err; }

static esp_err_t WIFI_Start(void)
{ esp_err_t Err;
  Err = esp_wifi_set_mode(WIFI_MODE_STA); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_start(); return Err; }

static esp_err_t WIFI_Stop(void)
{ return esp_wifi_stop(); }

static esp_err_t WIFI_Connect(wifi_ap_record_t *AP, const char *Pass) // connect to given Access Point with gicen password
{ esp_err_t Err;
  memcpy(WIFI_Config.sta.ssid, AP->ssid, 32);
  if(Pass) strncpy((char *)WIFI_Config.sta.password, Pass, 64);
     else  WIFI_Config.sta.password[0]=0;
  memcpy(WIFI_Config.sta.bssid, AP->bssid, 6);
  WIFI_Config.sta.bssid_set = 1;
  WIFI_Config.sta.channel = AP->primary;
  WIFI_Config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
  WIFI_Config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
  WIFI_Config.sta.threshold.rssi = -127;
  Err = esp_wifi_set_config(ESP_IF_WIFI_STA, &WIFI_Config); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_connect(); if(Err!=ESP_OK) return Err;
  return Err; }

static esp_err_t WIFI_Connect(const char *SSID, const char *Pass)
{ esp_err_t Err;
  strncpy((char *)WIFI_Config.sta.ssid, SSID, 32);
  if(Pass && Pass[0]) strncpy((char *)WIFI_Config.sta.password, Pass, 64);
                else  WIFI_Config.sta.password[0]=0;
  WIFI_Config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
  WIFI_Config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
  WIFI_Config.sta.threshold.rssi = -80;
  Err = esp_wifi_set_config(ESP_IF_WIFI_STA, &WIFI_Config); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_connect(); if(Err!=ESP_OK) return Err;
  return Err; }

static esp_err_t WIFI_Disconnect(void)                      // disconnect from WiFi AP
{ esp_err_t Err=esp_wifi_disconnect();
  return Err; }

static uint32_t WIFI_getLocalIP(void)                       // get local IP, once DHCP phase is done
{ esp_err_t Err;
  tcpip_adapter_ip_info_t Info;
  Err=tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &Info); if(Err!=ESP_OK) return 0;
  return Info.ip.addr; }

uint8_t IP_Print(char *Out, uint32_t IP)
{ uint8_t Len=0;
  for(uint8_t Idx=0; Idx<4; Idx++)
  { if(Idx) Out[Len++]='.';
    Len+=Format_UnsDec(Out+Len, IP&0xFF);
    IP>>=8; }
  Out[Len]=0; return Len; }

void IP_Print(void (*Output)(char), uint32_t IP)
{ for(uint8_t Idx=0; Idx<4; Idx++)
  { if(Idx) Output('.');
    Format_UnsDec(Output, IP&0xFF);
    IP>>=8; }
}

// ==============================================================================================

bool Stratux_isConnected(void)
{ return 0; }

 int Stratux_Read (uint8_t &Byte)
{ return 0; }

void Stratux_Write (char Byte)
{ }

extern "C"
void vTaskWIFI(void* pvParameters)
{ esp_err_t Err;
  vTaskDelay(1000);

  Err=WIFI_Init();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "WIFI_Init() => ");
  if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif

  Err=WIFI_Start();                                              // start WiFi
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "WIFI_Start() => ");
  if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif

  for( ; ; )                                                       // main (endless) loop
  { vTaskDelay(1000);
    if(Parameters.StratuxPass[0]==0) continue;
    Err=WIFI_Connect(Parameters.StratuxWIFI, Parameters.StratuxPass);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, Parameters.StratuxWIFI);
    if(Parameters.StratuxPass[0]) { CONS_UART_Write('/'); Format_String(CONS_UART_Write, Parameters.StratuxPass); }
    CONS_UART_Write(':');
    CONS_UART_Write(' ');
    Format_String(CONS_UART_Write, "WIFI_Connect() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    if(Err) { vTaskDelay(10000); continue; }

    WIFI_LocalIP=0;
    for(uint8_t Idx=0; Idx<10; Idx++)                     // wait to obtain local IP from DHCP
    { vTaskDelay(1000);
      WIFI_LocalIP = WIFI_getLocalIP();
      if(WIFI_LocalIP) break; }

#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "LocalIP: ");
    IP_Print(CONS_UART_Write, WIFI_LocalIP);
    // Format_Hex(CONS_UART_Write, WIFI_LocalIP);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    if(WIFI_LocalIP==0) { WIFI_Disconnect(); continue; }     // if getting local IP failed then give up

    vTaskDelay(5000);
    WIFI_Disconnect(); WIFI_LocalIP=0;
    vTaskDelay(2000);
  }

}

#endif // WITH_STRATUX
