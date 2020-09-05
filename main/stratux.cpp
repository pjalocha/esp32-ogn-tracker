#include <ctype.h>

#include "hal.h"

#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "format.h"
#include "fifo.h"
#include "socket.h"
#include "proc.h"
#include "stratux.h"

#define DEBUG_PRINT

#ifdef WITH_STRATUX

// ==============================================================================================

wifi_config_t WIFI_Config;        // WIFI config: ESSID, etc.
tcpip_adapter_ip_info_t WIFI_IP = { 0, 0, 0 };  // WIFI local IP address, mask and gateway

static union
{ uint32_t Flags;
  struct
  { uint8_t isON       : 2;
    uint8_t isConnected: 2;
    uint8_t hasIP      : 2;
  } ;
} WIFI_State;

bool WIFI_isConnected(void) { return WIFI_IP.ip.addr!=0; }  // return "connected" status when IP from DHCP is there

static esp_err_t WIFI_event_handler(void *ctx, system_event_t *event)
{
#ifdef DEBUG_PRINT
  const char *EventName[9] = { 0, 0, "STA_START", "STA_STOP", "STA_CONN", "STA_DISCONN", 0, "GOT_IP", "LOST_IP" } ;
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "WIFI_event_handler => ");
  if(event->event_id>=0 && event->event_id<9 && EventName[event->event_id])
    Format_String(CONS_UART_Write, EventName[event->event_id]);
  else
    Format_SignDec(CONS_UART_Write, event->event_id);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  switch (event->event_id)
  { case SYSTEM_EVENT_STA_START:        // #2 after WIFI_Start()
      WIFI_State.isON=3;
      break;
    case SYSTEM_EVENT_STA_STOP:         // #3 after WIFI_Stop()
      WIFI_State.isON=1;
      break;
    case SYSTEM_EVENT_STA_CONNECTED:    // #4 after WIFI_Connect();
      WIFI_State.isConnected=3;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED: // #5 after WIFI_Connect();
      WIFI_State.isConnected=1;
      break;
    case SYSTEM_EVENT_STA_GOT_IP:       // #7
      // ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip)
      WIFI_State.hasIP=3;
      break;
    case SYSTEM_EVENT_STA_LOST_IP:      // #8
      WIFI_State.hasIP=1;
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

static esp_err_t WIFI_setPowerSave(bool ON)
{ return esp_wifi_set_ps(ON?WIFI_PS_MAX_MODEM:WIFI_PS_MIN_MODEM); }

static esp_err_t WIFI_Start(void)
{ esp_err_t Err;
  Err = esp_wifi_set_mode(WIFI_MODE_STA); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_start();
  return Err; }

static esp_err_t WIFI_Stop(void)
{ return esp_wifi_stop(); }

static esp_err_t WIFI_setTxPower(int8_t TxPwr=40) // [0.25dBm] 80:L0=20dBm, 76:L1, 74:L2, 70:L3, 64:L4, 56:L5, 50:L5-2dBm, 0:L5-14dBm
{ return esp_wifi_set_max_tx_power(TxPwr); }

static esp_err_t WIFI_Connect(wifi_ap_record_t *AP, const char *Pass, int8_t MinSig=(-90)) // connect to given Access Point with gicen password
{ esp_err_t Err;
  memcpy(WIFI_Config.sta.ssid, AP->ssid, 32);
  if(Pass) strncpy((char *)WIFI_Config.sta.password, Pass, 64);
     else  WIFI_Config.sta.password[0]=0;
  memcpy(WIFI_Config.sta.bssid, AP->bssid, 6);
  WIFI_Config.sta.bssid_set = 1;
  WIFI_Config.sta.channel = AP->primary;
  WIFI_Config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
  WIFI_Config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
  WIFI_Config.sta.threshold.rssi = MinSig;
  Err = esp_wifi_set_config(ESP_IF_WIFI_STA, &WIFI_Config); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_connect(); if(Err!=ESP_OK) return Err;
  return Err; }

static esp_err_t WIFI_Connect(const char *SSID, const char *Pass, int8_t MinSig=(-90))
{ esp_err_t Err;
  strncpy((char *)WIFI_Config.sta.ssid, SSID, 32);
  if(Pass && Pass[0]) strncpy((char *)WIFI_Config.sta.password, Pass, 64);
                else  WIFI_Config.sta.password[0]=0;
  WIFI_Config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
  WIFI_Config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
  WIFI_Config.sta.threshold.rssi = MinSig;
  Err = esp_wifi_set_config(ESP_IF_WIFI_STA, &WIFI_Config); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_connect(); if(Err!=ESP_OK) return Err;
  return Err; }

static esp_err_t WIFI_Disconnect(void)                      // disconnect from WiFi AP
{ esp_err_t Err=esp_wifi_disconnect();
  return Err; }

static uint32_t WIFI_getLocalIP(void)                       // get local IP, once DHCP phase is done
{ esp_err_t Err;
  Err=tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &WIFI_IP); if(Err!=ESP_OK) return 0;
  return WIFI_IP.ip.addr; }

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

static char   Stratux_Host[32] = { 0 };
static char   Stratux_Port[8];
static Socket Stratux_Socket;

static FIFO<char, 512> Stratux_TxFIFO;
static FIFO<uint8_t, 256> Stratux_RxFIFO;

bool Stratux_isConnected(void)
{ return WIFI_isConnected() && Stratux_Socket.isConnected(); }

 int Stratux_Read (uint8_t &Byte)
{ return Stratux_RxFIFO.Read(Byte); }

void Stratux_Write (char Byte)
{ Stratux_TxFIFO.Write(Byte); }

static int Stratux_TxPush(size_t MaxLen=256)                      // transmit part of the TxFIFO to the Stratux link
{ char *Data; size_t Len=Stratux_TxFIFO.getReadBlock(Data);       // see how much data is there in the queue for transmission
  if(Len==0) return 0;                                            // if block is empty then give up
  if(Len>MaxLen) Len=MaxLen;                                      // limit the block size
  int Ret=Stratux_Socket.Send(Data, Len);                         // write the block to the Stratux socket
  if(Ret!=Len) return -1;                                         // if an error then give up
  Stratux_TxFIFO.flushReadBlock(Len);                             // remove the transmitted block from the FIFO
  return Len; }                                                   // return number of transmitted bytes

extern "C"
void vTaskSTX(void* pvParameters)
{ esp_err_t Err;
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

  for( ; ; )                                                       // main (endless) loop
  { vTaskDelay(1000);
    if(Parameters.StratuxWIFI[0]==0) continue;

    Err=WIFI_Start();                                              // start WiFi
    WIFI_setTxPower(Parameters.StratuxTxPwr);
    WIFI_setPowerSave(1);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_Start() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif

    WIFI_State.isConnected=2;
    Err=WIFI_Connect(Parameters.StratuxWIFI, Parameters.StratuxPass, Parameters.StratuxMinSig);
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
    if(Err) { WIFI_Disconnect(); WIFI_Stop(); vTaskDelay(20000); continue; }

    WIFI_IP.ip.addr = 0;
    for(uint8_t Idx=0; Idx<10; Idx++)                     // wait to obtain local IP from DHCP
    { vTaskDelay(1000);
      if(WIFI_State.isConnected==1) break;
      if(WIFI_getLocalIP()) break; }
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Local IP: ");
    IP_Print(CONS_UART_Write, WIFI_IP.ip.addr);
    Format_String(CONS_UART_Write, " GW: ");
    IP_Print(CONS_UART_Write, WIFI_IP.gw.addr);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    if(WIFI_IP.ip.addr==0) { WIFI_Disconnect(); WIFI_Stop(); vTaskDelay(20000); continue; }     // if getting local IP failed then give up

    Stratux_TxFIFO.Clear();

    uint8_t Len=Format_UnsDec(Stratux_Port, Parameters.StratuxPort); Stratux_Port[Len]=0;                 // port number in ASCII form
    const char *Host = Parameters.StratuxHost;
    if(Host[0]==0) { Len=IP_Print(Stratux_Host, WIFI_IP.gw.addr); Stratux_Host[Len]=0; Host=Stratux_Host; } // if internal host: gateway IP in ASCII form
    int ConnErr=Stratux_Socket.Connect(Host, Stratux_Port);                                               // connect to the Stratux
    if(ConnErr>=0)                                                                                        // if connection succesfull
    { Stratux_Socket.setReceiveTimeout(1);
// #ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "Connected to ");
      IP_Print(CONS_UART_Write, Stratux_Socket.getIP());
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex);
// #endif
      for( ; ; )
      { int Len=Stratux_TxPush(); if(Len<0) break;
        if(Len==0) vTaskDelay(5);
        if(WIFI_State.isConnected!=3 || WIFI_State.hasIP!=3) break;
// #ifdef DEBUG_PRINT
//         xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
//         Format_String(CONS_UART_Write, "Stratux_TxPush() => ");
//         Format_SignDec(CONS_UART_Write, Len);
//         Format_String(CONS_UART_Write, "\n");
//         xSemaphoreGive(CONS_Mutex);
// #endif
      }
    }
    else
    { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "Failed to connect to Stratux -> ");
      Format_SignDec(CONS_UART_Write, ConnErr);
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex); }

    Stratux_Socket.Disconnect();
    vTaskDelay(2000);
    WIFI_Disconnect(); WIFI_Stop(); WIFI_IP.ip.addr=0;
    vTaskDelay(2000);
  }

}

#endif // WITH_STRATUX
