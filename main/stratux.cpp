#include <ctype.h>

#include "hal.h"

#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "format.h"
#include "fifo.h"
#include "socket.h"
#include "proc.h"
#include "wifi.h"
#include "stratux.h"

#ifdef WITH_HTTP
#include "http.h"
#endif

#define DEBUG_PRINT

#ifdef WITH_STRATUX

// ==============================================================================================

static char   Stratux_Host[32] = { 0 };
static char   Stratux_Port[8];
static Socket Stratux_Socket;

static FIFO<char, 512> Stratux_TxFIFO;
static FIFO<uint8_t, 256> Stratux_RxFIFO;
static uint32_t Stratux_ConnectDelay = 2000;

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
  { Stratux_ConnectDelay*=2; if(Stratux_ConnectDelay>60000) Stratux_ConnectDelay=60000;
    vTaskDelay(Stratux_ConnectDelay);
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
    if(Err) { WIFI_Disconnect(); WIFI_Stop(); continue; }

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
    if(WIFI_IP.ip.addr==0) { WIFI_Disconnect(); WIFI_Stop(); continue; }     // if getting local IP failed then give up

#ifdef WITH_HTTP
    HTTP_Start();
#endif
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
      Stratux_ConnectDelay = 2000;
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
#ifdef WITH_HTTP
    HTTP_Stop();
#endif
    WIFI_Disconnect(); WIFI_Stop(); WIFI_IP.ip.addr=0;
    vTaskDelay(2000);
  }

}

#endif // WITH_STRATUX
