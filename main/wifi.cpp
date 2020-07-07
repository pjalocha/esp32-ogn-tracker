#include "hal.h"

#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include <ctype.h>
#include "format.h"

#include "socket.h"

#ifdef WITH_WIFI

#define DEBUG_PRINT

// tutorial on WiFi and scan: http://www.lucadentella.it/en/2017/01/09/esp32-5-wifi-scanner/
// example on TCP/IP: https://github.com/espressif/esp-idf/blob/master/examples/protocols/http_request/main/http_request_example_main.c
//                    https://github.com/lucadentella/esp32-tutorial/blob/master/04_httpclient/main/main.c

// ======================================================================================================

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

static esp_err_t WIFI_ActiveScan(wifi_ap_record_t *AP, uint16_t &APs)
{ esp_err_t Err;
  wifi_scan_config_t Config = { ssid:0, bssid:0, channel:0, show_hidden:0,
                                scan_type:WIFI_SCAN_TYPE_ACTIVE,
                                scan_time: { active: { min:100, max:2000 } } };
  Err = esp_wifi_scan_start(&Config, 1); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_scan_get_ap_records(&APs, AP); return Err; }

static esp_err_t WIFI_PassiveScan(wifi_ap_record_t *AP, uint16_t &APs)
{ esp_err_t Err;
  wifi_scan_config_t Config = { ssid:0, bssid:0, channel:0, show_hidden:0,
                                scan_type:WIFI_SCAN_TYPE_PASSIVE,
                                scan_time: { passive: 100 } };
  Err = esp_wifi_scan_start(&Config, 1); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_scan_get_ap_records(&APs, AP); return Err; }

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

static esp_err_t WIFI_Disconnect(void)
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

uint8_t AP_Print(char *Out, wifi_ap_record_t *AP)                  // print numbers for a found Wi-Fi Access Point
{ uint8_t Len=0;
  for(uint8_t Idx=0; Idx<6; Idx++)
  { Len+=Format_Hex(Out+Len, AP->bssid[Idx]); Out[Len++]=':'; }
  Len+=Format_String(Out+Len, (const char *)(AP->ssid), 32, 32);
  Out[Len++]=' ';
  Out[Len++]='0'+AP->authmode;
  Out[Len++]=' ';
  Len+=Format_UnsDec(Out+Len, AP->primary, 2);
  Len+=Format_String(Out+Len, "ch/");
  Len+=Format_SignDec(Out+Len, AP->rssi);
  Len+=Format_String(Out+Len, "dBm ");
  Out[Len++]=AP->phy_11b ? 'B':'_';
  Out[Len++]=AP->phy_11g ? 'G':'_';
  Out[Len++]=AP->phy_11n ? 'N':'_';
  Out[Len++]=AP->phy_lr  ? 'L':'_';
  Out[Len++]=AP->wps     ? 'W':'_';
  Out[Len++]='\n'; Out[Len]=0;
  return Len; }

static wifi_ap_record_t AP[8];
static uint16_t APs=0;

const int MaxLineLen=512;
static char Line[MaxLineLen];

static void AP_Print(void (*Output)(char))
{ for(uint16_t Idx=0; Idx<APs; Idx++)
  { Line[0]='0'+Idx; Line[1]=':'; Line[2]=' ';
    uint8_t Len=3+AP_Print(Line+3, AP+Idx);
    Format_String(Output, Line); }
}

Socket APRS_Socket;                                                 // socket to talk to APRS server
bool APRS_isConnected(void) { return APRS_Socket.isConnected(); }

static const char *APRS_Host = "aprs.glidernet.org";                // server address
static const char *APRS_Port = "14580";                             // server socket
static const char *APRS_Soft = "OGN-Tracker";                       // software name for APRS server

static uint16_t APRS_CallPass(const char *Call)                     // calc. password for the APRS server login
{ uint16_t Hash=0x73e2;
  for( ; ; )
  { uint16_t High=(*Call++); if((High==0)||(High=='-')) break;
    High=toupper(High); Hash^=(High<<8);
    uint16_t Low =(*Call++); if((Low ==0)||(Low =='-')) break;
    Low=toupper(Low); Hash^=(Low); }
  return Hash&0x7FFF; }

static uint8_t LoginLine(char *Line, /* , const char *Call,*/ uint16_t Range=0 )
{ uint8_t Len=0;
  Len += Format_String(Line, "user ");           // prepare the login line for the APRS server
  Len += Parameters.getAprsCall(Line+Len);
  uint16_t Pass = APRS_CallPass(Line+5);
  Len += Format_String(Line+Len, " pass ");
  Len += Format_UnsDec(Line+Len, Pass);
  Len += Format_String(Line+Len, " vers OGN-Tracker v0.0");
  if(Range)
  { Len += Format_String(Line+Len, " filter m/");
    Len += Format_UnsDec(Line+Len, Range); }
  Line[Len++]='\n';
  Line[Len]=0;
  return Len; }

int APRS_RxMsg(const char *Msg)
{
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "APRS -> ");
  Format_String(CONS_UART_Write, Msg);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  return 0; }

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
  for( ; ; )                                                       // main (endless) loop
  { vTaskDelay(5000);
    Err=WIFI_Start();                                              // start WiFi
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_Start() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
/*
    vTaskDelay(5000);
    APs=8;
    Err=WIFI_ActiveScan(AP, APs);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_ActiveScan() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    CONS_UART_Write('/');
    Format_UnsDec(CONS_UART_Write, APs);
    Format_String(CONS_UART_Write, "\n");
    if(Err==ESP_OK) AP_Print(CONS_UART_Write);
    xSemaphoreGive(CONS_Mutex);
#endif
*/
    vTaskDelay(1000);
    APs=8;
    Err=WIFI_PassiveScan(AP, APs);                           // perform a passive scan: find Access Points around
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_PassiveScan() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    CONS_UART_Write('/');
    Format_UnsDec(CONS_UART_Write, APs);
    Format_String(CONS_UART_Write, "\n");
    if(Err==ESP_OK) AP_Print(CONS_UART_Write);
    xSemaphoreGive(CONS_Mutex);
#endif

    if(Err==ESP_OK)                                           // if WiFi scan went well
    { for(uint16_t Idx=0; Idx<APs; Idx++)                     // loop over Access Points
      { const char *NetName  = (const char *)(AP[Idx].ssid);
        const char *NetPass = 0;
        if(AP[Idx].authmode!=WIFI_AUTH_OPEN)                  // if not an open network
        { NetPass=Parameters.getWIFIpass(NetName);            // then search the password
          if(NetPass==0) continue; }                          // give up if no password for this network
        Err=WIFI_Connect(AP+Idx, NetPass);
#ifdef DEBUG_PRINT
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, NetName);
        if(NetPass) { CONS_UART_Write('/'); Format_String(CONS_UART_Write, NetPass); }
        CONS_UART_Write(':');
        CONS_UART_Write(' ');
        Format_String(CONS_UART_Write, "WIFI_Connect() => ");
        if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
        Format_SignDec(CONS_UART_Write, Err);
        Format_String(CONS_UART_Write, "\n");
        xSemaphoreGive(CONS_Mutex);
#endif
        if(Err) continue;                                     // if connection failed then give up

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

        int ConnErr=APRS_Socket.Connect(APRS_Host, APRS_Port);   // connect to the APRS server
        if(ConnErr>=0)                                           // if connection succesfull
        {
          uint8_t LoginLen = LoginLine(Line, 5);
#ifdef DEBUG_PRINT
          xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
          Format_String(CONS_UART_Write, "Connected to ");
          IP_Print(CONS_UART_Write, APRS_Socket.getIP());
          Format_String(CONS_UART_Write, "\nAPRS <- ");
          Format_String(CONS_UART_Write, Line);
          xSemaphoreGive(CONS_Mutex);
#endif
          APRS_Socket.setReceiveTimeout(1);                      // [sec] receive timeout
          // APRS_Socket.setBlocking(0);
          int Write=APRS_Socket.Send(Line, LoginLen);            // send login to the APRS server
          int LinePtr=0;
          for(int Idx=0; Idx<120; Idx++)                         // wait for some time and watch what the server sends to us
          { int Left = MaxLineLen-LinePtr; if(Left<1) break;
            int Read=APRS_Socket.Receive(Line+LinePtr, Left-1);
#ifdef DEBUG_PRINT
            xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
            Format_String(CONS_UART_Write, "APRS: ");
            Format_SignDec(CONS_UART_Write, Read);
            Format_String(CONS_UART_Write, "\n");
            xSemaphoreGive(CONS_Mutex);
#endif
            if(Read<0) break;                                    // if an error reading from the APRS server
            if(Read==0) continue;                                // no more bytes: keep going
            int LineLen = LinePtr+Read;
            int MsgPtr = 0;
            Line[LineLen]=0;
            for(int Idx=LinePtr; Idx<LineLen; Idx++)
            { char ch=Line[Idx]; if(ch==0) break;
              if(ch=='\n')
              { Line[Idx]=0; APRS_RxMsg(Line+MsgPtr); MsgPtr=Idx+1; }
            }
            if(MsgPtr>0)
            { int Copy=LineLen-MsgPtr;
              if(Copy>0)
              { memcpy(Line, Line+MsgPtr, Copy); LinePtr=0; }
            }
          }
        }
#ifdef DEBUG_PRINT
        else                                                      // if connection failed to the APRS server
        { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
          Format_String(CONS_UART_Write, "Failed to connect to APRS -> ");
          Format_SignDec(CONS_UART_Write, ConnErr);
          Format_String(CONS_UART_Write, "\n");
          xSemaphoreGive(CONS_Mutex); }
#endif
        APRS_Socket.Disconnect();
        vTaskDelay(5000);
        WIFI_Disconnect(); WIFI_LocalIP=0;
        vTaskDelay(2000);
      }
    }

    vTaskDelay(2000);
    Err=WIFI_Stop();
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_Stop() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
  }
}

#endif // WITH_WIFI

