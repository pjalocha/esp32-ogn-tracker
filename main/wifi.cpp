#include <string.h>

#include "wifi.h"
#include "format.h"

wifi_config_t WIFI_Config;        // WIFI config: ESSID, etc.
tcpip_adapter_ip_info_t WIFI_IP = { 0, 0, 0 };  // WIFI local IP address, mask and gateway
WIFI_State_t WIFI_State;

bool WIFI_isConnected(void) { return WIFI_IP.ip.addr!=0; }  // return "connected" status when IP from DHCP is there

static esp_err_t WIFI_event_handler(void *ctx, system_event_t *event)
{
#ifdef DEBUG_PRINT
  const char *EventName[9] = { "WIFI_READY", "SCAN_DONE", "STA_START", "STA_STOP", "STA_CONN", "STA_DISCONN", "AUTHMODE", "GOT_IP", "LOST_IP" } ;
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

esp_err_t WIFI_Init(void)
{ esp_err_t Err;
  tcpip_adapter_init();
  Err = esp_event_loop_init(WIFI_event_handler, NULL); if(Err!=ESP_OK) return Err;
  wifi_init_config_t Config = WIFI_INIT_CONFIG_DEFAULT();
  Err = esp_wifi_init(&Config); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  return Err; }

esp_err_t WIFI_setPowerSave(bool ON)
{ return esp_wifi_set_ps(ON?WIFI_PS_MAX_MODEM:WIFI_PS_MIN_MODEM); }

esp_err_t WIFI_Start(void)
{ esp_err_t Err;
  Err = esp_wifi_set_mode(WIFI_MODE_STA); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_start();
  return Err; }

esp_err_t WIFI_StartAP(const char *SSID, const char *Pass, int MaxConnections)
{ esp_err_t Err;
  WIFI_Config.ap.ssid_len = strlen(SSID);
  strcpy((char *)WIFI_Config.ap.ssid, SSID);
  if(Pass) strcpy((char *)WIFI_Config.ap.password, Pass);
      else WIFI_Config.ap.password[0]=0;
  WIFI_Config.ap.max_connection = MaxConnections;
  WIFI_Config.ap.authmode = (Pass && Pass[0]) ? WIFI_AUTH_WPA_WPA2_PSK:WIFI_AUTH_OPEN;
  // tcpip_adapter_ip_info_t IPinfo;                             // attempt to change the IP range to 192.168.1.1 but does not work
  // IP4_ADDR(&IPinfo.ip, 192,168,1,1);
  // IP4_ADDR(&IPinfo.gw, 192,168,1,1);
  // IP4_ADDR(&IPinfo.netmask, 255,255,255,0);
  // tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &IPinfo);
  Err = esp_wifi_set_config(ESP_IF_WIFI_AP, &WIFI_Config); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_set_mode(WIFI_MODE_AP); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_start();
  return Err; }

esp_err_t WIFI_Stop(void)
{ return esp_wifi_stop(); }

esp_err_t WIFI_setTxPower(int8_t TxPwr) // [0.25dBm] 80:L0=20dBm, 76:L1, 74:L2, 70:L3, 64:L4, 56:L5, 50:L5-2dBm, 0:L5-14dBm
{ return esp_wifi_set_max_tx_power(TxPwr); }

esp_err_t WIFI_ActiveScan(wifi_ap_record_t *AP, uint16_t &APs)
{ esp_err_t Err;
  wifi_scan_config_t Config = { ssid:0, bssid:0, channel:0, show_hidden:0,
                                scan_type:WIFI_SCAN_TYPE_ACTIVE,
                                scan_time: { active: { min:100, max:2000 } } };
  Err = esp_wifi_scan_start(&Config, 1); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_scan_get_ap_records(&APs, AP); return Err; }

esp_err_t WIFI_PassiveScan(wifi_ap_record_t *AP, uint16_t &APs) //
{ esp_err_t Err;
  wifi_scan_config_t Config = { ssid:0, bssid:0, channel:0, show_hidden:0,
                                scan_type:WIFI_SCAN_TYPE_PASSIVE,
                                scan_time: { passive: 100 } };
  Err = esp_wifi_scan_start(&Config, 1); if(Err!=ESP_OK) return Err;
  Err = esp_wifi_scan_get_ap_records(&APs, AP); return Err; }

esp_err_t WIFI_Connect(wifi_ap_record_t *AP, const char *Pass, int8_t MinSig) // connect to given Access Point with gicen password
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

esp_err_t WIFI_Connect(const char *SSID, const char *Pass, int8_t MinSig)
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

esp_err_t WIFI_Disconnect(void)                      // disconnect from WiFi AP
{ esp_err_t Err=esp_wifi_disconnect();
  return Err; }

uint32_t WIFI_getLocalIP(void)                       // get local IP, once DHCP phase is done
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


