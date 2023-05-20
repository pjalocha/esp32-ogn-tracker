#ifndef __WIFI_H__
#define __WIFI_H__

#include "tcpip_adapter.h"

#if ESP_IDF_VERSION_MINOR<3
#include "esp_wifi.h" // v4.1
#else
#include <esp_wifi.h> // v4.3
#endif

#include "esp_event_loop.h"

typedef union
{ uint32_t Flags;
  struct
  { uint8_t isON       : 2;
    uint8_t isConnected: 2;
    uint8_t hasIP      : 2;
  } ;
} WIFI_State_t;

extern WIFI_State_t            WIFI_State;

extern wifi_config_t           WIFI_Config;
extern tcpip_adapter_ip_info_t WIFI_IP; // WIFI local IP address, mask and gateway

bool WIFI_isConnected(void);
bool WIFI_isAP(void);

esp_err_t WIFI_Init(void);
esp_err_t WIFI_setPowerSave(bool ON);
esp_err_t WIFI_Start(void);
esp_err_t WIFI_StartAP(const char *SSID, const char *Pass=0, int MaxConnecions=8);
esp_err_t WIFI_Stop(void);
esp_err_t WIFI_setTxPower(int8_t TxPwr=40);
esp_err_t WIFI_Connect(wifi_ap_record_t *AP, const char *Pass, int8_t MinSig=(-90));
esp_err_t WIFI_Connect(const char *SSID, const char *Pass, int8_t MinSig=(-90));
esp_err_t WIFI_Disconnect(void);

uint32_t WIFI_getLocalIP(void);

esp_err_t WIFI_ActiveScan(wifi_ap_record_t *AP, uint16_t &APs);
esp_err_t WIFI_PassiveScan(wifi_ap_record_t *AP, uint16_t &APs);

uint8_t AP_Print(char *Out, wifi_ap_record_t *AP);

uint8_t IP_Print(char *Out, uint32_t IP);
void    IP_Print(void (*Output)(char), uint32_t IP);

#endif // __WIFI_H__
