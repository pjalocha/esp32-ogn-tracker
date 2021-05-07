
// ~/esp-idf/components/bt/bluedroid/api/include/esp_spp_api.h
// esp_err_t esp_spp_write(uint32_t handle, int len, uint8_t *p_data);

// #ifdef WITH_BT_SPP

#include "hal.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "format.h"
#include "fifo.h"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t  sec_mask     = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave   = ESP_SPP_ROLE_SLAVE;

static FIFO<char, 1024> BT_SPP_TxFIFO;        // buffer for console output to be sent over BT
static FIFO<uint8_t, 256> BT_SPP_RxFIFO;      // buffer for BT data to be send to the console
static uint32_t        BT_SPP_Conn = 0;       // BT incoming connection handle
static uint32_t        BT_SPP_TxCong = 0;     // congestion control
// static TickType_t      BT_SPP_LastTxPush=0;   // [ms]

// static esp_bd_addr_t BT_SPP_MAC;  // BT incoming connection MAC - could be used for pilot id in the flight log
// static uint32_t BT_SPP_Wait = 0;  // bytes waiting to be written into BT_SPP

// static const char *BT_SPP_Welcome = "ESP32 OGN-Tracker\n";

bool BT_SPP_isConnected(void) { return BT_SPP_Conn; }             // is a client connected to BT_SPP ?

static void setPilotID(esp_bd_addr_t MAC, size_t Len=6)           // set PilotID in the parameters from the BT SPP client MAC (thus Pilot's smartphone)
{ char *ID = Parameters.PilotID;
  ID[0]='B'; ID[1]='T'; ID[2]='_'; ID+=3;
  for(int Idx=0; Idx<Len; Idx++)
  { Format_Hex(ID, MAC[Idx]); ID+=2; }
  ID[0]=0; }

static void clrPilotID(void)                                      // clear the Pilot_ID when BT SPP gets disconnected
{ Parameters.PilotID[0]=0; }

static size_t BT_SPP_TxPush(size_t MaxLen=128)                    // transmit part of the TxFIFO to the BT link
{ // BT_SPP_LastTxPush = xTaskGetTickCount();                        // [ms] remember last time the TxPush was done
  char *Data; size_t Len=BT_SPP_TxFIFO.getReadBlock(Data);        // see how much data is there in the queue for transmission
  if(Len==0) return 0;                                            // if block is empty then give up
  if(Len>MaxLen) Len=MaxLen;                                      // limit the block size
  esp_err_t Ret=esp_spp_write(BT_SPP_Conn, Len, (uint8_t *)Data); // write the block to the BT
  if(Ret!=ESP_OK) return 0;                                       // if an error then give up
  BT_SPP_TxFIFO.flushReadBlock(Len);                              // remove the transmitted block from the FIFO
  return Len; }                                                   // return number of transmitted bytes

static void esp_spp_cb(esp_spp_cb_event_t Event, esp_spp_cb_param_t *Param)
{ switch (Event)
  { case ESP_SPP_INIT_EVT:                                        // [0]
      esp_bt_dev_set_device_name(Parameters.BTname);
      // esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE); // for older ESP-IDF
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
      esp_spp_start_srv(sec_mask, role_slave, 0, "SPP_SERVER");
      break;
    case ESP_SPP_DISCOVERY_COMP_EVT:                              // [8]
      break;
    case ESP_SPP_START_EVT:                                       // [28] SPP server started succesfully
      break;
    case ESP_SPP_SRV_OPEN_EVT:                                    // [34] server connection opens: new handle comes
      BT_SPP_TxFIFO.Clear();                                      // clear the TxFIFO
      BT_SPP_Conn = Param->srv_open.handle;                       // store handle for esp_spp_write()
      BT_SPP_TxCong = 0;                                          // assume no congestion
      setPilotID(Param->srv_open.rem_bda, sizeof(esp_bd_addr_t)); // PilotID is now taken from the connected BT client
      // memcpy(BT_SPP_MAC, Param->srv_open.rem_bda, sizeof(esp_bd_addr_t));
      // esp_spp_write(Param->srv_open.handle, BT_SPP_Wait, (uint8_t *)BT_SPP_Welcome); // write Welcome message to the BT_SPP
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "BT_SPP: ");
      Format_MAC(CONS_UART_Write, Param->srv_open.rem_bda, sizeof(esp_bd_addr_t));
      Format_String(CONS_UART_Write, " connected\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      break;
    case ESP_SPP_OPEN_EVT:                                        // [26] connection opens: what's the difference to ESP_SPP_SRV_OPEN_EVT ?
      // Param->open.handle, Param->open.rem_bda
      break;
    case ESP_SPP_CLOSE_EVT:                                       // [27] connection closes for given handle
      BT_SPP_Conn=0;                                              // clear the handle: signal the BT connection is off
      clrPilotID();
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "BT_SPP: \n");
      // Format_MAC(CONS_UART_Write, BT_SPP_MAC);
      Format_String(CONS_UART_Write, " disconnected\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      break;
    case ESP_SPP_DATA_IND_EVT:                                    // [30] data is sent by the client
      BT_SPP_RxFIFO.Write(Param->data_ind.data, Param->data_ind.len);
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      // Param->data_ind.handle, Param->data_ind.data, Param->data_ind.len
      Format_String(CONS_UART_Write, "BT_SPP: [");
      Format_UnsDec(CONS_UART_Write, Param->data_ind.len);
      Format_String(CONS_UART_Write, "]\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      break;
    case ESP_SPP_CONG_EVT:                                        // [31] congestion on the outgoing data
      BT_SPP_TxCong = Param->cong.cong;
      break;
    case ESP_SPP_WRITE_EVT:                                       // [33] (queued) data has been sent to the client
      BT_SPP_TxCong = Param->write.cong;
      break;
    default:
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "BT_SPP: Event ");
      Format_UnsDec(CONS_UART_Write, (uint32_t)Event);
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      break;
  }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t Event, esp_bt_gap_cb_param_t *Param)
{
  switch (Event)     // event numbers are in esp-idf/components/bt/bluedroid/api/include/api/esp_gap_bt_api.h
  {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      if (Param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
      { Format_String(CONS_UART_Write, "BT_GAP: ");
        Format_String(CONS_UART_Write, (const char *)Param->auth_cmpl.device_name);
        Format_String(CONS_UART_Write, " authenticated\n"); }
      else
      { Format_String(CONS_UART_Write, "BT_GAP: Authentication failure (");
        Format_SignDec(CONS_UART_Write, Param->auth_cmpl.stat);
        Format_String(CONS_UART_Write, ")\n"); }
      // ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
      // esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
      // ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
      xSemaphoreGive(CONS_Mutex);
      break;
    case ESP_BT_GAP_PIN_REQ_EVT:
     /*
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
      */
      break;
    default:
      break;
    }

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "BT_GAP: Event ");
  Format_UnsDec(CONS_UART_Write, (uint32_t)Event);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
}

int BT_SPP_Read (uint8_t &Byte)   // read a character from the BT serial port (buffer)
{ // if(!BT_SPP_Conn) return 0;
  return BT_SPP_RxFIFO.Read(Byte); }

void BT_SPP_Write (char Byte)     // send a character to the BT serial port
{ if(!BT_SPP_Conn) return;                                                                // if BT connection is active
  BT_SPP_TxFIFO.Write(Byte);                                                              // write the byte into the TxFIFO
  // TickType_t Behind = xTaskGetTickCount() - BT_SPP_LastTxPush;                         // [ms]
  // if(Behind>=20) BT_SPP_TxPush();
  if( (BT_SPP_TxCong==0) && ( (Byte=='\n') || (BT_SPP_TxFIFO.Full()>=64) ) )              // if no congestion and EOL or 64B waiting already
  { BT_SPP_TxPush(); }                                                                    // read a block from TxFIFO ad push it into the BT_SPP
}

int BT_SPP_Init(void)
{ esp_err_t Err=ESP_OK;
  if(Parameters.BTname[0]==0) return Err;

  esp_bt_controller_config_t BTconf = BT_CONTROLLER_INIT_CONFIG_DEFAULT();                  // the default mode is defined by the menuconfig settings
  Err = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  Err = esp_bt_controller_init(&BTconf); if(Err!=ESP_OK) return Err;
  Err = esp_bt_controller_enable((esp_bt_mode_t)BTconf.mode); if(Err!=ESP_OK) return Err;   // mode must be same as in BTconf
  Err = esp_bluedroid_init(); if(Err!=ESP_OK) return Err;                                   // init the BT stack
  Err = esp_bluedroid_enable(); if(Err!=ESP_OK) return Err;                                 // enable the BT stack
  Err = esp_bt_gap_register_callback(esp_bt_gap_cb); if(Err!=ESP_OK) return Err;
  Err = esp_spp_register_callback(esp_spp_cb); if(Err!=ESP_OK) return Err;
  Err = esp_spp_init(esp_spp_mode); if(Err!=ESP_OK) return Err;

  // Set default parameters for Secure Simple Pairing */
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE; // _IO;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

  // Set default parameters for Legacy Pairing: fixed PIN
  esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
  esp_bt_pin_code_t pin_code = { '0', '1', '2', '3' };
  esp_bt_gap_set_pin(pin_type, 4, pin_code);

  // set the UUID so this BT device is recognized as a serial port: thanks to Linar for this code
  esp_bt_cod_t cod;
  cod.minor   =      0b000101;
  cod.major   =       0b00001;
  cod.service = 0b00000001101; // 0b00000001101 = serial port, 0b00000010000 = generic
  esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

  return Err; }

// #endif // WITH_BT_SPP

// ========================================================================================================
