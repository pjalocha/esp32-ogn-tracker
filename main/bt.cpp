
// ~/esp-idf/components/bt/bluedroid/api/include/esp_spp_api.h
// esp_err_t esp_spp_write(uint32_t handle, int len, uint8_t *p_data);

#include "hal.h"

#include "format.h"
#include "fifo.h"

#ifdef WITH_BT_SPP                            // classic BT

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t  sec_mask     = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave   = ESP_SPP_ROLE_SLAVE;

static FIFO<char,   1024> BT_SPP_TxFIFO;      // buffer for console output to be sent over BT
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
{ switch(Event)
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
  switch(Event)     // event numbers are in esp-idf/components/bt/bluedroid/api/include/api/esp_gap_bt_api.h
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
  BTconf.mode = ESP_BT_MODE_CLASSIC_BT;
  // Err = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  Err = esp_bt_controller_init(&BTconf); if(Err!=ESP_OK) return Err;
  Err = esp_bt_controller_enable((esp_bt_mode_t)BTconf.mode); if(Err!=ESP_OK) return Err;   // mode must be same as in BTconf
  // Err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT); if(Err!=ESP_OK) return Err;
  Err = esp_bluedroid_init(); if(Err!=ESP_OK) return Err;                                   // init the BT stack
  Err = esp_bluedroid_enable(); if(Err!=ESP_OK) return Err;                                 // enable the BT stack
  Err = esp_bt_gap_register_callback(esp_bt_gap_cb); if(Err!=ESP_OK) return Err;
  Err = esp_spp_register_callback(esp_spp_cb); if(Err!=ESP_OK) return Err;
  Err = esp_spp_init(esp_spp_mode); if(Err!=ESP_OK) return Err;

  // Set default parameters for Secure Simple Pairing
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

#endif // WITH_BT_SPP

#ifdef WITH_BLE_SPP                            // BLE BT

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"

#define ESP_SPP_APP_ID 0x56
#define SPP_SVC_INST_ID   0  // the instance id of the service

#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (1024)

// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;

// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

static esp_ble_adv_params_t spp_adv_params =
{ .adv_int_min        = 0x20,
  .adv_int_max        = 0x40,
  .adv_type           = ADV_TYPE_IND,
  .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
  .channel_map        = ADV_CHNL_ALL,
  .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

enum
{ SPP_IDX_SVC,

  SPP_IDX_SPP_DATA_RECV_CHAR,
  SPP_IDX_SPP_DATA_RECV_VAL,

  SPP_IDX_SPP_DATA_NOTIFY_CHAR,
  SPP_IDX_SPP_DATA_NTY_VAL,
  SPP_IDX_SPP_DATA_NTF_CFG,

  SPP_IDX_SPP_COMMAND_CHAR,
  SPP_IDX_SPP_COMMAND_VAL,

  SPP_IDX_SPP_STATUS_CHAR,
  SPP_IDX_SPP_STATUS_VAL,
  SPP_IDX_SPP_STATUS_CFG,

  SPP_IDX_NB // the number of attribute to be added to the service database
};

static uint16_t spp_handle_table[SPP_IDX_NB];

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

// SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

// SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

// SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

// SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
  // SPP -  Service Declaration
  [SPP_IDX_SVC]                       =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
  sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

  // SPP -  data receive characteristic Declaration
  [SPP_IDX_SPP_DATA_RECV_CHAR]            =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
  CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

  // SPP -  data receive characteristic Value
  [SPP_IDX_SPP_DATA_RECV_VAL]                 =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
  SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

  // SPP -  data notify characteristic Declaration
  [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
  CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

  // SPP -  data notify characteristic Value
  [SPP_IDX_SPP_DATA_NTY_VAL]   =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
  SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

  // SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
  [SPP_IDX_SPP_DATA_NTF_CFG]         =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
  sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

  // SPP -  command characteristic Declaration
  [SPP_IDX_SPP_COMMAND_CHAR]            =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
  CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

  // SPP -  command characteristic Value
  [SPP_IDX_SPP_COMMAND_VAL]                 =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
  SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

  // SPP -  status characteristic Declaration
  [SPP_IDX_SPP_STATUS_CHAR]            =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
  CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

  // SPP -  status characteristic Value
  [SPP_IDX_SPP_STATUS_VAL]                 =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
  SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

  // SPP -  status characteristic - Client Characteristic Configuration Descriptor
  [SPP_IDX_SPP_STATUS_CFG]         =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
  sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

} ;

static FIFO<char,   1024> BT_SPP_TxFIFO;      // buffer for console output to be sent over BT
static FIFO<uint8_t, 256> BT_SPP_RxFIFO;      // buffer for BT data to be send to the console
static uint32_t        BT_SPP_Conn = 0;       // BT incoming connection handle
static uint32_t        BT_SPP_TxCong = 0;     // congestion control

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
  // esp_err_t Ret=esp_spp_write(BT_SPP_Conn, Len, (uint8_t *)Data); // write the block to the BT
  // if(Ret!=ESP_OK) return 0;                                       // if an error then give up
  BT_SPP_TxFIFO.flushReadBlock(Len);                              // remove the transmitted block from the FIFO
  return Len; }                                                   // return number of transmitted bytes

static uint8_t spp_adv_data_len = 23;
static uint8_t spp_adv_data[25] =
{ 0x02, 0x01, 0x06,
  0x03, 0x03, 0xF0, 0xAB,
  0x0F, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x53, 0x50, 0x50, 0x5f, 0x53, 0x45, 0x52, 0x56, 0x45, 0x52, 0x00, 0x00 } ; // ESP_SPP_SERVER

// static esp_ble_adv_data_t spp_adv_data =
// { .include_name = true
// };

static void esp_ble_gatts_cb(esp_gatts_cb_event_t Event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *Param)
{ // esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)Param;
  switch(Event) // event numbers: esp-idf/components/bt/host/bluedroid/api/include/api/esp_gatts_api.h
  { case ESP_GATTS_REG_EVT:   // #0
    { // esp_ble_gap_set_device_name(Parameters.BTname);
      // esp_ble_gap_config_adv_data(&spp_adv_data);
      int NameLen = strlen(Parameters.BTname); if(NameLen>16) NameLen=16;
      memcpy(spp_adv_data+9, Parameters.BTname, NameLen); spp_adv_data[7]=NameLen+1; spp_adv_data_len=9+NameLen;
      esp_ble_gap_config_adv_data_raw(spp_adv_data, spp_adv_data_len);
      esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
      break; }
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:  // #22
      if(Param->add_attr_tab.status != ESP_GATT_OK) break;
      if(Param->add_attr_tab.num_handle != SPP_IDX_NB) break;
      memcpy(spp_handle_table, Param->add_attr_tab.handles, sizeof(spp_handle_table));
      esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
      break;
    case ESP_GATTS_START_EVT: // #12
      break;
    case ESP_GATTS_UNREG_EVT: // #6
      break;
    case ESP_GATTS_MTU_EVT:   // #4
      // spp_mtu_size = p_data->mtu.mtu;
      break;
    case ESP_GATTS_CONNECT_EVT: // #14
      break;
    case ESP_GATTS_DISCONNECT_EVT: // #15
      break;
    default:
      break;
  }

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "BLE_GATTS: Event ");
  Format_UnsDec(CONS_UART_Write, (uint32_t)Event);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
}

static void esp_ble_gap_cb(esp_gap_ble_cb_event_t Event, esp_ble_gap_cb_param_t *Param)
{
  switch(Event) // event numbers: esp-idf/components/bt/host/bluedroid/api/include/api/esp_gap_ble_api.h
  { case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
      esp_ble_gap_start_advertising(&spp_adv_params);
      break;
    case ESP_GAP_BLE_SEC_REQ_EVT: // #10
      break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: // #20
      break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: // #8
      break;
    default:
      break;
  }

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "BLE_GAP: Event ");
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

  if( (BT_SPP_TxCong==0) && ( (Byte=='\n') || (BT_SPP_TxFIFO.Full()>=64) ) )              // if no congestion and EOL or 64B waiting $
  { BT_SPP_TxPush(); }                                                                    // read a block from TxFIFO ad push it into$
}

int BT_SPP_Init(void)
{ esp_err_t Err=ESP_OK;
  if(Parameters.BTname[0]==0) return Err;

  esp_bt_controller_config_t BTconf = BT_CONTROLLER_INIT_CONFIG_DEFAULT();                  // the default mode is defined by the men$
  BTconf.mode = ESP_BT_MODE_BLE;
  // Err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  Err = esp_bt_controller_init(&BTconf); if(Err!=ESP_OK) return Err;
  Err = esp_bt_controller_enable((esp_bt_mode_t)BTconf.mode); if(Err!=ESP_OK) return Err;   // mode must be same as in BTconf
  Err = esp_bluedroid_init(); if(Err!=ESP_OK) return Err;                                   // init the BT stack
  Err = esp_bluedroid_enable(); if(Err!=ESP_OK) return Err;                                 // enable the BT stack
  Err = esp_ble_gap_register_callback(esp_ble_gap_cb); if(Err!=ESP_OK) return Err;
  Err = esp_ble_gatts_register_callback(esp_ble_gatts_cb); if(Err!=ESP_OK) return Err;
  Err = esp_ble_gatts_app_register(ESP_SPP_APP_ID); if(Err!=ESP_OK) return Err;

  // Set default parameters for Secure Simple Pairing
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE; // _IO;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

  // Set default parameters for Legacy Pairing: fixed PIN
  esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
  esp_bt_pin_code_t pin_code = { '0', '1', '2', '3' };
  esp_bt_gap_set_pin(pin_type, 4, pin_code);

  return Err; }

#endif // WITH_BLE_SPP

// ========================================================================================================
