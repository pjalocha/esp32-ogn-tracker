// ======================================================================================================

#include "freertos/FreeRTOS.h"    // FreeeRTOS
#include "freertos/task.h"

#include "hal.h"                  // Hardware Abstraction Layer

#include "rf.h"                   // RF control/transmission/reception task
#include "proc.h"                 // GPS/RF process taskk
#include "gps.h"                  // GPS control data acquisiton
#include "sens.h"
#include "imu.h"
#include "ctrl.h"                 // Control task
#include "log.h"                  // Data logging task
#include "knob.h"                 // potentiometer as rotary encoder
#include "sound.h"                // sounds, warnings, alarms
#include "disp.h"

#ifdef WITH_SDLOG
#include "sdlog.h"
#endif

#ifdef WITH_AERO
#include "aero.h"
#endif

#ifdef WITH_STRATUX
#include "stratux.h"
#endif

#if defined(WITH_BT_SPP) || defined(WITH_BLE_SPP)
#include "bt.h"
#endif

#ifdef WITH_AP
#include "ap.h"
#endif

#ifdef WITH_APRS
#include "aprs.h"                 // APRS task
#endif

extern "C"
void app_main(void)
{
    // printf("OGN Tracker on ESP32\n");

    CONS_UART_Init();
    CONS_Mutex = xSemaphoreCreateMutex();    // semaphore for sharing the writing to the console
    I2C_Mutex  = xSemaphoreCreateMutex();    // semaphore for sharing the I2C bus
#ifdef WITH_AXP
    // AXP_Mutex  = xSemaphoreCreateMutex();    // semaphore for sharing the AXP power controller
#endif

    NVS_Init();                              // initialize Non-Volatile-Storage in Flash and read the tracker parameters

    Parameters.setDefault(getUniqueAddress()); // set default parameter values
    if(Parameters.ReadFromNVS()!=ESP_OK)     // try to get parameters from NVS
    { Parameters.WriteToNVS(); }             // if did not work: try to save (default) parameters to NVS

#ifdef WITH_SPIFFS
    SPIFFS_Register();                       // initialize the file system in the Flash
#endif
    IO_Configuration();                      // initialize the GPIO/UART/I2C/SPI for Radio, GPS, OLED, Baro

#ifdef WITH_AXP
    uint8_t PwrStatus = AXP.readStatus();  // bit #0 = 1:by ext. power, 0:by power-on-button
    bool ExtPwrON = PwrStatus&1;
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, ExtPwrON ? "Power-ON by ext. power\n":"Power-ON by Power-ON button\n");
    if(ExtPwrON)
    { Format_String(CONS_UART_Write, "Power-ON by ext. power\n");
      if(!Parameters.PowerON)
      { AXP.setLED(4);
        vTaskDelay(500);
        AXP.setPowerOutput(AXP.OUT_LDO2,  0); // turn off RFM power
        AXP.setPowerOutput(AXP.OUT_LDO3,  0); // turn off GPS power
        AXP.setPowerOutput(AXP.OUT_DCDC1, 0);
        AXP.ShutDown();
        vTaskDelay(1000); }
    }
    else
    { Format_String(CONS_UART_Write, "Power-ON button\n");
      if(!Parameters.PowerON) { Parameters.PowerON=1; Parameters.WriteToNVS(); }
    }
    xSemaphoreGive(CONS_Mutex);
#endif
#ifdef WITH_SD
    if(SD_isMounted())                       // if SD card succesfully mounted at startup
    { Parameters.SaveToFlash=0;
      if(Parameters.ReadFromFile("/sdcard/TRACKER.CFG")>0)    // try to read parameters from the TRACKER.CFG file
      { if(Parameters.SaveToFlash) Parameters.WriteToNVS(); } // if succesfull and SaveToFlash==1 then save them to flash
// #ifdef WITH_SPIFFS
//       FlashLog_CopyToSD();                                   // copy all flash log files to the SD card
// #endif
    }
#endif

    CONS_UART_SetBaudrate(Parameters.CONbaud);

#ifdef WITH_LORAWAN
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Parameters.AppKey:");
    Format_HexBytes(CONS_UART_Write, Parameters.AppKey, 16);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    WANdev.Reset(getUniqueID(), Parameters.AppKey);     // set default LoRaWAN config.
    if(WANdev.ReadFromNVS()!=ESP_OK)                    // if can't read the LoRaWAN setup from NVS
    { WANdev.WriteToNVS(); }                            // then store the default
    if(Parameters.hasAppKey())
    { if(!Parameters.sameAppKey(WANdev.AppKey))         // if LoRaWAN key different from the one in Parameters
      { WANdev.Reset(getUniqueID(), Parameters.AppKey); // then reset LoRaWAN to this key
        WANdev.WriteToNVS();                            // and save LoRaWAN config. to NVS
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "LoRaWAN: AppKey <- ");
        Format_HexBytes(CONS_UART_Write, Parameters.AppKey, 16);
        // Format_String(CONS_UART_Write, " => ");
        // Format_SignDec(CONS_UART_Write, Err);
        Format_String(CONS_UART_Write, "\n");
        xSemaphoreGive(CONS_Mutex);
      }
      Parameters.clrAppKey();                           // clear the AppKey in the Parameters and save it to Flash
      Parameters.WriteToNVS(); }
    // WANdev.Disconnect();                                // restart with network join-request/accept at each restart

#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "LoRaWAN: AppEUI:");
    Format_Hex(CONS_UART_Write, WANdev.AppEUI);
    Format_String(CONS_UART_Write, " DevEUI:");
    Format_Hex(CONS_UART_Write, WANdev.DevEUI);
    Format_String(CONS_UART_Write, " DevNonce:");
    Format_Hex(CONS_UART_Write, WANdev.DevNonce);
    Format_String(CONS_UART_Write, " State:");
    Format_Hex(CONS_UART_Write, WANdev.State);
    Format_String(CONS_UART_Write, " AppKey:");
    Format_HexBytes(CONS_UART_Write, WANdev.AppKey, 16);
    Format_String(CONS_UART_Write, "\n");
    Format_String(CONS_UART_Write, "Parameters.AppKey:");
    Format_HexBytes(CONS_UART_Write, Parameters.AppKey, 16);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
#endif

#ifdef WITH_AP
#ifdef WITH_AP_BUTTON
    bool StartAP = Button_isPressed() && Parameters.APname[0]; // start WiFi AP when button pressed during startup and APname non-empty
#else
    bool StartAP = Parameters.APname[0]; // start WiFi AP when APname non-empty
#endif
#else  // WITH_AP
    const bool StartAP=0;
#endif // WITH_AP

#if defined(WITH_BT_SPP) || defined(WITH_BLE_SPP)
    if(!StartAP)
    { int32_t Err=BT_SPP_Init();                // start BT SPP
// #ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "BT_SPP_Init() => ");
      Format_SignDec(CONS_UART_Write, Err);
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex);
// #endif
    }
#endif

#ifdef WITH_SDLOG
    Log_Mutex = xSemaphoreCreateMutex();
    xTaskCreate(vTaskSDLOG, "SDLOG", 4000, 0, tskIDLE_PRIORITY+1, 0);
#endif

#ifdef WITH_LOG
    xTaskCreate(vTaskLOG ,  "LOG",   4500, 0, tskIDLE_PRIORITY+1, 0);
#endif

    xTaskCreate(vTaskRF,    "RF",    2000, 0, tskIDLE_PRIORITY+5, 0);
    xTaskCreate(vTaskPROC,  "PROC",  2000, 0, tskIDLE_PRIORITY+3, 0);

    xTaskCreate(vTaskGPS,   "GPS",   2000, 0, tskIDLE_PRIORITY+4, 0);
#if defined(WITH_BMP180) || defined(WITH_BMP280) || defined(WITH_BME280) || defined(WITH_MS5607) || defined(WITH_MS5611)
    xTaskCreate(vTaskSENS,  "SENS",  2000, 0, tskIDLE_PRIORITY+4, 0);
#endif
#ifdef WITH_BMX055
    xTaskCreate(vTaskIMU,   "IMU",   2000, 0, tskIDLE_PRIORITY+4, 0);
#endif

#ifdef WITH_KNOB
    xTaskCreate(vTaskKNOB,  "KNOB",  2000, 0, tskIDLE_PRIORITY+3, 0);
#endif
#ifdef WITH_AERO
    xTaskCreate(vTaskAERO,  "AERO",  2000, 0, tskIDLE_PRIORITY+3, 0);
#endif
#ifdef WITH_STRATUX
    xTaskCreate(vTaskSTX,  "STX",  4000, 0, tskIDLE_PRIORITY+3, 0);
#endif

#ifdef WITH_AP
    if(StartAP)
      xTaskCreate(vTaskAP,  "AP",  3000, 0, tskIDLE_PRIORITY+3, 0);
#endif
#ifdef WITH_APRS
    if(!StartAP)
      xTaskCreate(vTaskAPRS,  "APRS",  4000, 0, tskIDLE_PRIORITY+2, 0);
#endif
#if defined(WITH_OLED) || defined(WITH_U8G2_OLED) || defined(WITH_ST7789) || defined(WITH_ILI9341)
    xTaskCreate(vTaskDISP,  "DISP",  2000, 0, tskIDLE_PRIORITY+2, 0);
#endif
#ifdef WITH_SOUND
    xTaskCreate(vTaskSOUND, "SOUND", 2000, 0, tskIDLE_PRIORITY+3, 0);
#endif
    xTaskCreate(vTaskTICK , "TICK",  1500, 0, tskIDLE_PRIORITY+3, 0);
    // xTaskCreate(vTaskCTRL,  "CTRL",  1536, 0, tskIDLE_PRIORITY+2, 0);
    vTaskCTRL(0); // run directly the CTRL task, instead of creating a separate one.

    // while(1)
    // { vTaskDelay(10000); }
}

