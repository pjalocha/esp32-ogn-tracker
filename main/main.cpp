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

#ifdef WITH_WIFI
#include "wifi.h"                 // WIFI task
#endif

extern "C"
void app_main(void)
{
    // printf("OGN Tracker on ESP32\n");

    CONS_Mutex = xSemaphoreCreateMutex();    // semaphore for sharing the writing to the console
    I2C_Mutex  = xSemaphoreCreateMutex();    // semaphore for sharing the I2C bus

    NVS_Init();                              // initialize Non-Volatile-Storage in Flash and read the tracker parameters

    Parameters.setDefault(getUniqueAddress()); // set default parameter values
    if(Parameters.ReadFromNVS()!=ESP_OK)     // try to get parameters from NVS
    { Parameters.WriteToNVS(); }             // if did not work: try to save (default) parameters to NVS
#ifdef WITH_SPIFFS
    SPIFFS_Register();                       // initialize the file system in the Flash
#endif
    IO_Configuration();                      // initialize the GPIO/UART/I2C/SPI for Radio, GPS, OLED, Baro

#ifdef WITH_SD
    if(SD_isMounted())                       // if SD card succesfully mounted at startup
    { Parameters.SaveToFlash=0;
      if(Parameters.ReadFromFile("/sdcard/TRACKER.CFG")>0)    // try to read parameters from the TRACKER.CFG file
      { if(Parameters.SaveToFlash) Parameters.WriteToNVS(); } // if succesfull and SaveToFlash==1 then save them to flash
// #ifdef WITH_SPIFFS
//       SPIFFSlog_CopyToSD();                                   // copy all flash log files to the SD card
// #endif
    }
#endif

    CONS_UART_SetBaudrate(Parameters.CONbaud);

#ifdef WITH_BT_SPP
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

    xTaskCreate(vTaskRF,    "RF",    2048, 0, tskIDLE_PRIORITY+4, 0);
#ifdef WITH_LOG
    xTaskCreate(vTaskLOG ,  "LOG",   4096, 0, tskIDLE_PRIORITY+1, 0);
#endif
#ifdef WITH_SDLOG
    Log_Mutex = xSemaphoreCreateMutex();
    xTaskCreate(vTaskSDLOG, "SDLOG", 4096, 0, tskIDLE_PRIORITY+1, 0);
#endif
    xTaskCreate(vTaskPROC,  "PROC",  2048, 0, tskIDLE_PRIORITY+3, 0);
    xTaskCreate(vTaskGPS,   "GPS",   2048, 0, tskIDLE_PRIORITY+4, 0);
#if defined(WITH_BMP180) || defined(WITH_BMP280) || defined(WITH_BME280) || defined(WITH_MS5607) || defined(WITH_MS5611)
    xTaskCreate(vTaskSENS,  "SENS",  2048, 0, tskIDLE_PRIORITY+4, 0);
#endif
#ifdef WITH_BMX055
    xTaskCreate(vTaskIMU,   "IMU",   2048, 0, tskIDLE_PRIORITY+4, 0);
#endif
#ifdef WITH_KNOB
    xTaskCreate(vTaskKNOB,  "KNOB",  2048, 0, tskIDLE_PRIORITY+3, 0);
#endif
#ifdef WITH_AERO
    xTaskCreate(vTaskAERO,  "AERO",  2048, 0, tskIDLE_PRIORITY+4, 0);
#endif
#ifdef WITH_WIFI
    xTaskCreate(vTaskWIFI,  "WIFI",  4096, 0, tskIDLE_PRIORITY+2, 0);
#endif
#if defined(WITH_OLED) || defined(WITH_U8G2_OLED) || defined(WITH_ST7789) || defined(WITH_ILI9341)
    xTaskCreate(vTaskDISP,  "DISP",  2048, 0, tskIDLE_PRIORITY+2, 0);
#endif
#ifdef WITH_SOUND
    xTaskCreate(vTaskSOUND, "SOUND", 2048, 0, tskIDLE_PRIORITY+3, 0);
#endif
    // xTaskCreate(vTaskCTRL,  "CTRL",  1536, 0, tskIDLE_PRIORITY+2, 0);
    vTaskCTRL(0); // run directly the CTRL task, instead of creating a separate one.

    // while(1)
    // { vTaskDelay(10000); }
}

