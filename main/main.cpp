// ======================================================================================================

#include "freertos/FreeRTOS.h"    // FreeeRTOS
#include "freertos/task.h"

#include "hal.h"                  // Hardware Abstraction Layer

#include "rf.h"                   // RF control/transmission/reception task
#include "proc.h"                 // GPS/RF process taskk
#include "gps.h"                  // GPS control data acquisiton
#include "sens.h"
#include "ctrl.h"                 // Control task

extern "C"
void app_main(void)
{
    // printf("OGN Tracker on ESP32\n");

    CONS_Mutex = xSemaphoreCreateMutex();    // semaphore for sharing the writing to the console
    // I2C_Mutex  = xSemaphoreCreateMutex();    // semaphore for sharing the I2C bus

    NVS_Init();                              // initialize Non-Volatile-Storage in Flash and read the tracker parameters

    Parameters.setDefault(getUniqueAddress()); // set default parameter values
    if(Parameters.ReadFromNVS()!=ESP_OK)     // try to get parameters from NVS
    { Parameters.WriteToNVS(); }             // if did not work: try to save (default) parameters to NVS

    SPIFFS_Register();                       // initialize the file system in the Flash

    IO_Configuration();                      // initialize the GPIO/UART/I2C/SPI for Radio, GPS, OLED, Baro

#ifdef WITH_BT_SPP
    BT_SPP_Init();                           // start BT SPP
#endif

    xTaskCreate(vTaskRF,    "RF",    2048, 0, tskIDLE_PRIORITY+4, 0);
    xTaskCreate(vTaskPROC,  "PROC",  2048, 0, tskIDLE_PRIORITY+3, 0);
    xTaskCreate(vTaskGPS,   "GPS",   2048, 0, tskIDLE_PRIORITY+4, 0);
#if defined(WITH_BMP180) || defined(WITH_BMP280) || defined(BME280) || defined(WITH_MS5607)
    xTaskCreate(vTaskSENS,  "SENS",  2048, 0, tskIDLE_PRIORITY+4, 0);
#endif
    // xTaskCreate(vTaskCTRL,  "CTRL",  1536, 0, tskIDLE_PRIORITY+2, 0);
    vTaskCTRL(0); // run directly the CTRL task, instead of creating a separate one.

    // while(1)
    // { vTaskDelay(10000); }
}

