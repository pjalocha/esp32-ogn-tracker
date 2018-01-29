// ======================================================================================================

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "esp_system.h"
// #include "nvs_flash.h"

#include "hal.h"

#include "rf.h"
#include "proc.h"
#include "gps.h"
#include "ctrl.h"

extern "C"
void app_main(void)
{
    // printf("OGN Tracker on ESP32\n");

    NVS_Init();                       // initialize Non-Volatile-Storage in Flash and read the tracker parameters
    SPIFFS_Register();                // initialize the file system in the Flash

    // BT_SPP_Init();

    IO_Configuration();               // initialize the GPIO/UART/I2C/SPI for Radio, GPS, OLED, Baro

    xTaskCreate(vTaskRF,    "RF",    2048, 0, tskIDLE_PRIORITY+4, 0);
    xTaskCreate(vTaskPROC,  "PROC",  2048, 0, tskIDLE_PRIORITY+3, 0);
    xTaskCreate(vTaskGPS,   "GPS",   2048, 0, tskIDLE_PRIORITY+4, 0);
    // xTaskCreate(vTaskCTRL,  "CTRL",  1536, 0, tskIDLE_PRIORITY+2, 0);
    vTaskCTRL(0); // run directly the CTRL task, instead of creating a separate one.

    // while(1)
    // { vTaskDelay(10000); }
}

