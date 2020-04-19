#ifndef __SENS_H__
#define __SENS_H__

#ifdef WITH_BMP180
#include "bmp180.h"
extern BMP180   Baro;                       // BMP180 barometer sensor
#endif

#ifdef WITH_BMP280
#include "bmp280.h"
extern BMP280   Baro;                       // BMP280 barometer sensor
#endif

#ifdef WITH_BME280
#include "bme280.h"
extern BME280   Baro;                       // BMP280 barometer sensor with humidity sensor
#endif

#ifdef WITH_MS5607
#include "ms5607.h"
extern MS5607   Baro;                       // MS5607 barometer sensor
#endif

#ifdef WITH_MS5611
#include "ms5611.h"
extern MS5611   Baro;                       // MS5611 barometer sensor
#endif


#ifdef __cplusplus
  extern "C"
#endif
void vTaskSENS(void* pvParameters);

#endif // __SENS_H__
