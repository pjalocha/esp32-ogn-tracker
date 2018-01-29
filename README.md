# esp32-ogn-tracker
OGN Tracker implementation on ESP32 devices

The initial code is written for and tested on HALTEC LoRa 32 module with sx1276 and 128x64 OLED display.
Most likely it can be easily ported to other ESP32 devices, as these are very flexible for the I/O assignement.
If you need to change the pins assigned to various periferials, see the top of the hal.cpp file.

To compile and upload use the standard ESP32 toolchain and esp-idf

To wire the UART GPS follow the pins defined in the hal.cpp
```
                                  // wire colours for VK2828U GPS
#define PIN_GPS_TXD  GPIO_NUM_12  // green
#define PIN_GPS_RXD  GPIO_NUM_35  // blue
#define PIN_GPS_PPS  GPIO_NUM_34  // white
#define PIN_GPS_ENA  GPIO_NUM_13  // yellow -> well, I had a problem here, thus I tied the enable wire to 3.3V for the time being.
```
Note the yellow wire: put it to 3.3V to start with.
An attempt to control it from the CPU did not work, to be followed.

Note: I have seen GPSes where red was ground and black was power thus be carefull !

Note: If you use MTK GPS check the definitions in the hal.h accordingly if you want higher baud rates on the GPS UART

