# ESP32 OGN-Tracker
OGN Tracker implementation on ESP32 devices.
It works with HELTEC and TTGO boards with sx1276 RF chip for 868/915MHz
The quickest board to run is the T-Beam from TTGO as it includes GPS, RF chip, battery circuit and holder, optionally as well a small OLED display. Yout to solder BMP280 or BME280 pressure/temperature/humidity sensor.

The initial code is written for and tested on HALTEC LoRa 32 module with sx1276 and 128x64 OLED display.
Most likely it can be easily ported to other ESP32 devices, as these are very flexible for the I/O assignement.
If you need to change the pins assigned to various periferials, see the top of the hal.cpp file.

To compile and flash the ESP32 board you need to install the ESP-IDF v4.0, start with:

```
git clone -b v4.0 --recursive https://github.com/espressif/esp-idf.git
```
then run *install.sh* and *export.sh* so you are ready to run *make* in the project directory.

If you are doing this on a Raspberry PI you may need this trick to install the ESP-IDF
```
pip install --upgrade virtualenv==16.7.9
```

If you want to use the OLED display with the U8g2 library you need to install it from the project directory:

```
mkdir components
cd components
git clone https://github.com/olikraus/u8g2.git
cd ..
```

For the original HELTEC board you need to wire the UART GPS follow the pins defined in the hal.cpp
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

