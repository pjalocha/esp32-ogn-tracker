# ESP32 OGN-Tracker
OGN Tracker implementation on ESP32 devices.
It works with HELTEC and TTGO boards with sx1276 RF chip for 868/915MHz
The quickest board to run is the T-Beam from TTGO as it includes GPS, RF chip, battery circuit and holder, optionally as well a small OLED display. Yout to solder BMP280 or BME280 pressure/temperature/humidity sensor.

The initial code is written for and tested on HALTEC LoRa 32 module with sx1276 and 128x64 OLED display.
Most likely it can be easily ported to other ESP32 devices, as these are very flexible for the I/O assignement.
If you need to change the pins assigned to various periferials, see the top of the hal.cpp file.

## To compile the code: install the ESP-IDF

To compile and flash the ESP32 board you need to install the ESP-IDF v4.0
Start with:

```
cd
git clone -b v4.0 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
```
If you are doing this on a Raspberry PI and see error messages about *virtualenv* you may need the following:
```
pip install --upgrade virtualenv==16.7.9
```

Then, in order to be able to *make* projects you need to run
```
source ~/esp-idf/export.sh
```

To get the OGN-Tracker source code from this github repository:
```
cd
git clone https://github.com/pjalocha/esp32-ogn-tracker.git
cd ogn32-ogn-tracker
```

If you want to use the OLED display with the U8g2 library you need to install it from the project directory:

```
mkdir components
cd components
git clone https://github.com/olikraus/u8g2.git
cd ..
```

To choose the configuration file for the T-Beam
```
cp T-Beam.cfg main/config.h
```

To compile and flash the board
```
make
make flash
```

## Wiring the GPS

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

Note: If you use MTK GPS check the definitions in the hal.cpp accordingly if you want higher baud rates on the GPS UART

