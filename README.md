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
sudo apt-get install libffi-dev
pip install --upgrade virtualenv==16.7.9
```

If you see errors about openssl/opensslv.h then you need:
```
sudo apt-get install libssl-dev
```

Then, in order to be able to *make* projects you need to run
```
source ~/esp-idf/export.sh
```

To get the OGN-Tracker source code from this github repository:
```
cd
git clone https://github.com/pjalocha/esp32-ogn-tracker.git
cd esp32-ogn-tracker
```

If you want to use the OLED display with the U8g2 library you need to install it in *components* in the project directory:

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

To see the console output and thus to check if the ESP32 is alive
```
minicom
```
For minicom setup use 115200bps /dev/ttyUSB0 serial port and turn hardware and software handshake OFF. It is important, otherwise if you type something it won't be sent to the ESP32.
Press *Ctrl-C* to list the internal state and parameters. To change parameters, use $POGNS like this:
```
$POGNS,AcftType=1,Pilot=YourName
```

## Wiring the GPS

Wiring is fairly flexible, as the ESP32 can easily redefine the I/O signals. You need to avoid pins which are already used: the list normally comes with every ESP32 module. You can choose the pins after their placement on the board, for example such, that all wires are soldered on one side. Wiring for various modules has been chosen but if needed can easily be changed.

### HELTEC v1 and TTGO boards without GPS

For the original HELTEC board and similar TTGO boards without the GPS you need to wire the UART GPS follow the pins defined in the hal.cpp
```
                                  // wire colours for VK2828U GPS
#define PIN_GPS_TXD  GPIO_NUM_12  // green
#define PIN_GPS_RXD  GPIO_NUM_35  // blue
#define PIN_GPS_PPS  GPIO_NUM_34  // white
```

Note: I have seen GPSes where red was ground and black was power thus be carefull !

Note: If you use an MTK GPS change the definitions in the config.h accordingly.

### HELTEC v2

RXD and PPS need to be relocated because now they are connected to DIO lines of the RF module, thus they were relocated to pins 39 and 38.

### TTGO T-Beam

GPS is already wired and the definition is in the hal.cpp.

## Wiring I2C (pressure sensors and/or OLED displays)

For devices with an OLED screen the I2C pins are already defined and you should follow these, as there is one common I2C bus in use.
For devices without we still use the same I/O pins.

### HELTEC v1/2 and similar TTGO boards

```
#define PIN_I2C_SCL GPIO_NUM_15   // SCL pin
#define PIN_I2C_SDA GPIO_NUM_4    // SDA pin
```

### TTGO T-Beam
```
#define PIN_I2C_SCL GPIO_NUM_22   // SCL pin => this way the pin pattern fits the BMP280 module
#define PIN_I2C_SDA GPIO_NUM_21   // SDA pin
```

I2C pins are arranged such that they are next to GND and 3.3V and fit the pattern of most I2C modules
so for example to solder a pressure sensor like BMP280 just a 4-pin header is required.

## Wiring LCD display

![240x240 SPI LCD](/images/TrackerLCD.jpg)
A small, cheap LCD 240x240 screen has been tested with the TTGO T-Beam v1.0 boards and the wiring is as follows:

```
GND GND
VCC 3.3V
SDA 14
SCL 13
RES 33
DC   2
BLK 32
```
In this arrangement all signals are on one side of the board except for the VCC line

Note: the SDA/SCL naming suggest this display is interfaced with I2C but actually it is an SPI device with the CS being constantly active.

There is as well an LCD code for the M5 stack, which is already wired to the ESP32 thus no additional wiring is needed.
There you need however to wire other devices like the RF chip and pressure sensor.

## Console dialog and configuration

Use minicom and connect to /dev/ttyUSB0 (on Linux) for configuration set 115200bps and turn the hard- and soft-handshake OFF.
You shall see stream of NMEA sentences.
You can give the following commands:

Ctrl-C - lists internal state, internal log files and current parameter values.
Ctrl-L - list internal log files
Ctrl-V - hold the NMEA stream for 1 min
Ctrl-X - restarts the system

To set parameters send $POGNS with parameter name and value like
```
$POGNS,Pilot=John
```
the parameter value changes and all parameters are writen to internal flash thus they are preserved across system restart or repower.
To list all parameters with their values send Ctrl-C (software and hardware handshake must be OFF).

## BT interface

OGN-Tracker can be connected via Bluetooth from Android devices. The BT port is like a serial port and carriers the same data as the USB serial port.

