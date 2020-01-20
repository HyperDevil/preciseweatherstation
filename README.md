# Arduino based weatherstation

## Requirements for building:
* high precision outdoor temperature 0.1 max deviation
* high precision outdoor relative humidity 1.5% max deviation
* high precision barometric pressure max 0.5 hPa deviation
* outdoor unit is wireless 868 mhz (EU) with CRC (possibly encryption)
* possible to add more sensors later (i2c is possibly the easiest option)

## Parts:
* SHT-35D Temperature / Humidity sensor by Sensirion with IP67 cap (for outdoor)
* SHT-31D Temperature / Humidity sensor by Sensirion (for indoor)
* BMP388 Temperature / Barometric pressure sensor by Bosch
* Atmel 328P chip (or complete Arduino)
* Prototyping boards
* RFM69HCW Transceiver Radio
* 5v power supply (battery, psu, solar...)
* Soldering skills incl. soldering iron, tin, sponge
* Arduino IDE and a way to program the 328P (directly on arduino, ZIF TX/RX, FTDI etc.)
* Different color wiring 22 AWG minimum
* Housing for outdoor and indoor (can 3d print but should be white and ABS outdoor)

## Software:
* Arduino IDE
* Libraries: 
 * https://github.com/adafruit/Adafruit_SHT31
 * https://github.com/adafruit/Adafruit_BMP3XX
 * https://github.com/PaulStoffregen/RadioHead

You can get the sensors from multiple vendors, some already use breakout boards which will make the 
build easier. Make sure the outdoor sensors are covered so they dont get wet. Always use i2c.
