Parallel EEPROM Programmer
==========================

This is a fork of [erikvanzijst/eeprom](https://github.com/erikvanzijst/eeprom) that adds support for Arduino Mega 2650 allowing reading and writing of eeprom devices just usng a breadboard.


This repository contains:

* Arduino code
* Interactive Python based client to read and write to the EEPROM

## Installation and assembly

Install [platformio](https://platformio.org) to compile and upload the firmware
to the Arduino:

    $ platformio run --target upload

Wire up the Mega2560 to a breadboard following the pin mapping at the top of [src/arduino.cpp](src/arduino.cpp).

With the shield attached to the Uno and hooked up to a computer, fire up the
Python client. To write an image to the EEPROM:

```
$ source ./env/bin/activate
$ cat image.bin | ./eeprom.py load 
Loading 400 bytes into EEPROM...
100%
Complete.
$
```
