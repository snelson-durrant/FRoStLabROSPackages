#!/bin/bash

cd ~/teensy_ws/sensor_test
pio run

cd .pio/build/teensy41
teensy_loader_cli --mcu=TEENSY41 -sv firmware.hex
# teensy_loader_cli --mcu=TEENSY41 -sv firmware.hex

screen /dev/ttyACM0 115200