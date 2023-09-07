#!/bin/bash

cd ~/teensy_ws/sensor_test
pio run

cd .pio/build/teensy41
echo "Press Program Button on Teensy..."
teensy_loader_cli --mcu=TEENSY41 -s firmware.hex
teensy_loader_cli --mcu=TEENSY41 -w firmware.hex
echo "Upload Complete"

screen /dev/ttyACM0 115200