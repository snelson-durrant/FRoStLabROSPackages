#!/bin/bash

cd ~/teensy_ws/drew_uuv
pio run --target clean_microros
pio lib install
pio run

cd .pio/build/teensy41
teensy_loader_cli --mcu=TEENSY41 -sv firmware.hex