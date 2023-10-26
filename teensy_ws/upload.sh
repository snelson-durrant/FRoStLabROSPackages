#!/bin/bash

cd ~/teensy_ws/cougars/.pio/build/teensy41
teensy_loader_cli --mcu=TEENSY41 -sv firmware.hex