#!/bin/bash

cd ~/teensy_ws/cougars
pio run --target clean_microros
pio lib install
pio run