#!/bin/bash

cd ~/teensy_ws/drew_uuv
pio run --target clean_microros
pio lib install
pio run