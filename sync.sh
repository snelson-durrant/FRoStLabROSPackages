#!/bin/bash

# syncs changes made in the ros2 frost_interfaces package with the teensy_ws
rsync -avu --delete ~/ros2_ws/src/frost_interfaces ~/teensy_ws/drew_uuv/extra_packages
