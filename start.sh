#!/bin/bash

echo -e "\e[38;5;6m  /$$$$$$            /$$   /$$  /$$$$$$   /$$$$$$  /$$$$$$$   /$$$$$$ \e[0m"
echo -e "\e[38;5;6m /$$__  $$          | $$  | $$ /$$__  $$ /$$__  $$| $$__  $$ /$$__  $$\e[0m"
echo -e "\e[38;5;6m| $$  \__/  /$$$$$$ | $$  | $$| $$  \__/| $$  \ $$| $$  \ $$| $$  \__/\e[0m"
echo -e "\e[38;5;6m| $$       /$$__  $$| $$  | $$| $$ /$$$$| $$$$$$$$| $$$$$$$/|  $$$$$$ \e[0m"
echo -e "\e[38;5;6m| $$      | $$  \ $$| $$  | $$| $$|_  $$| $$__  $$| $$__  $$ \____  $$\e[0m"
echo -e "\e[38;5;6m| $$    $$| $$  | $$| $$  | $$| $$  \ $$| $$  | $$| $$  \ $$ /$$  \ $$\e[0m"
echo -e "\e[38;5;6m|  $$$$$$/|  $$$$$$/|  $$$$$$/|  $$$$$$/| $$  | $$| $$  | $$|  $$$$$$/\e[0m"
echo -e "\e[38;5;6m \______/  \______/  \______/  \______/ |__/  |__/|__/  |__/ \______/ \e[0m"
echo ""
echo -e "\e[38;5;6mBYU FROST LAB - COOPERATIVE UNDERWATER GROUP OF AUTONOMOUS ROBOTIC SYSTEMS\e[0m"
echo ""

cd ~/microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
sleep 5

cd ~/ros2_ws
source install/setup.bash
ros2 launch frost_uuv frost_uuv_launch.py

killall micro_ros_agent
wait
