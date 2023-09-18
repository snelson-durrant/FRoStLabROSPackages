#!/bin/bash

# sudo udevadm trigger
# sudo gpsd /dev/gps_data -F /var/run/gpsd.sock

echo -e "\e[38;5;6m _   _ _  _  ___ _    ___   ___  ___ _____      __  _   ___   __\e[0m"
echo -e "\e[38;5;6m| | | | \| |/ __| |  | __| |   \| _ \ __\ \    / / | | | \ \ / /  ()\e[0m"
echo -e "\e[38;5;6m| |_| | .\` | (__| |__| _|  | |) |   / _| \ \/\/ /  | |_| |\ V /      ()\e[0m"
echo -e "\e[38;5;6m \___/|_|\_|\___|____|___| |___/|_|_\___| \_/\_/    \___/  \_/   () \e[0m"
echo ""
echo -e "\e[38;5;6mUnmanned Navigationally-Cooperative Low-Expense Disposable Reusable Expeditionary Warfare Underwater Vehicle\e[0m"

echo ""
echo -e "\e[38;5;6m()\e[0m CONFIGURING MICROROS AGENT..."
cd ~/microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
sleep 2

echo ""
echo -e "\e[38;5;6m()\e[0m RECORDING ROS2 TOPICS..."
cd ~/ros2_ws
source install/setup.bash
cd ~/ros2_ws/bag_files
ros2 bag record nav_instructions echo_data modem_data imu_data gps_data depth_data leak_detected voltage humidity &
sleep 2

echo ""
echo -e "\e[38;5;6m()\e[0m LAUNCHING ROS2 PUBLISHERS AND SUBSCRIBERS..."
cd ~/ros2_ws
ros2 launch frost_uuv frost_uuv_launch.py

killall ros2
killall micro_ros_agent
