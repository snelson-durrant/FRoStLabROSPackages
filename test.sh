#!/bin/bash

cd ~/microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000 &
sleep 5

echo ""
echo "TESTING SERVO COMMANDS..."
cd ~/ros2_ws
source install/setup.bash
ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 45, servo2: 45, servo3: 45, thruster: 0}'
ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 135, servo2: 135, servo3: 135, thruster: 0}'
ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 90, servo2: 90, servo3: 90, thruster: 0}'

echo ""
echo "TESTING THRUSTER COMMANDS..."
ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 90, servo2: 90, servo3: 90, thruster: 20}'
ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 90, servo2: 90, servo3: 90, thruster: 0}'

echo ""
echo "CALLING GPS SERVICE..."
ros2 service call /gps_service frost_interfaces/srv/GetGPS "{test: True}"

echo ""
echo "CALLING ECHO SERVICE..."
ros2 service call /echo_service frost_interfaces/srv/GetEcho "{test: True}"

echo ""
echo "READING IMU DATA..."
ros2 topic echo --once /imu_data

echo ""
echo "READING DEPTH DATA..."
ros2 topic echo --once /depth_data

echo ""
echo "READING LEAK DATA (TRIGGER LEAK SENSOR)..."
ros2 topic echo --once /leak_detected

echo ""
echo "READING HUMIDITY DATA (TRIGGER HUMIDITY SENSOR)..."
ros2 topic echo --once /humidity

echo ""
echo "READING VOLTAGE DATA (TRIGGER VOLTAGE SENSOR)..."
ros2 topic echo --once /voltage

echo ""
echo "TEST COMPLETE"

killall micro_ros_agent
wait
