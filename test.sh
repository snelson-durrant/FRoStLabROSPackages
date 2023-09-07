#!/bin/bash

echo ""
echo "CONFIGURING MICROROS AGENT..."
cd ~/microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 &
sleep 2

echo ""
echo "RUNNING SERVO TESTS..."
cd ~/ros2_ws
source install/setup.bash
ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 45, servo2: 45, servo3: 45, thruster: 0}'
ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 90, servo2: 90, servo3: 90, thruster: 0}'
ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 135, servo2: 135, servo3: 135, thruster: 0}'
ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 90, servo2: 90, servo3: 90, thruster: 0}'

echo ""
echo "RUNNING THRUSTER TESTS..."
# ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 90, servo2: 90, servo3: 90, thruster: 20}'
# ros2 topic pub -1 /nav_instructions frost_interfaces/msg/Nav '{servo1: 90, servo2: 90, servo3: 90, thruster: 0}'

killall ros2
killall micro_ros_agent
