#include <MS5837.h>
#include <Wire.h>
#include <frost_interfaces/msg/depth.h>
#include <publisher.cpp>
#include <std_msgs/msg/float64.h>

class PressurePub : Publisher {

public:
  void setup(rcl_node_t node) {

    Wire2.begin();
    pressure_sensor.init();

    pressure_sensor.setFluidDensity(997);

    RCCHECK(rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Depth),
        "depth_data"));
  }

  void publish() {

    pressure_sensor.read();
    msg.pressure = pressure_sensor.pressure();
    msg.depth = pressure_sensor.depth();
    msg.temperature = pressure_sensor.temperature();
    msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }

  using Publisher::destroy;

private:
  float pressure;
  float depth;
  float temperature;
  MS5837 pressure_sensor;

  frost_interfaces__msg__Depth msg;
};