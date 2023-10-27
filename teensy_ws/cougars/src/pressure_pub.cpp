#include "publisher.hpp"
#include <MS5837.h>
#include <Wire.h>
#include <frost_interfaces/msg/depth.h>
#include <std_msgs/msg/float64.h>

#define AVG_COUNT 10
#define AVG_DEC 0.1

class PressurePub : Publisher {

public:
  void setup(rcl_node_t node) {

    RCCHECK(rclc_publisher_init_best_effort(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Depth),
        "depth_data"));
  }

  void pressure_setup() {

    pressure_sensor.init();
    pressure_calibrate();
  }

  void pressure_update() {

    pressure_sensor.read();
    msg.pressure = pressure_sensor.pressure(); // - pressure_at_zero_depth
    msg.depth = pressure_sensor.depth() - depth_error_at_zero_depth;
    msg.temperature = pressure_sensor.temperature();
  }

  void publish() {

    msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }

  using Publisher::destroy;

private:
  float pressure;
  float depth;
  float temperature;

  float pressure_at_zero_depth;
  float depth_error_at_zero_depth;

  MS5837 pressure_sensor;

  void pressure_calibrate() {

    pressure_sensor.setFluidDensity(997);

    float sum_pressure_at_zero_depth = 0;
    float sum_depth_error_at_zero_depth = 0;

    for (int i = 0; i < AVG_COUNT; i++) {
      pressure_sensor.read();
      sum_pressure_at_zero_depth += pressure_sensor.pressure();
      sum_depth_error_at_zero_depth += pressure_sensor.depth();
      // the read function takes ~ 40 ms according to documentation
    }

    pressure_at_zero_depth = sum_pressure_at_zero_depth * AVG_DEC;
    depth_error_at_zero_depth = sum_depth_error_at_zero_depth * AVG_DEC;
  }

  frost_interfaces__msg__Depth msg;
};