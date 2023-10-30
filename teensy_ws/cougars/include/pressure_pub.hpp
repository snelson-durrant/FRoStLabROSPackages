#ifndef PRESSURE_PUB
#define PRESSURE_PUB

#include "publisher.hpp"
#include <MS5837.h>
#include <Wire.h>
#include <frost_interfaces/msg/depth.h>
#include <std_msgs/msg/float64.h>

class PressurePub : Publisher {

public:
  void setup(rcl_node_t node);
  void pressure_setup();
  void pressure_update();
  void publish();
  using Publisher::destroy;

private:
  void pressure_calibrate();
  frost_interfaces__msg__Depth msg;

};

#endif // PRESSURE_PUB