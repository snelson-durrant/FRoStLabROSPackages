#ifndef VOLTAGE_PUB
#define VOLTAGE_PUB

#include "publisher.hpp"
#include <Adafruit_INA260.h>
#include <frost_interfaces/msg/volt.h>
#include <Wire.h>

class VoltagePub : Publisher {

public:
  void setup(rcl_node_t node);
  void publish();
  using Publisher::destroy;

private:
  frost_interfaces__msg__Volt msg;

};

#endif // VOLTAGE_PUB