#ifndef HUMIDITY_PUB
#define HUMIDITY_PUB

#include "publisher.hpp"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <frost_interfaces/msg/humid.h>

class HumidityPub : Publisher {

public:

  void setup(rcl_node_t node);
  void publish();
  using Publisher::destroy;

private:

  frost_interfaces__msg__Humid msg;
  void humidity_calibrate();

};

#endif // HUMIDITY_PUB