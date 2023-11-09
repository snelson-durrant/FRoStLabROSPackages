#ifndef IMU_PUB_NEW
#define IMU_PUB_NEW

#include "publisher.hpp"
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <frost_interfaces/msg/imu.h>

class IMUPubNew : Publisher {

public:
  void setup(rcl_node_t node);
  void imu_setup();
  void imu_update();
  void publish();
  using Publisher::destroy;

private:
  frost_interfaces__msg__IMU msg;
};

#endif // IMU_PUB