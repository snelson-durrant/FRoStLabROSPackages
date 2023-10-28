#ifndef IMU_PUB
#define IMU_PUB

#include "publisher.hpp"
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <frost_interfaces/msg/imu.h>

class IMUPub : Publisher {

public:
  struct euler_t {
    float yaw;
    float pitch;
    float roll;
  } ypr;

  void setReports(void);
  void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr,
                         bool degrees = false);
  void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector,
                           euler_t *ypr, bool degrees = false);
  void setup(rcl_node_t node);
  void imu_setup();
  float returnYaw();
  double returnVel();
  void calculate_velocity();
  void imu_update();
  void publish();
  using Publisher::destroy;

private:
  frost_interfaces__msg__IMU msg;
};

#endif // IMU_PUB