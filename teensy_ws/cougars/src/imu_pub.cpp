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
 
  void setReports(void) {
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
      Serial5.println("Could not enable linear acceleration");
    }
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
      Serial5.println("Could not enable stabilized remote vector");
    }
  }

  void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr,
                         bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
  }

  void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector,
                           euler_t *ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i,
                      rotational_vector->j, rotational_vector->k, ypr, degrees);
  }

  void setup(rcl_node_t node) {

    RCCHECK(rclc_publisher_init_best_effort(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, IMU), "imu_data"));
  }

  void imu_setup() {

    Serial5.begin(9600);
    bno08x = Adafruit_BNO08x(-1);
    Wire2.begin();

    while (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire2, 0)) {
      Serial5.println("Failed to find BNO08x chip");
      Serial5.println(millis());
      delay(100);
    }
    setReports();
    bno08x.getSensorEvent(&sensorValue);
  }

  float returnYaw() { return ypr.yaw + 180.00; }
  double returnVel() { return velocity; }

  void calculate_velocity() { // Could make function with pointers so it can
                              // calculate all velocities
    if (n_time < 2) {
      prev_time_2 = prev_time_1;
      prev_time_1 = micros();
      prev_accel_2 = prev_accel_1;
      prev_accel_1 = linear_accel_x;
      n_time++;
    } else {
      unsigned long current_time = micros();
      float delta_time = (current_time - prev_time_2) * 1e-6;
      velocity += (prev_accel_2 + 4.0 * prev_accel_1 + linear_accel_x) *
                  delta_time / 6.0;
      // velocity += (prev_accel + linear_accel_x) * 0.50 * delta_time;
      // trapezoidal prev_time = micros(); prev_accel = linear_accel_x;
      prev_time_2 = prev_time_1;
      prev_time_1 = current_time;
      prev_accel_2 = prev_accel_1;
      prev_accel_1 = linear_accel_x;
      Serial5.println("Velocity: ");
      Serial5.println(velocity);
    }
  }

  void imu_update() {
    if (bno08x.wasReset()) {
      setReports();
    }

    sensorValue = bno08x.getHackSensorEvent();
    if (sensorValue.timestamp != 0) {
      switch (sensorValue.sensorId) {
      case SH2_LINEAR_ACCELERATION:
        msg.lin_accel_x = sensorValue.un.linearAcceleration.x;
        linear_accel_x = sensorValue.un.linearAcceleration.x;
        msg.lin_accel_y = sensorValue.un.linearAcceleration.y;
        msg.lin_accel_z = sensorValue.un.linearAcceleration.z;
        calculate_velocity();
        msg.mag_x = velocity;
        break;
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        msg.gyro_x = ypr.yaw;
        msg.gyro_y = ypr.pitch;
        msg.gyro_z = ypr.roll;
        break;
      }
    }
  }

  void publish() {

    msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }

  using Publisher::destroy;

private:
  Adafruit_BNO08x bno08x;
  sh2_SensorValue_t sensorValue;

  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long report_interval = 10000;
  float linear_accel_x;
  int n_time = 0;
  double velocity = 0.0;
  float prev_accel_1 = 0.0;
  float prev_accel_2 = 0.0;
  unsigned long prev_time_1 = 0.0;
  unsigned long prev_time_2 = 0.0;

  frost_interfaces__msg__IMU msg;
};