
#include "publisher.cpp"
#include <Adafruit_BNO08x.h>
#include <frost_interfaces/msg/imu.h>

class IMUPub : Publisher {

public:
  struct euler_t {
    float yaw;
    float pitch;
    float roll;
  } ypr;

  void setReports(void) {
    Serial.println("Setting desired reports");
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
      Serial.println("Could not enable linear acceleration");
    }
    if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
    }
  }

  void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

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

  void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
  }

  void setup(rcl_node_t node) {

    RCCHECK(rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, IMU), "imu_data"));
  }

  void imu_setup() {

    if (!bno08x.begin_I2C()) {
      Serial.println("Failed to find BNO08x chip");
    } else {
      Serial.println("BNO08x Found!");
      setReports();
      Serial.println("Reading events");
    }
  }

  float returnYaw(){return ypr.yaw + 180.00;}
  double returnVel(){return velocity;}

  void calculate_velocity(){     //Could make function with pointers so it can calculate all velocities
    if(!first_time){
      prev_time = micros();
      prev_accel = linear_accel_x;
      first_time = true;
    }
    else{
      velocity += (prev_accel + linear_accel_x) * 0.50 * (micros() - prev_time) * (10^-6);
      prev_time = micros();
      prev_accel = linear_accel_x;
    }
  }

  void imu_update() {
    if (bno08x.wasReset()) {
      setReports();
    }
    if (!bno08x.getSensorEvent(&sensorValue)) {
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
  long report_interval = 5000;
  float linear_accel_x;
  bool first_time = false;
  double velocity = 0;
  float prev_accel;
  unsigned long prev_time;

  frost_interfaces__msg__IMU msg;
};