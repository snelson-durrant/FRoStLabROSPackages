#include "publisher.cpp"
#include <Adafruit_BNO08x.h>
#include <frost_interfaces/msg/imu.h>

class IMUPub : Publisher {

public:
  void setReports(void) {

    Serial.println("Setting desired reports");
    if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
      Serial.println("Could not enable accelerometer");
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
      Serial.println("Could not enable gyroscope");
    }
    if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
      Serial.println("Could not enable magnetic field calibrated");
    }
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
      Serial.println("Could not enable linear acceleration");
    }
    if (!bno08x.enableReport(SH2_GRAVITY)) {
      Serial.println("Could not enable gravity vector");
    }
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      Serial.println("Could not enable rotation vector");
    }
    if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
      Serial.println("Could not enable geomagnetic rotation vector");
    }
    if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
      Serial.println("Could not enable raw accelerometer");
    }
    if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
      Serial.println("Could not enable raw gyroscope");
    }
    if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
      Serial.println("Could not enable raw magnetometer");
    }
  }

  void setup(rcl_node_t node) {

    if (!bno08x.begin_I2C()) {
      Serial.println("Failed to find BNO08x chip");
    } else {
      Serial.println("BNO08x Found!");
      setReports();
      Serial.println("Reading events");
    }

    RCCHECK(rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, IMU), "imu_data"));
  }

  void imu_update() {
    if (bno08x.wasReset()) {
      Serial.print("sensor was reset ");
      setReports();
    }
    if (!bno08x.getSensorEvent(&sensorValue)) {
      return;
    }

    switch (sensorValue.sensorId) {

    case SH2_ACCELEROMETER:
      msg.accel_x = sensorValue.un.accelerometer.x;
      msg.accel_y = sensorValue.un.accelerometer.y;
      msg.accel_z = sensorValue.un.accelerometer.z;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      msg.gyro_x = sensorValue.un.gyroscope.x;
      msg.gyro_y = sensorValue.un.gyroscope.y;
      msg.gyro_z = sensorValue.un.gyroscope.z;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      msg.mag_x = sensorValue.un.magneticField.x;
      msg.mag_y = sensorValue.un.magneticField.y;
      msg.mag_z = sensorValue.un.magneticField.z;
      break;
    case SH2_LINEAR_ACCELERATION:
      msg.lin_accel_x = sensorValue.un.linearAcceleration.x;
      msg.lin_accel_y = sensorValue.un.linearAcceleration.y;
      msg.lin_accel_z = sensorValue.un.linearAcceleration.z;
      break;
    case SH2_GRAVITY:
      msg.grav_x = sensorValue.un.gravity.x;
      msg.grav_y = sensorValue.un.gravity.y;
      msg.grav_z = sensorValue.un.gravity.z;
      break;
    case SH2_ROTATION_VECTOR:
      msg.rot_vec_i = sensorValue.un.rotationVector.i;
      msg.rot_vec_j = sensorValue.un.rotationVector.j;
      msg.rot_vec_k = sensorValue.un.rotationVector.k;
      break;
    case SH2_GEOMAGNETIC_ROTATION_VECTOR:
      msg.geomag_rot_vec_i = sensorValue.un.geoMagRotationVector.i;
      msg.geomag_rot_vec_j = sensorValue.un.geoMagRotationVector.j;
      msg.geomag_rot_vec_k = sensorValue.un.geoMagRotationVector.k;
      break;
    case SH2_RAW_ACCELEROMETER:
      msg.raw_accel_x = sensorValue.un.rawAccelerometer.x;
      msg.raw_accel_y = sensorValue.un.rawAccelerometer.y;
      msg.raw_accel_z = sensorValue.un.rawAccelerometer.z;
      break;
    case SH2_RAW_GYROSCOPE:
      msg.raw_gyro_x = sensorValue.un.rawGyroscope.x;
      msg.raw_gyro_y = sensorValue.un.rawGyroscope.y;
      msg.raw_gyro_z = sensorValue.un.rawGyroscope.z;
      break;
    case SH2_RAW_MAGNETOMETER:
      msg.raw_mag_x = sensorValue.un.rawMagnetometer.x;
      msg.raw_mag_y = sensorValue.un.rawMagnetometer.y;
      msg.raw_mag_z = sensorValue.un.rawMagnetometer.z;
      break;
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

  frost_interfaces__msg__IMU msg;
};