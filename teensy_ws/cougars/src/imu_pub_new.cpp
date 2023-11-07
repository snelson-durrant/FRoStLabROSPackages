#include "imu_pub_new.hpp"

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void IMUPubNew::setup(rcl_node_t node) {

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, IMU), "imu_data"));
}

void IMUPubNew::imu_setup() {

  Wire2.begin();
  myIMU.begin(0x4A, Wire2);
  Wire2.setClock(400000);

  myIMU.enableLinearAccelerometer(50); //Send data update every 50ms
  myIMU.enableRotationVector(50); //Send data update every 50ms

}

void IMUPubNew::imu_update() {

  if (myIMU.dataAvailable() == true)
  {
    msg.gyro_x = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    msg.gyro_y = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    msg.gyro_z = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees

    msg.accel_x = myIMU.getLinAccelX();
    msg.accel_y = myIMU.getLinAccelY();
    msg.accel_z = myIMU.getLinAccelZ();
    // byte linAccuracy = myIMU.getLinAccelAccuracy();
  }

}

void IMUPubNew::publish() {

  msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}