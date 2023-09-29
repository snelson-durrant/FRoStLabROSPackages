#include <Adafruit_BNO08x.h>
#include <Arduino.h>

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
        
void setReports() {

    printf("Setting desired reports");
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
    // if (!bno08x.enableReport(SH2_GRAVITY)) {
    //     Serial.println("Could not enable gravity vector");
    // }
    // if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    //     Serial.println("Could not enable rotation vector");
    // }  
    // if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
    //     Serial.println("Could not enable geomagnetic rotation vector");
    // }
    // if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    //     Serial.println("Could not enable raw accelerometer");
    // }
    // if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    //     Serial.println("Could not enable raw gyroscope");
    // }
    // if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    //     Serial.println("Could not enable raw magnetometer");
    // }
}

void setup_imu() {
    
    while(!bno08x.begin_I2C()) {
        Serial.println("Failed to find BNO08x chip");
        // while (1) {
        //     delay(10);
        //     Serial.print("IMU is not on");
        // }
        delay(10);
    }

    Serial.println("BNO08x Found!");
    setReports();
    Serial.println("Reading events");

}

void loop_imu() {
    
    
    if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

  case SH2_ACCELEROMETER:
    Serial.print("Accelerometer - x: ");
    Serial.print(sensorValue.un.accelerometer.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.accelerometer.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.accelerometer.z);
    break;
  case SH2_GYROSCOPE_CALIBRATED:
    Serial.print("Gyro - x: ");
    Serial.print(sensorValue.un.gyroscope.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.gyroscope.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.gyroscope.z);
    break;
  case SH2_MAGNETIC_FIELD_CALIBRATED:
    Serial.print("Magnetic Field - x: ");
    Serial.print(sensorValue.un.magneticField.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.magneticField.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.magneticField.z);
    break;
  case SH2_LINEAR_ACCELERATION:
    Serial.print("Linear Acceration - x: ");
    Serial.print(sensorValue.un.linearAcceleration.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.linearAcceleration.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.linearAcceleration.z);
    break;
  }
    
    // float accel_x=sensorValue.un.accelerometer.x;
    // float accel_y=sensorValue.un.accelerometer.y;
    // float accel_z=sensorValue.un.accelerometer.z;
    // float gyro_x=sensorValue.un.gyroscope.x;
    // float gyro_y=sensorValue.un.gyroscope.y;
    // float gyro_z=sensorValue.un.gyroscope.z;
    // float mag_x=sensorValue.un.magneticField.x;
    // float mag_y=sensorValue.un.magneticField.y;
    // float mag_z=sensorValue.un.magneticField.z;
    // float lin_accel_x=sensorValue.un.linearAcceleration.x;
    // float lin_accel_y=sensorValue.un.linearAcceleration.y;
    // float lin_accel_z=sensorValue.un.linearAcceleration.z;
    // float grav_x=sensorValue.un.gravity.x;
    // float grav_y=sensorValue.un.gravity.y;
    // float grav_z=sensorValue.un.gravity.z;
    // float rot_vec_i=sensorValue.un.rotationVector.i;
    // float rot_vec_j=sensorValue.un.rotationVector.j;
    // float rot_vec_k=sensorValue.un.rotationVector.k;
    // float geomag_rot_vec_i=sensorValue.un.geoMagRotationVector.i;
    // float geomag_rot_vec_j=sensorValue.un.geoMagRotationVector.j;
    // float geomag_rot_vec_k=sensorValue.un.geoMagRotationVector.k;
    // float raw_accel_x=sensorValue.un.rawAccelerometer.x;
    // float raw_accel_y=sensorValue.un.rawAccelerometer.y;
    // float raw_accel_z=sensorValue.un.rawAccelerometer.z;
    // float raw_gyro_x=sensorValue.un.rawGyroscope.x;
    // float raw_gyro_y=sensorValue.un.rawGyroscope.y;
    // float raw_gyro_z=sensorValue.un.rawGyroscope.z;
    // float raw_mag_x=sensorValue.un.rawMagnetometer.x;
    // float raw_mag_y=sensorValue.un.rawMagnetometer.y;
    // float raw_mag_z=sensorValue.un.rawMagnetometer.z;
    // Serial.print("acceleromerter x: ");
    // Serial.println(accel_x);
    // Serial.print("acceleromerter y: ");
    // Serial.println(accel_y);
    // Serial.print("acceleromerter z: ");
    // Serial.println(accel_z);

}

     
