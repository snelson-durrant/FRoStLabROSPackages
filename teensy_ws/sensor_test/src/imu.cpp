#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <Wire.h>

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;
#define FAST_MODE
#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
        
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
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

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setup_imu() {
    
    while(!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire2, 0)) {
        Serial.println("Failed to find BNO08x chip");
        Serial.println(millis());
        delay(10);
    }

    Serial.println("BNO08x Found!");
    setReports(reportType, reportIntervalUs);
    Serial.println("Reading events");

}

void loop_imu() {
    
    
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }


  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
  }
    static long last = 0;
    long now = micros();
    Serial.print(now - last);             Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.println(ypr.roll);
  



  // switch (sensorValue.sensorId) {

  // case SH2_ACCELEROMETER:
  //   Serial.print("Accelerometer - x: ");
  //   Serial.print(sensorValue.un.accelerometer.x);
  //   Serial.print(" y: ");
  //   Serial.print(sensorValue.un.accelerometer.y);
  //   Serial.print(" z: ");
  //   Serial.println(sensorValue.un.accelerometer.z);
  //   break;
  // case SH2_GYROSCOPE_CALIBRATED:
  //   Serial.print("Gyro - x: ");
  //   Serial.print(sensorValue.un.gyroscope.x);
  //   Serial.print(" y: ");
  //   Serial.print(sensorValue.un.gyroscope.y);
  //   Serial.print(" z: ");
  //   Serial.println(sensorValue.un.gyroscope.z);
  //   break;
  // case SH2_MAGNETIC_FIELD_CALIBRATED:
  //   Serial.print("Magnetic Field - x: ");
  //   Serial.print(sensorValue.un.magneticField.x);
  //   Serial.print(" y: ");
  //   Serial.print(sensorValue.un.magneticField.y);
  //   Serial.print(" z: ");
  //   Serial.println(sensorValue.un.magneticField.z);
  //   break;
  // case SH2_LINEAR_ACCELERATION:
  //   Serial.print("Linear Acceration - x: ");
  //   Serial.print(sensorValue.un.linearAcceleration.x);
  //   Serial.print(" y: ");
  //   Serial.print(sensorValue.un.linearAcceleration.y);
  //   Serial.print(" z: ");
  //   Serial.println(sensorValue.un.linearAcceleration.z);
  //   break;
  // }
    
  //   float accel_x=sensorValue.un.accelerometer.x;
  //   float accel_y=sensorValue.un.accelerometer.y;
  //   float accel_z=sensorValue.un.accelerometer.z;
  //   float gyro_x=sensorValue.un.gyroscope.x;
  //   float gyro_y=sensorValue.un.gyroscope.y;
  //   float gyro_z=sensorValue.un.gyroscope.z;
  //   float mag_x=sensorValue.un.magneticField.x;
  //   float mag_y=sensorValue.un.magneticField.y;
  //   float mag_z=sensorValue.un.magneticField.z;
  //   float lin_accel_x=sensorValue.un.linearAcceleration.x;
  //   float lin_accel_y=sensorValue.un.linearAcceleration.y;
  //   float lin_accel_z=sensorValue.un.linearAcceleration.z;
  //   float grav_x=sensorValue.un.gravity.x;
  //   float grav_y=sensorValue.un.gravity.y;
  //   float grav_z=sensorValue.un.gravity.z;
  //   float rot_vec_i=sensorValue.un.rotationVector.i;
  //   float rot_vec_j=sensorValue.un.rotationVector.j;
  //   float rot_vec_k=sensorValue.un.rotationVector.k;
  //   float rot_vec_real=sensorValue.un.rotationVector.real;
  //   float geomag_rot_vec_i=sensorValue.un.geoMagRotationVector.i;
  //   float geomag_rot_vec_j=sensorValue.un.geoMagRotationVector.j;
  //   float geomag_rot_vec_k=sensorValue.un.geoMagRotationVector.k;
  //   float geomag_rot_vec_real = sensorValue.un.geoMagRotationVector.real;
  //   float raw_accel_x=sensorValue.un.rawAccelerometer.x;
  //   float raw_accel_y=sensorValue.un.rawAccelerometer.y;
  //   float raw_accel_z=sensorValue.un.rawAccelerometer.z;
  //   float raw_gyro_x=sensorValue.un.rawGyroscope.x;
  //   float raw_gyro_y=sensorValue.un.rawGyroscope.y;
  //   float raw_gyro_z=sensorValue.un.rawGyroscope.z;
  //   float raw_mag_x=sensorValue.un.rawMagnetometer.x;
  //   float raw_mag_y=sensorValue.un.rawMagnetometer.y;
  //   float raw_mag_z=sensorValue.un.rawMagnetometer.z;

}

     
