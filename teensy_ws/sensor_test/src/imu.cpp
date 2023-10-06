#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <Servo.h>
#define SERVO_PIN1 9
#define THRUSTER_PIN 10
Servo my_servo;
Servo my_thruster; 
PID_Control Heading;
PID_Control Velocity; 

float input;
float output;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

float linear_accel_x;
int n_time = 0;
double velocity = 0;
float prev_accel_1 = 0;
float prev_accel_2 = 0;
unsigned long prev_time_1 = 0;
unsigned long prev_time_2 = 0;

float returnYaw(){return ypr.yaw + 180.00;}
double returnVel(){return velocity;}

float goal_heading = 275;
// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 0;
double p = 0.2;   //Tune the PID!
double i = 0.05;
double d = 0.5;

float goal_velocity = 3.0;
// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint_velocity = 0;
double pv = 0.4;   //Tune the PID!
double iv = 0.2;
double dv = 0.5;

void setup_PID(){
  my_servo.attach(SERVO_PIN1);
  my_servo.write(90);
  Heading.begin(p,i,45, 135, 90);
  my_thruster.attach(THRUSTER_PIN);
  my_thruster.write(90);
  Velocity.begin(pv, iv, 0, 100);
}

int compute_heading(float heading_curr){          //TODO: Make the parameter a pointer directly to the heading 
    Serial.print("Heading: ");
    Serial.println(heading_curr);
    if(goal_heading > heading_curr){
        if((360 - goal_heading + heading_curr) < (goal_heading - heading_curr)){
            input = -1 * (360 - goal_heading + heading_curr);
        }
        else{
            input = goal_heading - heading_curr;
        }
    }
    else{ // add this to nav
      if((360 - heading_curr + goal_heading) < (heading_curr - goal_heading)){
          input = (360 - heading_curr + goal_heading);
      }
      else{
          input = goal_heading - heading_curr;
      }
    }
    Serial.print("Input: ");
    Serial.println(input);
    output = Heading.compute(goal_heading, heading_curr);    
    Serial.print("output");
    Serial.println(output);
    return int(output);
  }

void PID_loop(){
  int servo1_angle = compute_heading(returnYaw());      //TODO: make this with pointers
  Serial.print("Servo Angle: ");
  Serial.println(servo1_angle);
  my_servo.write(servo1_angle);
  //int thruster = compute_velocity(returnVel());
}

//#define FAST_MODE
#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif



void calculate_velocity(){     //Could make function with pointers so it can calculate all velocities
  if(n_time < 2){
    prev_time_2 = prev_time_1;
    prev_time_1 = micros();
    prev_accel_2 = prev_accel_1;
    prev_accel_1 = linear_accel_x;
    n_time++;
  }
  else{
    unsigned long current_time = micros();
    float delta_time = (current_time - prev_time_2) *1e-6;
    velocity += (prev_accel_2 + 4.0 * prev_accel_1 + linear_accel_x) * delta_time / 6.0;
    // velocity += (prev_accel + linear_accel_x) * 0.50 * delta_time;  //trapezoidal
    // prev_time = micros();
    // prev_accel = linear_accel_x;
    prev_time_2 = prev_time_1;
    prev_time_1 = current_time;
    prev_accel_2 = prev_accel_1;
    prev_accel_1 = linear_accel_x;
    Serial.print("Velocity: ");
    Serial.println(velocity);
  }
}
        
void setReports(void) {
    Serial.println("Setting desired reports");
    // if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    //   Serial.println("Could not enable accelerometer");
    // }
    // if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    //   Serial.println("Could not enable gyroscope");
    // }
    // if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    //   Serial.println("Could not enable magnetic field calibrated");
    // }
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
      Serial.println("Could not enable linear acceleration");
    }
    if (! bno08x.enableReport(reportType)) {
    Serial.println("Could not enable stabilized remote vector");
    }
    // if (!bno08x.enableReport(SH2_GRAVITY)) {
    //   Serial.println("Could not enable gravity vector");
    // }
    // if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    //   Serial.println("Could not enable rotation vector");
    // }
    // if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
    //   Serial.println("Could not enable geomagnetic rotation vector");
    // }
    // if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    //   Serial.println("Could not enable raw accelerometer");
    // }
    // if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    //   Serial.println("Could not enable raw gyroscope");
    // }
    // if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    //   Serial.println("Could not enable raw magnetometer");
    // }
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
    // Wire2.begin(BNO08x_I2CADDR_DEFAULT);
    while(!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire2, 0)) {
        Serial.println("Failed to find BNO08x chip");
        Serial.println(millis());
        delay(100);
        // bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire2, 0);
    }

    Serial.println("BNO08x Found!");
    setReports();
    Serial.println("Reading events");
    setup_PID();
}

long now;
static long last =0;

void loop_imu() {
  PID_loop();
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }


  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    Serial.println("Here");
    switch (sensorValue.sensorId) {
      case SH2_LINEAR_ACCELERATION:
        Serial.print("Linear Acceration - x: ");
        Serial.print(sensorValue.un.linearAcceleration.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.linearAcceleration.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.linearAcceleration.z);
        calculate_velocity();
        
        break;
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        now = micros();
        Serial.print(now - last);             Serial.print("\t");
        last = now;
        Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
        Serial.print(ypr.yaw);                Serial.print("\t");
        Serial.print(ypr.pitch);              Serial.print("\t");
        Serial.println(ypr.roll);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
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
      
    }
  }


}
