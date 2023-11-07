#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <Servo.h>
#define SERVO_PIN1 9
#define SERVO_PIN2 10
#define SERVO_PIN3 11
#define THRUSTER_PIN 10
Servo my_servo;
Servo my_thruster; 
Servo my_servo12;
Servo my_servo22;
Servo my_servo32;
// PID Objects
#define HEADING_P 0.6
#define HEADING_I 0.05
PID_Control Heading(HEADING_P, HEADING_I, 35, 145, 90);
#define VELOCITY_P 0.6
#define VELOCITY_I 0.05
PID_Control Velocity(VELOCITY_P, VELOCITY_I, 0, 100, 0);
#define DEPTH_P 0.2
#define DEPTH_I 0.05
PID_Control Depth(DEPTH_P, DEPTH_I, 45, 135, 90);
float goal_heading = 275;
float goal_velocity = 3.0;
float goal_pitch = 180;

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
float prev_accel = 0;
unsigned long prev_time = 0;

float returnYaw(){return ypr.yaw + 180.00;}
double returnVel(){return velocity;}
float returnPitch(){return ypr.pitch +180.00;}



void setup_PID(){
  my_servo12.attach(SERVO_PIN1);
  my_servo12.write(90);
  my_servo22.attach(SERVO_PIN2);
  my_servo22.write(90);
  my_servo32.attach(SERVO_PIN3);
  my_servo32.write(90);
}

int compute_heading(float heading_curr){          //TODO: Make the parameter a pointer directly to the heading 
    static float input;
    static float output;
    // Serial.print("Heading: ");
    // Serial.println(heading_curr);
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
    // Serial.print("Input: ");
    // Serial.println(input);
    output = Heading.compute(goal_heading, heading_curr);    
    // Serial.print("output");
    // Serial.println(output);
    return int(output);
  }

void PID_loop(){
  int servo2_angle = 1; // compute_heading(returnYaw());      //TODO: make this with pointers
  // Serial.print("Servo Angle: ");
  //Serial.println(servo1_angle);
  my_servo.write(servo2_angle);

  servo2_angle = 1; //compute_depth(returnPitch());
  my_servo22.write(servo2_angle);
  static int servo3_angle;
  if(servo2_angle > 90){
    servo3_angle = 90 - (servo2_angle - 90);
  }
  else{
    servo3_angle = 90 + (90-servo2_angle);
  }
  my_servo32.write(servo3_angle);
  // int thruster_speed = Velocity.compute(goal_velocity, returnVel());    
  // int servo2_angle = map(thruster_speed, 0, 100, 25, 165);
  // my_thruster.write(servo2_angle);
  // Serial.print("Thruster Angle: ");
  // Serial.println(servo2_angle);
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



void calculate_velocity() { // Could make function with pointers so it can
                            // calculate all velocities
  if (n_time < 1) {
    prev_time = micros();
    prev_accel = linear_accel_x;
    n_time++;
  } else {
    unsigned long current_time = micros();
    float delta_time = (current_time - prev_time) * 1e-6;
    velocity += (prev_accel + linear_accel_x) * 0.50 * delta_time;
    prev_time = current_time;
    prev_accel = linear_accel_x;
    Serial.println("Velocity: ");
    Serial.print(velocity*1000);
    Serial.print("\t");
    Serial.print(linear_accel_x*1000);
    Serial.print("\t");
    Serial.println(delta_time*1000);
  }
}


float calculated_velocity = 0.0;
unsigned long previous_time = 0;
float previous_acceleration = 0.0;
unsigned long prev_time_1 = 0;
float prev_accel_1 = 0.0;
bool first_time = false;

void calculate_velocity_simp() {
    if (!first_time) {
        previous_time = micros();
        previous_acceleration = linear_accel_x;
        first_time = true;
    } else {
        unsigned long current_time = micros();
        float delta_time = (current_time - previous_time) * 1e-6;

        // Simpson's 1/3 rule for numerical integration
        calculated_velocity += (prev_accel_1 + 4.0 * previous_acceleration + linear_accel_x) * delta_time / 6.0;

        prev_time_1 = previous_time;
        previous_time = current_time;
        prev_accel_1 = previous_acceleration;
        previous_acceleration = linear_accel_x;

        Serial.println("Velocity Simp: ");
        Serial.print(calculated_velocity*1000);
        Serial.print("\t");
        Serial.print(linear_accel_x*1000);
        Serial.print("\t");
        Serial.println(delta_time*1000);
    }
}

// float filter_accelerometer(float accelx, float rawaccel){
  
//   return 
// }
        
void setReports(void) {
    Serial.println("Setting desired reports");
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
      Serial.println("Could not enable linear acceleration");
    }
    if (! bno08x.enableReport(reportType)) {
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
    // Wire2.begin(BNO08x_I2CADDR_DEFAULT);
    Wire2.begin();
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
  //PID_loop();
  calculate_velocity();
  calculate_velocity_simp();
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }


  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    //Serial.println("Here");
    switch (sensorValue.sensorId) {
      case SH2_LINEAR_ACCELERATION:
        // Serial.print("Linear Acceration - x: ");
        // Serial.print(sensorValue.un.linearAcceleration.x);
        linear_accel_x = sensorValue.un.linearAcceleration.x;
        // linear_accel_x = filter
        // Serial.print(" y: ");
        // Serial.print(sensorValue.un.linearAcceleration.y);
        // Serial.print(" z: ");
        // Serial.println(sensorValue.un.linearAcceleration.z);
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
      
    }
  }


}
