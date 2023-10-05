#include <Arduino.h>
#include <humidity_sensor.h>
#include <pressure_sensor.h>
#include <leak_sensor.h>
#include <echosounder_sensor.h>
#include <voltage_monitor.h>
#include <servo_thruster.h>
#include <gps.h>
#include <imu.h>

//uncomment the one you want to test. Only one component can be tested at a time 

void setup() {
  Serial.begin(115200);
  delay(2000);
  //setup_hum();
  //setup_pressure();
  //setup_leak();
  //setup_echo();
  //setup_voltage();
  //setup_servo();
  //setup_gps();
  setup_imu();
  //setup_humidity_calibrate();
  //setup_pressure_calibrate();
}

void loop() {
  //loop_hum();
  //loop_pressure();
  //loop_leak();
  //loop_echo();
  //loop_voltage();
  //loop_servo();
  //loop_gps();
  loop_imu();
  //loop_calibrate();
}
