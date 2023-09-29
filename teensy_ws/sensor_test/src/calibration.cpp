#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"
#include <Adafruit_BNO08x.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA260.h>

//global vars for humidity sensor
#define DHTPIN2 4
#define DHTTYPE2 DHT22
DHT dht2(DHTPIN2, DHTTYPE2);

//global var for pressure sensor
MS5837 pressure_sensor2;

Adafruit_INA260 ina260 = Adafruit_INA260();

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;   



/*
calibration performed out of water, I'd think
A more accurate test 'might' include an underwater data point.
I couldn't find a function that sets a particular depth reading to zero.
so I stored both surface pressure and depth offset in a global variable.
Unless we find a built in function, it might be a good idea to encapsulate
the pressure sensor in another class and call a get depth that accounts
for the measured error.
https://github.com/bluerobotics/BlueRobotics_MS5837_Library/blob/master/README.md
*/
float pressure_at_zero_depth;
float depth_error_at_zero_depth;

void pressure_calibrate() {
    
    float sum_pressure_at_zero_depth = 0;
    float sum_depth_error_at_zero_depth = 0;

    for (int i=0; i<10; i++) {
        pressure_sensor2.read(); //ensures we get a new values each time
        sum_pressure_at_zero_depth += pressure_sensor2.pressure();
        sum_depth_error_at_zero_depth += pressure_sensor2.depth();
        // delay(60) can add if nessesary. the read function takes ~ 40 ms according to documentation
    }

    pressure_at_zero_depth = sum_pressure_at_zero_depth*.1;
    depth_error_at_zero_depth = sum_depth_error_at_zero_depth*.1;
    
}

/*
again couldn't find built-in function, so saving to 
global variable
https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp
*/
float humidity_on_init;
void humidity_calibrate() {
    
    float sum_humidity_on_init = 0;
    for (int i=0; i<10; i++) {
        sum_humidity_on_init += dht2.readHumidity();
        delay(100);
    }
    humidity_on_init = sum_humidity_on_init * .1;

}


void setup_pressure_calibrate() {

    Wire2.begin();
    pressure_sensor2.init();

    Serial.println("calibrating pressure");

    pressure_sensor2.setFluidDensity(997); // freshwater density kg/m^3

    pressure_calibrate(); 

    Serial.println("pressure calibration complete");
    Serial.print("depth error (zero depth): ");
    Serial.println(depth_error_at_zero_depth);
    Serial.print("pressure_error_zero_depth: ");
    Serial.println(pressure_at_zero_depth);

}

void setup_humidity_calibrate() {
    dht2.begin();

    Serial.println("Calibrating humidity sensor");
    humidity_calibrate();
    Serial.println("humidity calibration complete");
    Serial.print("humidity on initialization: ");
    Serial.println(humidity_on_init);

}

//https://github.com/adafruit/Adafruit_INA260/blob/master/examples/ina260_tuning/ina260_tuning.ino
void setup_voltage_calibrate() {
    Serial.println("Calibrating voltage sensor");
    
    ina260.begin();

    // set the number of samples to average
    ina260.setAveragingCount(INA260_COUNT_16);
    // set the time over which to measure the current and bus voltage
    ina260.setVoltageConversionTime(INA260_TIME_140_us);
    ina260.setCurrentConversionTime(INA260_TIME_140_us);

    Serial.println("voltage calibration complete")
}

void loop_voltage_calibration() {
  // measure and print current, voltage, and power to display on the serial plotter
  Serial.print(ina260.readCurrent());
  Serial.print(" ");
  Serial.print(ina260.readBusVoltage());
  Serial.print(" ");
  Serial.print(ina260.readPower());
  Serial.println();
 
  delay(125); // Delay for a period of time (in milliseconds).
}

void setup_imu_calibrate() {
    Serial.println("Calibrating IMU");
    Serial.println("Sys\tgyro\taccel\tmag");
    bno08x.begin_I2C();

    bool calibration_completed = false;

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;

    uint8_t threshold = 3; //calibrations from 0-3 where three is the best
    //calibration will exit when all levels are above threshold

    while (!calibration_completed) {
        bno08x.getCalibration(&system, &gyro, &accel, &mag);
        Serial.print(system);
        Serial.print("\t");
        Serial.print(gyro);
        Serial.print("\t");
        Serial.print(accel);
        Serial.print("\t");
        Serial.println(mag);

        if(gyro>=threshold & accel>=threshold & mag>=threshold) {
            calibration_completed = true;
        }

        delay(250);
    }

    System.println("IMU Calibration Complete")

}

