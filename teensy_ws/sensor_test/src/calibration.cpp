#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"
#include <Adafruit_BNO08x.h>


MS5837 pressure_sensor;
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;   


void calibrate_imu() {
    


}

void calibrate_pressure() {

    
}


void setup_calibrate() {

    Wire2.begin();
    pressure_sensor.init();

    pressure_sensor.setFluidDensity(997);

        calibrate_pressure();


    if (!bno08x.begin_I2C()) {
        Serial.println("Failed to find BNO08x chip");
        // while (1) {
        //     delay(10);
        //     Serial.print("IMU is not on");
        // }
    }

    else {
        Serial.println("BNO08x Found!");
        setReports();
        Serial.println("Reading events");
    }

    //calibrate_imu();

}

void loop_calibrate() {

}