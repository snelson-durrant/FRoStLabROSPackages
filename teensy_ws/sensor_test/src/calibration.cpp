// #include <Arduino.h>
// #include <Wire.h>
// #include "MS5837.h"
// #include <Adafruit_BNO08x.h>
// #include <DHT.h>


// //global vars for humidity sensor
// #define DHTPIN 4
// #define DHTTYPE DHT22
// DHT dht(DHTPIN, DHTTYPE)

// //global var for pressure sensor
// MS5837 pressure_sensor;
// //Adafruit_BNO08x bno08x;
// //sh2_SensorValue_t sensorValue;   


// /*
// calibration performed out of water, I'd think
// A more accurate test 'might' include an underwater data point.
// I couldn't find a function that sets a particular depth reading to zero.
// so I stored both surface pressure and depth offset in a global variable.
// Unless we find a built in function, it might be a good idea to encapsulate
// the pressure sensor in another class and call a get depth that accounts
// for the measured error.
// https://github.com/bluerobotics/BlueRobotics_MS5837_Library/blob/master/README.md
// */
// float pressure_at_zero_depth;
// float depth_error_at_zero_depth;

// void pressure_calibrate() {
    
//     float sum_pressure_at_zero_depth = 0;
//     float sum_depth_error_at_zero_depth = 0;

//     for (int i=0; i<10; i++) {
//         pressure_sensor.read(); //ensures we get a new values each time
//         sum_pressure_at_zero_depth += pressure_sensor.pressure();
//         sum_depth_error_at_zero_depth += pressure_sensor.depth();
//         // delay(60) can add if nessesary. the read function takes ~ 40 ms according to documentation
//     }

//     pressure_at_zero_depth = sum_pressure_at_zero_depth*.1;
//     depth_error_at_zero_depth = sum_depth_error_at_zero_depth*.1;
    
// }

// /*
// again couldn't find built-in function, so saving to 
// global variable
// https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp
// */
// float humidity_on_init;
// void humidity_calibrate() {
    
//     float sum_humidity_on_init = 0;
//     for (int i=0; i<10; i++) {
//         sum_humidity_on_init += dht.readTemperature();
//         delay(100);
//     }
//     humidity_on_init = sum_humidity_on_init * .1;

// }

// void imu_calibrate() {

// }


// void setup_pressure_calibrate() {

//     Wire2.begin();
//     pressure_sensor.init();

//     Serial.print("calibrating pressure\n");

//     pressure_sensor.setFluidDensity(997); // freshwater density kg/m^3

//     pressure_calibrate(); 

//     Serial.print("pressure calibration complete\n");
//     Serial.print("depth error (zero depth): ");
//     Serial.print(depth_error_at_zero_depth);
//     Serial.print("pressure_error_zero_depth: ");
//     Serial.print(pressure_at_zero_depth);

// }

// void setup_humidity_calibrate() {
//     dht.begin();

//     Serial.print("Calibrating humidity sensor");
//     humidity_calibrate();
//     Serial.print("humidity calibration complete\n");
//     Serial.print("humidity on initialization: ");
//     Serial.print(humidity_on_init);

// }

