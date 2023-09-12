#include <Arduino.h>
#include <Wire.h>
#include <MS5837.h>

float pressure;
float depth;
float temperature;
MS5837 pressure_sensor;

void setup_pressure(){
    
    Wire2.begin();
    
    while (!pressure_sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

    pressure_sensor.setFluidDensity(997);

}

void loop_pressure(){

    pressure_sensor.read();
    pressure = pressure_sensor.pressure();
    depth = pressure_sensor.depth();
    temperature=pressure_sensor.temperature();
    Serial.print("Temp: ");
    Serial.println(temperature);
    Serial.print("Pressure: ");
    Serial.println(pressure);
    Serial.print("Depth: ");
    Serial.println(depth);
    
}