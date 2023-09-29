#include <Arduino.h>
#include <Adafruit_INA260.h>
#include <Wire.h>

Adafruit_INA260 ina260 = Adafruit_INA260();
int64_t voltage;
int64_t current;

void setup_voltage() {
    ina260.begin(INA260_I2CADDR_DEFAULT, &Wire2);
}

void loop_voltage() {

    voltage = ina260.readBusVoltage();
	current = ina260.readCurrent(); 
    Serial.print("Voltage: ");
    Serial.println(voltage);
    Serial.print("Current: ");
    Serial.println(current);
    
}