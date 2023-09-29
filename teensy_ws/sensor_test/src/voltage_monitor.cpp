#include <Arduino.h>
#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();
int64_t voltage;
int64_t current;

void setup_voltage() {
    Serial.println("Calibrating voltage sensor");
    
    ina260.begin(INA260_I2CADDR_DEFAULT, &Wire2);

    // set the number of samples to average
    ina260.setAveragingCount(INA260_COUNT_16);
    // set the time over which to measure the current and bus voltage
    ina260.setVoltageConversionTime(INA260_TIME_140_us);
    ina260.setCurrentConversionTime(INA260_TIME_140_us);

    Serial.println("voltage calibration complete");
}

void loop_voltage() {

    voltage = ina260.readBusVoltage();
	current = ina260.readCurrent(); 
    Serial.print("Voltage: ");
    Serial.println(voltage);
    Serial.print("Current: ");
    Serial.println(current);
    
}