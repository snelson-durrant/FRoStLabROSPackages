#include <Arduino.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
// #include <DHT_U.h>

#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

float humidity;
float temp;

uint32_t delayMS;

void setup_hum() {
  
  dht.begin();
}

void loop_hum() {
    Serial.print("Temp: ");
    Serial.println(dht.readTemperature());
    Serial.println(millis());
    delay(1000);
    Serial.print("Humid: ");
    Serial.println(dht.readHumidity());
}