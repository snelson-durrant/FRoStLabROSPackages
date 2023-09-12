#include <Arduino.h>

int leak = 0;
#define leak_pin A2
int led_pin =13;

bool check_water_leak() {
    leak = digitalRead(leak_pin);
    digitalWrite(led_pin, leak);
    if (leak == HIGH) {
        return true;
    }
    else {
        return false;
    }  
}

void setup_leak() {

  pinMode(led_pin, OUTPUT);
  pinMode(leak_pin, INPUT);

}

void loop_leak() {
    
    Serial.println(check_water_leak());

}

