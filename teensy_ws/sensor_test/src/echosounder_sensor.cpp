#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ping1d.h>

static const uint8_t arduino_RxPin = 21;
static const uint8_t arduino_TxPin = 20;
SoftwareSerial ping_serial = SoftwareSerial(arduino_RxPin, arduino_TxPin);
static Ping1D ping{ping_serial};
int64_t echo_distance;
int64_t echo_confidence;

void setup_echo() {

     ping_serial.begin(115200);
     while (!ping.initialize()) {
        Serial.println("\nPing device failed to initialize!");
        Serial.println("Are the Ping rx/tx wired correctly?");
        Serial.print("Ping rx is the green wire, and should be connected to Arduino pin ");
        Serial.print(arduino_TxPin);
        Serial.println(" (Arduino tx)");
        Serial.print("Ping tx is the white wire, and should be connected to Arduino pin ");
        Serial.print(arduino_RxPin);
        Serial.println(" (Arduino rx)");
        delay(2000);
    }

}

void loop_echo() {

ping.update();
echo_distance = ping.distance();
echo_confidence = ping.confidence();

Serial.println(echo_confidence);
Serial.println(echo_distance);


if (ping.request(PingMessageId::PING1D_PROFILE)) {
    Serial.println("got profile");
    Serial.println("profile points: ");
    for (int i = 0; i < ping.profile_data_length(); i++) {
        Serial.print(" > ");
        Serial.println(ping.profile_data()[i]);
        }
    } else {
        Serial.println("attempt to get profile failed");
    }

}