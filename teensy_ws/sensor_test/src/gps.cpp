#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

static const uint8_t RXPin = 28;
static const uint8_t TXPin = 29;
SoftwareSerial gps_serial = SoftwareSerial(RXPin, TXPin);
SFE_UBLOX_GNSS GNSS;
long longitude;
long latitude;
int SIV;
long altitude;

void setup_gps() {

    gps_serial.begin(9600);
    GNSS.begin(gps_serial);
}

void loop_gps() {

    longitude = GNSS.getLongitude();
    latitude = GNSS.getLatitude();
    SIV = GNSS.getSIV();
    altitude = GNSS.getAltitude();
    Serial.print("Longitude: ");
    Serial.println(longitude);
    Serial.print("Latitude: ");
    Serial.println(latitude);
    Serial.print("SIV: ");
    Serial.println(SIV);
    Serial.print("altitude: ");
    Serial.println(altitude);
    GNSS.getDay();
    GNSS.getMonth();
    GNSS.getYear();
    GNSS.getHour();
    GNSS.getMinute();
    GNSS.getSecond();

}