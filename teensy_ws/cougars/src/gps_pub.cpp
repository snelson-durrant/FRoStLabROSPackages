#include "publisher.hpp"
#include <SoftwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <frost_interfaces/msg/gps.h>

#define GPS_RATE 9600
#define RX_PIN 28
#define TX_PIN 29

class GPSPub : Publisher {

public:
  void setup(rcl_node_t node) {
    gps_serial.begin(GPS_RATE);
    GNSS.begin(gps_serial);

    RCCHECK(rclc_publisher_init_best_effort(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, GPS), "gps_data"));
  }

  void publish() {
    msg.longitude = GNSS.getLongitude();
    msg.latitude = GNSS.getLatitude();
    msg.altitude = GNSS.getAltitude();
    msg.siv = GNSS.getSIV();
    msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }

  using Publisher::destroy;

private:
  SoftwareSerial gps_serial = SoftwareSerial(RX_PIN, TX_PIN);
  SFE_UBLOX_GNSS GNSS;

  frost_interfaces__msg__GPS msg;
};