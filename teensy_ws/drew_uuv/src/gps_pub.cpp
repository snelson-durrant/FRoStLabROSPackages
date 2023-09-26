#include <SoftwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <frost_interfaces/msg/gps.h>
#include <publisher.cpp>

class GPSPub : Publisher {

public:
  void setup(rcl_node_t node) {
    gps_serial.begin(9600);
    GNSS.begin(gps_serial);

    RCCHECK(rclc_publisher_init_default(
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
  static const uint8_t RXPin = 28;
  static const uint8_t TXPin = 29;
  SoftwareSerial gps_serial = SoftwareSerial(RXPin, TXPin);
  SFE_UBLOX_GNSS GNSS;

  frost_interfaces__msg__GPS msg;
};