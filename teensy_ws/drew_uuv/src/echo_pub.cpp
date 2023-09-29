#include <SoftwareSerial.h>
#include <frost_interfaces/msg/echo.h>
#include <ping1d.h>
#include <publisher.cpp>

class EchoPub : Publisher {

public:
  void setup(rcl_node_t node) {

    ping_serial.begin(115200);

    RCCHECK(rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Echo), "echo_data"));
  }

  void publish() {
    // ping.request(PingMessageId::PING1D_PROFILE);
    // msg.profile_data.data=ping.profile_data();
    ping.update();
    msg.distance = ping.distance();
    msg.conf_level = ping.confidence();
    msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }

  using Publisher::destroy;

private:
  static const uint8_t arduino_RxPin = 21;
  static const uint8_t arduino_TxPin = 20;
  SoftwareSerial ping_serial = SoftwareSerial(arduino_RxPin, arduino_TxPin);
  Ping1D ping{ping_serial};

  frost_interfaces__msg__Echo msg;
};