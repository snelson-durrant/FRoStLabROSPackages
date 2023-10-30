#include "publisher.hpp"
#include <SoftwareSerial.h>
#include <frost_interfaces/msg/echo.h>
#include <ping1d.h>

#define ECHO_RATE 115200
#define RX_PIN 21
#define TX_PIN 20

class EchoPub : Publisher {

public:
  void setup(rcl_node_t node) {

    ping_serial.begin(ECHO_RATE);

    RCCHECK(rclc_publisher_init_best_effort(
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
  SoftwareSerial ping_serial = SoftwareSerial(RX_PIN, TX_PIN);
  Ping1D ping{ping_serial};

  frost_interfaces__msg__Echo msg;
};