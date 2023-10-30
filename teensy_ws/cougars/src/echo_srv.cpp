#include "echo_srv.hpp"

#define ECHO_RATE 115200
#define ECHO_RX_PIN 23
#define ECHO_TX_PIN 24 // we changed this

static SoftwareSerial ping_serial = SoftwareSerial(ECHO_RX_PIN, ECHO_TX_PIN);
static Ping1D ping{ping_serial};

void EchoSrv::setup(rcl_node_t node) {
  ping_serial.begin(ECHO_RATE);

  RCCHECK(rclc_service_init_best_effort(
      &service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(frost_interfaces, srv, GetEcho),
      "echo_service"));
}

void EchoSrv::respond(const void *request_msg, void *response_msg) {
  frost_interfaces__srv__GetEcho_Response *res_in =
      (frost_interfaces__srv__GetEcho_Response *)response_msg;

  ping.update();
  res_in->distance = ping.distance();
  res_in->conf_level = ping.confidence();
  res_in->header.stamp.nanosec = rmw_uros_epoch_nanos();
}

void EchoSrv::destroy(rcl_node_t node) { 
  RCCHECK(rcl_service_fini(&service, &node));
}