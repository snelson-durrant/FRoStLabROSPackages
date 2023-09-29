#include "service.cpp"
#include <SoftwareSerial.h>
#include <ping1d.h>
#include <frost_interfaces/srv/get_echo.h>

#define ECHO_RATE 115200
#define RX_PIN 21
#define TX_PIN 20

class EchoSrv : Service {
public:
  rcl_service_t service;
  frost_interfaces__srv__GetEcho_Response msgRes;
  frost_interfaces__srv__GetEcho_Request msgReq;

  void setup(rcl_node_t node) {
    ping_serial.begin(ECHO_RATE);

    RCCHECK(rclc_service_init_best_effort(
        &service, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(frost_interfaces, srv, GetEcho),
        "echo_service"));
  }

  void respond(const void *request_msg, void *response_msg) {
    frost_interfaces__srv__GetEcho_Response *res_in =
        (frost_interfaces__srv__GetEcho_Response *)response_msg;

    ping.update();
    res_in->distance = ping.distance();
    res_in->conf_level = ping.confidence();
    res_in->header.stamp.nanosec = rmw_uros_epoch_nanos();
  }

  virtual void destroy(rcl_node_t node) { 
    RCCHECK(rcl_service_fini(&service, &node));
  }

private:
  SoftwareSerial ping_serial = SoftwareSerial(RX_PIN, TX_PIN);
  Ping1D ping{ping_serial};
};
