#ifndef ECHO_SRV
#define ECHO_SRV

#include "service.hpp"
#include <SoftwareSerial.h>
#include <ping1d.h>
#include <frost_interfaces/srv/get_echo.h>

class EchoSrv : Service {

public:
  rcl_service_t service;
  frost_interfaces__srv__GetEcho_Response msgRes;
  frost_interfaces__srv__GetEcho_Request msgReq;

  void setup(rcl_node_t node);
  void respond(const void *request_msg, void *response_msg);
  virtual void destroy(rcl_node_t node);

};

#endif // ECHO_SRV