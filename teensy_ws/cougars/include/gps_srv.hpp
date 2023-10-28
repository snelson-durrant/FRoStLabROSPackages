#ifndef GPS_SRV
#define GPS_SRV

#include "service.hpp"
#include <SoftwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <frost_interfaces/srv/get_gps.h>

class GPSSrv : Service {

public:
  rcl_service_t service;
  frost_interfaces__srv__GetGPS_Response msgRes;
  frost_interfaces__srv__GetGPS_Request msgReq;

  void setup(rcl_node_t node);
  void respond(const void *request_msg, void *response_msg);
  virtual void destroy(rcl_node_t node);

};

#endif // GPS_SRV