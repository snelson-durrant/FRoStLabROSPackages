#include <service.cpp>
#include <SoftwareSerial.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <frost_interfaces/srv/get_gps.h>

class GPSSrv : Service {
public:
  rcl_service_t service;
  frost_interfaces__srv__GetGPS_Response msgRes;
  frost_interfaces__srv__GetGPS_Request msgReq;

  void setup(rcl_node_t node) {
    gps_serial.begin(9600);
    GNSS.begin(gps_serial);

    RCCHECK(rclc_service_init_best_effort(
        &service, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(frost_interfaces, srv, GetGPS),
        "gps_service"));
      
  }

  void respond(const void *request_msg, void *response_msg) {
    frost_interfaces__srv__GetGPS_Response *res_in =
        (frost_interfaces__srv__GetGPS_Response *)response_msg;

    res_in->longitude = GNSS.getLongitude();
    res_in->latitude = GNSS.getLatitude();
    res_in->altitude = GNSS.getAltitude();
    res_in->siv = GNSS.getSIV();
    res_in->header.stamp.nanosec = rmw_uros_epoch_nanos();
  }

  virtual void destroy(rcl_node_t node) { 
    RCCHECK(rcl_service_fini(&service, &node));
  }

private:
  static const uint8_t RXPin = 28;
  static const uint8_t TXPin = 29;
  SoftwareSerial gps_serial = SoftwareSerial(RXPin, TXPin);
  SFE_UBLOX_GNSS GNSS;
};
