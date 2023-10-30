#include "gps_srv.hpp"

#define GPS_RATE 9600
#define GPS_RX_PIN 28
#define GPS_TX_PIN 29

static SoftwareSerial gps_serial = SoftwareSerial(GPS_RX_PIN, GPS_TX_PIN);
static SFE_UBLOX_GNSS GNSS;

void GPSSrv::setup(rcl_node_t node) {
  gps_serial.begin(GPS_RATE);
  GNSS.begin(gps_serial);

  RCCHECK(rclc_service_init_best_effort(
      &service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(frost_interfaces, srv, GetGPS),
      "gps_service"));
    
}

void GPSSrv::respond(const void *request_msg, void *response_msg) {
  frost_interfaces__srv__GetGPS_Response *res_in =
      (frost_interfaces__srv__GetGPS_Response *)response_msg;

  res_in->longitude = GNSS.getLongitude();
  res_in->latitude = GNSS.getLatitude();
  res_in->altitude = GNSS.getAltitude();
  res_in->siv = GNSS.getSIV();
  res_in->header.stamp.nanosec = rmw_uros_epoch_nanos();
}

void GPSSrv::destroy(rcl_node_t node) { 
  RCCHECK(rcl_service_fini(&service, &node));
}