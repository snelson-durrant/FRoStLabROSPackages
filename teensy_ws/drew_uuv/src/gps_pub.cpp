// #include <publisher.cpp>
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <gps_pub.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <SoftwareSerial.h>
#include <frost_interfaces/srv/get_gps.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loopy();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
frost_interfaces__srv__GetGPS_Response msgRes;
frost_interfaces__srv__GetGPS_Request msg_request;

static const uint8_t RXPin = 28;
static const uint8_t TXPin = 29;
SoftwareSerial gps_serial = SoftwareSerial(RXPin, TXPin);
SFE_UBLOX_GNSS GNSS;

void gps_service_callback(const void * request_msg, void * response_msg){
    //We might need to change this the input thing is wierd
    frost_interfaces__srv__GetGPS_Request * req_in =
    (frost_interfaces__srv__GetGPS_Request *) request_msg;
    frost_interfaces__srv__GetGPS_Response * res_in = 
        (frost_interfaces__srv__GetGPS_Response *) response_msg;
    
    res_in->longitude = GNSS.getLongitude();
    res_in->latitude = GNSS.getLatitude();
    res_in->altitude = GNSS.getAltitude();
    res_in->siv= GNSS.getSIV();
    res_in->header.stamp.nanosec = rmw_uros_epoch_nanos();
}

void error_loopy() {
    while(1) {
        delay(100);
    }
}


void GPSPub::setup(rcl_node_t node){
    gps_serial.begin(9600);
    GNSS.begin(gps_serial); 
    
    RCCHECK(rclc_service_init_best_effort(
    &gps_srv,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(frost_interfaces, srv, GetGPS),
    "gps_service"));
    

}

void GPSPub::destroy(rcl_node_t node){
    rcl_service_fini(&gps_srv, &node);
}

