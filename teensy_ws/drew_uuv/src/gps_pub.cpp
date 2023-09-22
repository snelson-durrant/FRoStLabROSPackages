#include <publisher.cpp>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <SoftwareSerial.h>
#include <frost_interfaces/msg/gps.h>


class GPSPub : Publisher 
{

    public:
        void setup(rcl_node_t node, rcl_allocator_t allocator, rclc_support_t support) 
        {
            gps_serial.begin(9600);
            GNSS.begin(gps_serial);

            
            RCCEHCK(rclc_service_init_best_effort(
            &gps_srv,
            &node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(frost_interfaces, srv, GetGPS),
            "gps_service"));

            // create Service executor (responds to requests with data to micro-ROS service)
	        RCSOFTCHECK(rclc_executor_init(&srv_executor, &support.context, 1, &allocator));
            RCSOFTCHECK(rclc_executor_add_service(&srvexecutor, &gps_srv, &msg_request, &msg, gps_service_callback);)

        }

        void publish() 
        {
            msg.longitude = GNSS.getLongitude();
            msg.latitude = GNSS.getLatitude();
            msg.altitude = GNSS.getAltitude();
            msg.siv= GNSS.getSIV();
            msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
            //RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        }
        void destroy(rcl_node_t node){
            rcl_service_fini(&gps_srv, &node);
            rclc_executor_fini(&srv_executor);
        }
        void spin_gps_executor(){
            RCSOFTCHECK(rclc_executor_spin_some(&srv_executor, RCL_MS_TO_NS(100)));
        }
      
   private:

    static const uint8_t RXPin = 28;
    static const uint8_t TXPin = 29;
    SoftwareSerial gps_serial = SoftwareSerial(RXPin, TXPin);
    SFE_UBLOX_GNSS GNSS;

    rclc_executor_t srv_executor;
    rcl_service_t gps_srv;

    frost_interfaces__srv__GetGPS_Response msg;
    frost_interfaces__srv__GetGPS_Request msg_request;

    void gps_service_callback(const void * request_msg, void * response_msg){
	    publish();
    }

};