#include <publisher.cpp>
#include <std_msgs/msg/bool.h>
#include <frost_interfaces/msg/leak.h>

// #define led_pin 13
#define leak_pin 16


class LeakPub : Publisher {

    public:

        void setup(rcl_node_t node) {

            pinMode(leak_pin, INPUT);

            RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Leak),
            "leak_detected"));
        }

        void publish() {
        
            bool water_leak = check_water_leak();
            msg.leak_detected = water_leak;
            msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
            RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        }

        using Publisher::destroy;    
      
   private:

        int leak = 0;

        frost_interfaces__msg__Leak msg;

        bool check_water_leak() {
            leak = digitalRead(leak_pin);
            if (leak == 1) {
                return true;
            }
            else {
                return false;
            }   
        }
};