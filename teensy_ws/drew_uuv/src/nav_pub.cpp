#include "publisher.cpp"
#include <frost_interfaces/msg/nav.h>
#include "ArduPID.h"

class NavPub : Publisher {

public:
  ArduPID Heading;
  int compute_heading(float heading_curr){          //TODO: Make the parameter a pointer directly to the heading 
    if(goal_heading > heading_curr){
        if((360 - goal_heading + heading_curr) < (goal_heading - heading_curr)){
            input = -1 * (360 - goal_heading + heading_curr);
        }
        else{
            input = goal_heading - heading_curr;
        }
        if((360 - heading_curr + goal_heading) < (heading_curr - goal_heading)){
            input = (360 - heading_curr + goal_heading);
        }
        else{
            input = goal_heading - heading_curr;
        }
    }
    Heading.compute();
    return int(output);
  }
  
  void setup(rcl_node_t node) {
    Heading.begin(&input, &output, &setpoint, p, i, d);

    // Heading.reverse()               // Uncomment if controller output is "reversed"
    // Heading.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
    Heading.setOutputLimits(45, 135);
    Heading.setBias(90);           //increase output by this amount
    Heading.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
    
    Heading.start();


//     RCCHECK(rclc_publisher_init_default(
//         &publisher, &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, nav),
//         "nav_teensy"));         //TODO: Make the nav teensy topic
//   
}

  void publish() {

   
  }

  using Publisher::destroy;

private:

    //frost_interfaces__msg__Nav msg;
    double input;
    double output;

    float goal_heading = 15.00;
    // Arbitrary setpoint and gains - adjust these as fit for your project:
    double setpoint = 0;
    double p = 10;
    double i = 1;
    double d = 0.5;
  
};