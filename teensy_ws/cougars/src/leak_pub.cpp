#include "leak_pub.hpp"

#define LEAK_PIN 16

static int leak = 0;

void LeakPub::setup(rcl_node_t node) {

  pinMode(LEAK_PIN, INPUT);

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Leak),
      "leak_detected"));
}

void LeakPub::publish() {

  bool water_leak = check_water_leak();
  if (water_leak) {
    msg.leak_detected = water_leak;
    msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

bool LeakPub::check_water_leak() {

  leak = digitalRead(LEAK_PIN);
  if (leak == 1) {
    return true;
  } else {
    return false;
  }
}