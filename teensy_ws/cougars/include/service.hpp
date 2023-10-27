#ifndef SERVICE
#define SERVICE

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop();                                                            \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

class Service {

public:
  // these need to be defined by each service
  virtual void setup(rcl_node_t node) = 0;
  virtual void respond(const void *request_msg, void *response_msg) = 0;
  virtual void destroy(rcl_node_t node) = 0;

protected:
  void error_loop() {
    while (1) {
      delay(100);
    }
  }
};

#endif // SERVICE