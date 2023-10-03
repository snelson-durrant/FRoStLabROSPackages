#include "echo_srv.cpp"
#include "gps_srv.cpp"
#include "humidity_pub.cpp"
#include "imu_pub.cpp"
#include "leak_pub.cpp"
#include "pressure_pub.cpp"
#include "voltage_pub.cpp"

#include <Servo.h>
#include <frost_interfaces/msg/nav.h>

#define EXECUTE_EVERY_N_MS(MS, X)                                              \
  do {                                                                         \
    static volatile int64_t init = -1;                                         \
    if (init == -1) {                                                          \
      init = uxr_millis();                                                     \
    }                                                                          \
    if (uxr_millis() - init > MS) {                                            \
      X;                                                                       \
      init = uxr_millis();                                                     \
    }                                                                          \
  } while (0)

#define BAUD_RATE 6000000
#define CALLBACK_TOTAL 5
#define TIMER_PERIOD 10
#define SYNC_TIMEOUT 1000

#define LOW_FINAL 0
#define HIGH_FINAL 100
#define LOW_INITIAL 1500
#define HIGH_INITIAL 2000

#define SERVO_PIN1 9
#define SERVO_PIN2 10
#define SERVO_PIN3 11
#define THRUSTER_PIN 12
#define DEFAULT_SERVO 90
#define DEFAULT_THRUSTER 1500

// micro-ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_timer_t timer;
frost_interfaces__msg__Nav msg;

// publisher objects
VoltagePub voltage_pub;
HumidityPub humidity_pub;
LeakPub leak_pub;
PressurePub pressure_pub;
IMUPub imu_pub;

// service objects
GPSSrv gps_srv;
EchoSrv echo_srv;

// servo, thruster variables
Servo my_servo1;
Servo my_servo2;
Servo my_servo3;
Servo thruster;

// states for statemachine in loop function
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// responds to errors with micro-ROS functions
void error_loop() {
  while (1) {
    delay(100);
  }
}

// pin setup for servos and thruster
void pin_setup() {

  pinMode(SERVO_PIN1, OUTPUT);
  pinMode(SERVO_PIN2, OUTPUT);
  pinMode(SERVO_PIN3, OUTPUT);
  pinMode(THRUSTER_PIN, OUTPUT);

  my_servo1.attach(SERVO_PIN1);
  my_servo2.attach(SERVO_PIN2);
  my_servo3.attach(SERVO_PIN3);
  thruster.attach(THRUSTER_PIN);

  my_servo1.write(DEFAULT_SERVO);
  my_servo2.write(DEFAULT_SERVO);
  my_servo3.write(DEFAULT_SERVO);
  thruster.writeMicroseconds(DEFAULT_THRUSTER);
}

// "fake function" to allow the service object function to be called
void gps_service_callback(const void *request_msg, void *response_msg) {
  gps_srv.respond(request_msg, response_msg);
}

// "fake function" to allow the service object function to be called
void echo_service_callback(const void *request_msg, void *response_msg) {
  echo_srv.respond(request_msg, response_msg);
}

// micro-ROS function that publishes all the data to their topics
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

  (void)last_call_time;
  if (timer != NULL) {

    voltage_pub.publish();
    humidity_pub.publish();
    leak_pub.publish();
    pressure_pub.publish();
    imu_pub.publish();
  }
}

// micro-Ros function that subscribes to navigation positions
void subscription_callback(const void *servo_msgin) {

  const frost_interfaces__msg__Nav *servo_msg =
      (const frost_interfaces__msg__Nav *)servo_msgin;
  my_servo1.write(servo_msg->servo1);
  my_servo2.write(servo_msg->servo2);
  my_servo3.write(servo_msg->servo3);
  int thrusterValue = map(servo_msg->thruster, LOW_FINAL, HIGH_FINAL,
                          LOW_INITIAL, HIGH_INITIAL);
  thruster.writeMicroseconds(thrusterValue);
}

bool create_entities() {

  // the allocator object wraps the dynamic memory allocation and deallocation
  // methods used in micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(
      rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // synchronize timestamps with the Raspberry Pi
  // after sync, timing should be able to be accessed with "rmw_uros_epoch"
  // functions
  RCCHECK(rmw_uros_sync_session(1000));

  // create publishers
  voltage_pub.setup(node);
  humidity_pub.setup(node);
  leak_pub.setup(node);
  pressure_pub.setup(node);
  imu_pub.setup(node);

  // create services
  gps_srv.setup(node);
  echo_srv.setup(node);

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Nav),
      "nav_instructions"));

  // create timer (handles periodic publications)
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(TIMER_PERIOD),
                                  timer_callback));

  // create executor
  RCSOFTCHECK(rclc_executor_init(&executor, &support.context, CALLBACK_TOTAL,
                                 &allocator));

  // add callbacks to executor
  RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));
  RCSOFTCHECK(rclc_executor_add_service(&executor, &gps_srv.service,
                                        &gps_srv.msgReq, &gps_srv.msgRes,
                                        gps_service_callback));
  RCSOFTCHECK(rclc_executor_add_service(&executor, &echo_srv.service,
                                        &echo_srv.msgReq, &echo_srv.msgRes,
                                        echo_service_callback));
  RCSOFTCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  Serial.print("end setup\n");

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // destroy publishers
  voltage_pub.destroy(node);
  humidity_pub.destroy(node);
  leak_pub.destroy(node);
  pressure_pub.destroy(node);
  imu_pub.destroy(node);

  // destroy services
  gps_srv.destroy(node);
  echo_srv.destroy(node);

  // destroy everything else
  rcl_subscription_fini(&subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {

  Serial.begin(BAUD_RATE);
  set_microros_serial_transports(Serial);
  pin_setup();
  imu_pub.imu_setup();

  state = WAITING_AGENT;
}

void loop() {
  imu_pub.imu_update();

  // state machine to manage connecting and disconnecting the micro-ROS agent
  switch (state) {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_AVAILABLE
                                        : WAITING_AGENT;);
    break;

  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
      destroy_entities();
    };
    break;

  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_CONNECTED
                                        : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;

  default:
    break;
  }
}
