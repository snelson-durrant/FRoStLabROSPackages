#include "pid_control.hpp"
#include "echo_srv.hpp"
#include "gps_srv.hpp"
#include "humidity_pub.hpp"
#include "imu_pub.hpp"
#include "leak_pub.hpp"
#include "pressure_pub.hpp"
#include "voltage_pub.hpp"

#include <Servo.h>
#include <frost_interfaces/msg/nav.h>
#include <frost_interfaces/msg/pid.h>

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
#define CALLBACK_TOTAL 6
#define TIMER_PERIOD 30000
#define TIMER_PID_PERIOD 5
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

#define HEADING_P 0.6
#define HEADING_I 0.05
#define VELOCITY_P 0.6
#define VELOCITY_I 0.05
#define DEPTH_P 0.2
#define DEPTH_I 0.05

// micro-ROS objects
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_subscription_t subscriber;
static rcl_publisher_t pid_publisher;
static rcl_publisher_t nav_publisher;
static rcl_timer_t timer_pub;
static rcl_timer_t timer_pid;
static frost_interfaces__msg__PID msg;
static frost_interfaces__msg__PID *pid_request_msg =
    new frost_interfaces__msg__PID;

// publisher objects
static VoltagePub voltage_pub;
static HumidityPub humidity_pub;
static LeakPub leak_pub;
static PressurePub pressure_pub;
static IMUPub imu_pub;

// service objects
static GPSSrv gps_srv;
static EchoSrv echo_srv;

// servo, thruster objects
static Servo my_servo1;
static Servo my_servo2;
static Servo my_servo3;
static Servo thruster;

// PID objects
static PID_Control Heading(HEADING_P, HEADING_I, 40, 140,
                           90); // TO DO: fix magic numbers
static PID_Control Velocity(VELOCITY_P, VELOCITY_I, 0, 100, 0);
static PID_Control Depth(DEPTH_P, DEPTH_I, 45, 135, 90);

// states for state machine in loop function
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} static state;

// used to make sure imu_setup is only called once
static bool already_setup = false;

// responds to errors with micro-ROS functions
static void error_loop() {
  while (1) {
    delay(100);
  }
}

// TODO: Can we add this to the PID object?
int compute_heading(float goal_heading, float heading_curr) {

  static float input = 0;
  static float output = 0;
  if (goal_heading > heading_curr) {
    if ((360 - goal_heading + heading_curr) <
        (goal_heading - heading_curr)) { // TODO: fix magic numbers
      input = -1 * (360 - goal_heading + heading_curr);
    } else {
      input = goal_heading - heading_curr;
    }
  } else { // add this to nav
    if ((360 - heading_curr + goal_heading) < (heading_curr - goal_heading)) {
      input = (360 - heading_curr + goal_heading);
    } else {
      input = goal_heading - heading_curr;
    }
  }
  output = Heading.compute(goal_heading, heading_curr);
  return int(output);
}

void run_pid() {

  //////////////////////////////////////////////////////////
  // LOW-LEVEL CONTROLLER CODE STARTS HERE
  //////////////////////////////////////////////////////////

  if (pid_request_msg->stop == false) {

    // TODO: add PID stuff here

    // reference wanted values using pid_request_msg->velocity,
    // pid_request_msg->yaw, etc

    // use custom functions from imu_pub and depth_pub to get values

    int servo1_angle =
        compute_heading(pid_request_msg->yaw, imu_pub.returnYaw());
    int thruster_speed =
        Velocity.compute(pid_request_msg->velocity, imu_pub.returnVel());

    // TODO: test thruster using this code
    int servo2_angle = map(thruster_speed, 0, 100, 25, 165);
    Serial5.print("Thruster Value: ");
    Serial5.print(map(thruster_speed, 0, 100, 1500, 2000));

    // TODO: use this code to write to the servos and thruster
    my_servo1.write(servo1_angle);
    my_servo2.write(DEFAULT_SERVO);
    my_servo3.write(DEFAULT_SERVO);
    int thrusterValue =
        map(0, LOW_FINAL, HIGH_FINAL, LOW_INITIAL, HIGH_INITIAL);
    thruster.writeMicroseconds(thrusterValue);

  } else {

    my_servo1.write(DEFAULT_SERVO);
    my_servo2.write(DEFAULT_SERVO);
    my_servo3.write(DEFAULT_SERVO);
    thruster.writeMicroseconds(DEFAULT_THRUSTER);
  }

  //////////////////////////////////////////////////////////
  // LOW-LEVEL CONTROLLER CODE ENDS HERE
  //////////////////////////////////////////////////////////
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
  delay(7000);
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
void timer_pub_callback(rcl_timer_t *timer, int64_t last_call_time) {

  (void)last_call_time;

  // TODO: test how fast the code is running using the below
  static unsigned long last_timer = 0;
  static unsigned long current = 0;

  if (timer != NULL) {

    // TODO: test how fast the code is running using the below
    Serial5.print("PUBLISH LOOP TIMER: ");
    current = millis();
    Serial5.println(current - last_timer);
    last_timer = current;

    voltage_pub.publish();
    humidity_pub.publish();
    leak_pub.publish();
    // pressure_pub.publish();
    // imu_pub.publish();
    // RCSOFTCHECK(rcl_publish(&nav_publisher, &nav_msg, NULL));
    // RCSOFTCHECK(rcl_publish(&pid_publisher, &pid_actual_msg, NULL));
  }
}

// TODO: ADD HERE
void timer_pid_callback(rcl_timer_t *timer, int64_t last_call_time) {

  (void)last_call_time;

  // TODO: test how fast the code is running using the below
  static unsigned long last_timer = 0;
  static unsigned long current = 0;

  if (timer != NULL) {

    // TODO: test how fast the code is running using the below
    Serial5.print("PID LOOP TIMER: ");
    current = millis();
    Serial5.println(current - last_timer);
    last_timer = current;

    imu_pub.imu_update();
    pressure_pub.pressure_update();
    run_pid();
  }
}

// micro-Ros function that subscribes to requested PID values
void subscription_callback(const void *pid_msgin) {
  pid_request_msg = (frost_interfaces__msg__PID *)pid_msgin;
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
  // pressure_pub.setup(node);
  // imu_pub.setup(node);

  // create services
  gps_srv.setup(node);
  echo_srv.setup(node);

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, PID), "pid_request"));

  // create publishers
  // RCCHECK(rclc_publisher_init_best_effort(
  //     &pid_publisher, &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, PID),
  //     "pid_actual"));
  // RCCHECK(rclc_publisher_init_best_effort(
  //     &nav_publisher, &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Nav),
  //     "nav_commands"));

  // create timer (handles periodic publications)
  RCCHECK(rclc_timer_init_default(
      &timer_pub, &support, RCL_MS_TO_NS(TIMER_PERIOD), timer_pub_callback));
  RCCHECK(rclc_timer_init_default(&timer_pid, &support,
                                  RCL_MS_TO_NS(TIMER_PID_PERIOD),
                                  timer_pid_callback));

  // create executor
  RCSOFTCHECK(rclc_executor_init(&executor, &support.context, CALLBACK_TOTAL,
                                 &allocator));

  // add callbacks to executor
  RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer_pub));
  RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer_pid));
  RCSOFTCHECK(rclc_executor_add_service(&executor, &gps_srv.service,
                                        &gps_srv.msgReq, &gps_srv.msgRes,
                                        gps_service_callback));
  RCSOFTCHECK(rclc_executor_add_service(&executor, &echo_srv.service,
                                        &echo_srv.msgReq, &echo_srv.msgRes,
                                        echo_service_callback));
  RCSOFTCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // wait for first new data to arrive from pid_request topic
  pid_request_msg->stop = true;

  if (!already_setup) {
    imu_pub.imu_setup();
    pressure_pub.pressure_setup();
    already_setup = true;
  }

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  // destroy publishers
  voltage_pub.destroy(node);
  humidity_pub.destroy(node);
  leak_pub.destroy(node);
  // pressure_pub.destroy(node);
  // imu_pub.destroy(node);

  // destroy services
  gps_srv.destroy(node);
  echo_srv.destroy(node);

  // destroy everything else
  rcl_subscription_fini(&subscriber, &node);
  // rcl_publisher_fini(&pid_publisher, &node);
  // rcl_publisher_fini(&nav_publisher, &node);
  rcl_timer_fini(&timer_pub);
  rcl_timer_fini(&timer_pid);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {

  Serial.begin(BAUD_RATE);
  set_microros_serial_transports(Serial);
  pin_setup();

  state = WAITING_AGENT;
}

void loop() {

  // state machine to manage connecting and disconnecting the micro-ROS agent
  switch (state) {
  case WAITING_AGENT:
    Serial5.println("test");
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

  Serial5.

}
