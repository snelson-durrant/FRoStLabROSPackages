#include <echo_pub.cpp>
#include <voltage_pub.cpp>
#include <humidity_pub.cpp>
#include <leak_pub.cpp>
#include <pressure_pub.cpp>
#include <gps_pub.cpp>
#include <imu_pub.cpp>

#include <frost_interfaces/msg/nav.h>
#include <Servo.h>
//hello

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
	static volatile int64_t init = -1; \
	if (init == -1) { init = uxr_millis();} \
	if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// micro-ROS objects
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t pub_executor;
rcl_subscription_t subscriber;
rclc_executor_t sub_executor;
rcl_timer_t timer;
frost_interfaces__msg__Nav msg;

// publisher objects
EchoPub echo_pub;
VoltagePub voltage_pub;
HumidityPub humidity_pub;
LeakPub leak_pub;
PressurePub pressure_pub;
GPSPub gps_pub;
IMUPub imu_pub;

// servo, thruster variables
Servo my_servo1;
Servo my_servo2;
Servo my_servo3;
Servo thruster;

// servo, thruster pins and default values;
int servo_pin1 = 9;
int servo_pin2 = 10;
int servo_pin3 = 11;
int thruster_pin = 12;
int default_pos_servo = 90;
int default_pos_thruster = 1500;

int prevServo1 = default_pos_servo;
int prevServo2 = default_pos_servo;
int prevServo3 = default_pos_servo;
int prevThruster = default_pos_thruster;

// states for statemachine in loop function
enum states {
	WAITING_AGENT,
	AGENT_AVAILABLE,
	AGENT_CONNECTED,
	AGENT_DISCONNECTED
} state;

// responds to errors with micro-ROS functions 
void error_loop() {
	while(1) {
		delay(100);
	}
}

// pin setup for servos and thruster
void pin_setup() {

	pinMode(servo_pin1, OUTPUT);
	pinMode(servo_pin2, OUTPUT);
	pinMode(servo_pin3, OUTPUT);
	pinMode(thruster_pin, OUTPUT);

	my_servo1.attach(servo_pin1);
	my_servo2.attach(servo_pin2);
	my_servo3.attach(servo_pin3);
    thruster.attach(thruster_pin);

	my_servo1.write(default_pos_servo);
	my_servo2.write(default_pos_servo);
	my_servo3.write(default_pos_servo);
    thruster.writeMicroseconds(default_pos_thruster);
	delay(7000);
}

// micro-ROS function that publishes all the data to their topics 
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {

	(void) last_call_time;
	if (timer != NULL) {

		//echo_pub.publish();
		//voltage_pub.publish();
		humidity_pub.publish();
		leak_pub.publish();
		pressure_pub.publish();
		//gps_pub.publish();
		//imu_pub.publish();
	}
}

// micro-Ros function that subscribes to navigation positions 
void subscription_callback(const void * servo_msgin) {

	const frost_interfaces__msg__Nav * servo_msg = (const frost_interfaces__msg__Nav  *)servo_msgin;
	if(prevServo1 != servo_msg->servo1){
		my_servo1.write(servo_msg->servo1);
		prevServo1 = servo_msg->servo1;
	}
	if(prevServo2 != servo_msg->servo2){
		my_servo2.write(servo_msg->servo2);
		prevServo2 = servo_msg->servo2;
	}
	if(prevServo3 != servo_msg->servo3){
		my_servo3.write(servo_msg->servo3);
		prevServo3 = servo_msg->servo3;
	}
	if(prevThruster != servo_msg->thruster){
		prevThruster = servo_msg->thruster;
		int thrusterValue = map(servo_msg->thruster, 0, 100, 1500, 2000);
		thruster.writeMicroseconds(thrusterValue);
	}
}

bool create_entities() {

	// the allocator object wraps the dynamic memory allocation and deallocation methods used in micro-ROS 
	allocator = rcl_get_default_allocator();
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

	// synchronize timestamps with the Raspberry Pi
	// after sync, timing should be able to be accessed with "rmw_uros_epoch" functions
	RCCHECK(rmw_uros_sync_session(1000));

	// create publishers
	echo_pub.setup(node);
	voltage_pub.setup(node);
	humidity_pub.setup(node);
	leak_pub.setup(node);
	pressure_pub.setup(node);
	gps_pub.setup(node);
	imu_pub.setup(node);

	// create subscribers
	RCCHECK(rclc_subscription_init_default(
	&subscriber,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Nav),
	"nav_instructions"));

	// create timer (handles periodic publications)
	const unsigned int timer_timeout = 500;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create publisher executor (sends data to micro-ROS topics)
	RCSOFTCHECK(rclc_executor_init(&pub_executor, &support.context, 1, &allocator));
	RCSOFTCHECK(rclc_executor_add_timer(&pub_executor, &timer));

	// create subscriber executor (recieves data from micro_ros topics)
	RCSOFTCHECK(rclc_executor_init(&sub_executor, &support.context, 1, &allocator));
	RCSOFTCHECK(rclc_executor_add_subscription(&sub_executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

	return true;
}

void destroy_entities() {
	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
	(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	// destroy publishers
	echo_pub.destroy(node);
	voltage_pub.destroy(node);
	humidity_pub.destroy(node);
	leak_pub.destroy(node);
	pressure_pub.destroy(node);
	gps_pub.destroy(node);
	imu_pub.destroy(node);

	// destroy everything else
	rcl_subscription_fini(&subscriber, &node);
	rcl_timer_fini(&timer);
	rclc_executor_fini(&sub_executor);
	rclc_executor_fini(&pub_executor);
	rcl_node_fini(&node);
	rclc_support_fini(&support);
}

void setup() {

	Serial.begin(115200);
	set_microros_serial_transports(Serial);
	pin_setup();

	state = WAITING_AGENT;
}

void loop() {

	// state machine to manage connecting and disconnecting the micro-ROS agent
	switch (state) {
		case WAITING_AGENT:
			EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
			break;
		case AGENT_AVAILABLE:
			state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
			if (state == WAITING_AGENT) {
				destroy_entities();
			};
			break; 
		case AGENT_CONNECTED:
			EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
			if (state == AGENT_CONNECTED) {
				// these micro-ROS functions are responsible for processing limited number of pending tasks in the executor's que  
				RCSOFTCHECK(rclc_executor_spin_some(&pub_executor, RCL_MS_TO_NS(100))); 
				RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(100))); 
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
