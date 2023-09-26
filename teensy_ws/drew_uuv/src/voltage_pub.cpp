#include <Adafruit_INA260.h>
#include <frost_interfaces/msg/volt.h>
#include <publisher.cpp>

#define VOLTAGE_WARNING 15

#define VOLTAGE_CRITICAL 13

class VoltagePub : Publisher {

public:
  void setup(rcl_node_t node) {

    RCCHECK(rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Volt), "voltage"));
  }

  void publish() {

    int64_t voltage = ina260.readBusVoltage();
    int64_t current = ina260.readCurrent();
    if (voltage < VOLTAGE_WARNING) {
      msg.voltage = voltage;
      msg.current = current;
      msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
  }

  using Publisher::destroy;

private:
  Adafruit_INA260 ina260 = Adafruit_INA260();

  frost_interfaces__msg__Volt msg;
};