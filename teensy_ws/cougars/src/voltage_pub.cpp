#include "voltage_pub.hpp"

#define VOLTAGE_WARNING 15000
#define VOLTAGE_CRITICAL 13000

static Adafruit_INA260 ina260 = Adafruit_INA260();

void VoltagePub::setup(rcl_node_t node) {
  ina260.begin(INA260_I2CADDR_DEFAULT, &Wire2);

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Volt), "voltage"));

  // set the number of samples to average
  ina260.setAveragingCount(INA260_COUNT_16);
  // set the time over which to measure the current and bus voltage
  ina260.setVoltageConversionTime(INA260_TIME_140_us);
  ina260.setCurrentConversionTime(INA260_TIME_140_us);
}

void VoltagePub::publish() {

  int64_t voltage = ina260.readBusVoltage();
  int64_t current = ina260.readCurrent();
  if (voltage < VOLTAGE_WARNING) {
    msg.voltage = voltage;
    msg.current = current;
    msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}