#include "humidity_pub.hpp"

#define DHTPIN 4
#define DHTTYPE DHT22
#define HUMID_THRES 10.0
#define AVG_COUNT 10
#define AVG_DEC 0.1
#define AVG_DELAY 10

// cross-function variables
static DHT dht = DHT(DHTPIN, DHTTYPE);
static float humidity_on_init;

void HumidityPub::setup(rcl_node_t node) {

  dht.begin();

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Humid), "humidity"));

  humidity_calibrate();
}

void HumidityPub::publish() {

  float humidity = dht.readHumidity() - humidity_on_init;
  float temperature = dht.readTemperature(true);
  if (humidity > HUMID_THRES) {
    msg.humidity = humidity;
    msg.temp = temperature;
    msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void HumidityPub::humidity_calibrate() {

  float sum_humidity_on_init = 0;
  for (unsigned int i = 0; i < AVG_COUNT; i++) {
    sum_humidity_on_init += dht.readHumidity();
    delay(AVG_DELAY);
  }
  humidity_on_init = sum_humidity_on_init * AVG_DEC;
}