#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <frost_interfaces/msg/humid.h>
#include <publisher.cpp>
#include <std_msgs/msg/int64_multi_array.h>
#define DHTPIN 4
#define DHTTYPE DHT22

class HumidityPub : Publisher {

public:
  void setup(rcl_node_t node) {

    dht.begin();

    RCCHECK(rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(frost_interfaces, msg, Humid), "humidity"));

    humidity_calibrate();
  }

  void publish() {

    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature(true);
    if (humidity > humidity_threshold) {
      msg.humidity = humidity;
      msg.temp = temperature;
      msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
  }

  using Publisher::destroy;

private:
  DHT dht = DHT(DHTPIN, DHTTYPE);

  frost_interfaces__msg__Humid msg;

  double humidity_threshold = 50.00;

  void humidity_calibrate() {
    // Clayton put code here to measure humidity
    //@ClaytonSmith
  }
};