#ifndef LEAK_PUB
#define LEAK_PUB

#include "publisher.hpp"
#include <frost_interfaces/msg/leak.h>
#include <std_msgs/msg/bool.h>

class LeakPub : Publisher {

public:
  void setup(rcl_node_t node);
  void publish();
  using Publisher::destroy;

private:
  bool check_water_leak();
  frost_interfaces__msg__Leak msg;
};

#endif // LEAK_PUB