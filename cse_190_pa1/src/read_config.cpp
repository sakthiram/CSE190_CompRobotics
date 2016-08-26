#include <fstream>
#include <iostream>
#include "../jsoncpp_example/json/json.h"
#include "../jsoncpp_example/jsoncpp.cpp"

Json::Value read_config() {
  Json::Value config;
  std::ifstream file("/home/sakthi/catkin_ws/configuration.json");
  file >> config;
  return config;
}
