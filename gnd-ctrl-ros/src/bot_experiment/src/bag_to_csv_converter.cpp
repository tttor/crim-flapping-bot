#include <ros/ros.h>
#include "helper.hpp"

int main(int argc, char** argv) {
  using namespace std;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  ros::init(argc, argv, "bag_converter");
  ros::NodeHandle nh;
  
  const string bag_path = "/home/tor/crim/crim-flapping-bot/gnd-ctrl-ros/src/bot_experiment/data/test.bag";
  const string csv_path = "/home/tor/crim/crim-flapping-bot/gnd-ctrl-ros/src/bot_experiment/data/rc.csv";
  const string metadata_path = "/home/tor/crim/crim-flapping-bot/gnd-ctrl-ros/src/bot_experiment/data/rc.metadata.csv";
  const string topic = "/fcu/rc";
  
  crim::Helper::convert_bag2csv(bag_path, topic, csv_path, metadata_path);
  
  return 0;
}
