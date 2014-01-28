#include <ros/ros.h>
#include "helper.hpp"

//tf::TransformBroadcaster crim::Helper::tf_broadcaster_ = tf::TransformBroadcaster();

int main(int argc, char** argv) {
  using namespace std;
  
  ros::init(argc, argv, "helper");
  ros::NodeHandle nh("/helper");
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  
  crim::Helper helper(nh, std::string("/home/tor/crim/crim-flapping-bot/gnd-ctrl-ros/src/bot_experiment/data/test.bag"));
  
  ros::Subscriber rc_sub = nh.subscribe("/fcu/rc", 1, &crim::Helper::rc_sub_cb, &helper);
  ros::Subscriber imu_sub = nh.subscribe("/fcu/imu", 1, &crim::Helper::imu_sub_cb, &helper);
  
  ros::spin();
  
  return 0;
}
