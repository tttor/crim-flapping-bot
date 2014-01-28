#ifndef BOT_EXPERIMENT_HELPER_H
#define BOT_EXPERIMENT_HELPER_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <mavlink_ros/RadioControl.h>

#include <boost/foreach.hpp>
#include <fstream>

namespace crim {

class Helper {
 public:
  Helper(ros::NodeHandle nh, std::string bag_path): nh_(nh), bag_(new rosbag::Bag(bag_path, rosbag::bagmode::Write)), bag_path_(bag_path) {
    // nothing
  }
  
  ~Helper() {
    bag_->close();
    delete bag_;
  }
  
  /*
   * @brief 
   * Supported topics: "/fcu/rc" (#port=1 that is port= 1), ...
   */
  static bool convert_bag2csv(const std::string& bag_path, const std::string& topic, const std::string& csv_path, const std::string& metadata_path) {
    using namespace std;
    
    ofstream csv_file(csv_path.c_str());
    if ( !csv_file.is_open() ) {
      ROS_ERROR_STREAM("Unable to open file: " << csv_path);
      return false;
    }
        
    rosbag::Bag bag(bag_path, rosbag::bagmode::Read);
    
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      mavlink_ros::RadioControl::ConstPtr i;
      
      if (topic == "/fcu/rc") {
        // Make the metadata
        ofstream metadata_file(metadata_path.c_str());
        if ( !metadata_file.is_open() ) {
          ROS_ERROR_STREAM("Unable to open file: " << metadata_path);
          return false;
        }
        metadata_file << "ch_1_raw" << "," << "ch_2_raw" << "," << "ch_3_raw" << "," << "ch_4_raw" << ","
                      << "ch_5_raw" << "," << "ch_6_raw" << "," << "ch_7_raw" << "," << "ch_8_raw" << ","
                      << "port" << "," << "rssi" << endl;
    
        // Make the data
        i = m.instantiate<mavlink_ros::RadioControl>();
        
        if (i != NULL) {
          csv_file << i->ch1_raw << ",";
          csv_file << i->ch2_raw << ",";
          csv_file << i->ch3_raw << ",";
          csv_file << i->ch4_raw << ",";
          csv_file << i->ch5_raw << ",";
          csv_file << i->ch6_raw << ",";
          csv_file << i->ch7_raw << ",";
          csv_file << i->ch8_raw << ",";
          csv_file << i->port << ",";
          csv_file << i->rssi << endl;
        }
      }
    }
    
    csv_file.close();
    
    return true;
  }
  
  /*
   * @brief This logs the topic of /fcu/rc to a bag file
   * 
   */
  void rc_sub_cb(const mavlink_ros::RadioControl& msg) {
    using namespace std;
    
    bag_->write("/fcu/rc", ros::Time::now(),msg);
  }
  
 private:
  ros::NodeHandle nh_;
  rosbag::Bag* bag_;
  std::string bag_path_;
};

}// using namespace crim

#endif
