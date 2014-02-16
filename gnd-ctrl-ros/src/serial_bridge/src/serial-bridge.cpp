#include <ros/ros.h>
#include <log4cxx/logger.h>
#include <std_msgs/String.h>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>

#include <serial_port/GndCtrlPacketHandler.h>
#include <data-format/string_data.h>
#include <helper/helper.hpp>

#include <serial_bridge/RCData.h>

#include <sstream>
#include <iostream>

using namespace std;

namespace crim {

class SerialBridge {
 public:
  SerialBridge(ros::NodeHandle nh): nh_(nh) {
    gps_data_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("gps_data", 1000);
    ch_1_rc_data_pub_ = nh_.advertise<serial_bridge::RCData>("rc_data", 1000);
    
    packet_handler_ = crim::GndCtrlPacketHandler("/dev/ttyUSB0", 9600, 5);
    ROS_INFO("serial_bridge: up and running ;)");
  }

  ~SerialBridge(){
  }

  void standby() {
    using namespace std;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
      size_t status;
      status = packet_handler_.receive();

      if (status==0) { //is good
        ROS_DEBUG_STREAM("received, status= " << status);
      
        crim::StringData data;
        data = packet_handler_.unwrap();

        publish(data);
      } else {
        ROS_WARN_STREAM("BAD packet: status= " << status);
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

 private:
  void publish(const crim::StringData& data) {
    using namespace std;

    // dispatch the data based-on ID then publish accordingly
    string id;
    id = data.content.at(0).at(0);

    if (id=="GPS") {
      bool status = 0;
      status = boost::lexical_cast<bool>(data.content.at(5).at(0));

      if (status) {
        ROS_DEBUG("GPS data received, with FIX");

        sensor_msgs::NavSatFix gps_data;
        gps_data.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;// # unaugmented fix
        gps_data.latitude = crim::Helper::convert_dms_to_dec(data.content.at(3).at(0), data.content.at(3).at(1));
        gps_data.longitude = crim::Helper::convert_dms_to_dec(data.content.at(3).at(2), data.content.at(3).at(3));
        gps_data.altitude = boost::lexical_cast<double>(data.content.at(3).at(4));

        gps_data_pub_.publish(gps_data);
      } else {
        ROS_DEBUG("GPS data received, but NO FIX");
      }
    } else if (id=="RCS") {
      size_t n_ch = data.content.size() - 1;// minus one for the header field
      
      serial_bridge::RCData rc_data;
      rc_data.PPMs.resize(n_ch);
      rc_data.init_PPMs.resize(n_ch);
      
      for (size_t i=0; i<n_ch; ++i) {
        rc_data.init_PPMs.at(i) = boost::lexical_cast<size_t>(data.content.at(i+1).at(0));
        rc_data.PPMs.at(i) = boost::lexical_cast<size_t>(data.content.at(i+1).at(1));
      }
      
      ch_1_rc_data_pub_.publish(rc_data);
    } else if (id=="POS") {
      static tf::TransformBroadcaster tf_bc;
      tf::Transform transform;
      
      std::string parent_frame, frame;
      parent_frame = data.content.at(1).at(0);
      frame = data.content.at(1).at(1);
      
      double x, y, z;
      x = boost::lexical_cast<double>(data.content.at(2).at(0));
      y = boost::lexical_cast<double>(data.content.at(2).at(1));
      z = boost::lexical_cast<double>(data.content.at(2).at(2));
      transform.setOrigin( tf::Vector3(x, y, z) );
      
      double roll, pitch, yaw;
      roll = boost::lexical_cast<double>(data.content.at(3).at(0));
      pitch = boost::lexical_cast<double>(data.content.at(3).at(1));
      yaw = boost::lexical_cast<double>(data.content.at(3).at(2));
      transform.setRotation( tf::Quaternion(roll, pitch, yaw) );
      
      tf_bc.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, frame));
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher gps_data_pub_;
  ros::Publisher ch_1_rc_data_pub_ ;
  crim::GndCtrlPacketHandler packet_handler_;
};

}// namespace crim

int main(int argc, char** argv) {
  ros::init(argc, argv, "serial_bridge");
  ros::NodeHandle nh;

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  crim::SerialBridge serial_bridge(nh);
  serial_bridge.standby();

  return 0;
}
