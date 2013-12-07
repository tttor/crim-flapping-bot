#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>

#include <serial_port/GndCtrlPacket.h>
#include <data-format/string_data.h>
#include <helper/helper.hpp>

#include <sstream>

namespace crim {

class SerialBridge {
 public:
  SerialBridge(ros::NodeHandle nh): nh_(nh) {
    gps_data_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("gps_data", 1000);
    packet_ = crim::GndCtrlPacket("/dev/ttyUSB0", 9600, 5);

    ROS_INFO("serial_bridge: up and running ;)");
  }

  ~SerialBridge(){
  }

  void standby() {
    using namespace std;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
      size_t status;
      status = packet_.receive();

      if (status==0) { //is good
        ROS_DEBUG_STREAM("received, status= " << status);
        crim::StringData data;
        data = packet_.unwrap();

        publish(data);
      } else {
        ROS_DEBUG_STREAM("received, but corrupted; status= " << status);
      }

      //
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
      int8_t status;
      status = boost::lexical_cast<int8_t>(data.content.at(5).at(0));

      if (status) {
        sensor_msgs::NavSatFix gps_data;
        gps_data.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        gps_data.latitude = crim::Helper::convert_dms_to_dec(data.content.at(3).at(0), data.content.at(3).at(1));
        gps_data.longitude = crim::Helper::convert_dms_to_dec(data.content.at(3).at(2), data.content.at(3).at(3));
        gps_data.altitude = boost::lexical_cast<double>(data.content.at(3).at(4));

        gps_data_pub_.publish(gps_data);
      }
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher gps_data_pub_;
  crim::GndCtrlPacket packet_;
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
