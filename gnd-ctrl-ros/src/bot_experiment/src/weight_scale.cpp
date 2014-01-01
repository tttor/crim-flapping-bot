#include <ros/ros.h>
#include <iostream>

#include <bot_experiment/TimeoutSerial.h>
#include <bot_experiment/WeightScaleData.h>

using namespace std;

namespace crim {
  
class WeightScale {
 public:
  WeightScale(ros::NodeHandle nh, std::string port, size_t baudrate, size_t timeout) 
      : nh_(nh),
        port_(port),
        baudrate_(baudrate),
        timeout_(timeout) {
    pub_ = nh_.advertise<bot_experiment::WeightScaleData>("weight_scale", 1000);
  }
  
  ~WeightScale() {
    // do nothing
  }
  
  void standby() {
    ros::Rate loop_rate(10);

    while (ros::ok()) {
      try {
        TimeoutSerial serial(port_, baudrate_);
        serial.setTimeout(boost::posix_time::seconds(timeout_));

        std::string raw_packet;
        raw_packet = serial.readStringUntil("\n");
        
        packet_ = raw_packet.substr(1,7);
        cerr << "packet_= " << packet_ << endl;
        
        bot_experiment::WeightScaleData data;
        data.weight = boost::lexical_cast<double>(packet_);
        
        pub_.publish(data);
        serial.close();
      } catch(std::exception& e) {
          cerr << "Error: " << e.what() << endl;
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
 private:  
  ros::NodeHandle nh_;   
  std::string port_;
  size_t baudrate_;
  size_t timeout_;
  std::string packet_;
  ros::Publisher pub_;
};

}// namespace crim

int main(int argc, char** argv) {
  ros::init(argc, argv, "scale");
  ros::NodeHandle nh;

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  crim::WeightScale scale(nh, "/dev/ttyUSB0", 9600, 5);
  scale.standby();

  return 0;
}
