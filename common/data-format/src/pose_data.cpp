#include "data-format/pose_data.h"

using namespace crim;
using namespace std;

PoseData::PoseData() {
  content.resize(4); // 0->id, 1-> frame information, 2-> origin, 3-> rotation
  content.at(0).push_back("POS");// data id
  content.at(1).resize(2);// 0-> parent frame, 1-> this frame
  content.at(2).resize(3);// 0-> x, 1-> y, 2-> z
  content.at(3).resize(3);// 0-> roll, 1-> pitch, 2-> yaw
}

PoseData::~PoseData() {
  // nothing
}

void PoseData::set_parent_frame(const std::string& frame_id) {
  char tmp[BUFFER_CAPACITY];
  
  int status;
  status = snprintf(tmp, BUFFER_CAPACITY, "%s", frame_id.c_str());
  
  if ((status>0) && (status<=BUFFER_CAPACITY)) 
    content.at(1).at(0) = tmp;
}

void PoseData::set_frame(const std::string& frame_id) {
  char tmp[BUFFER_CAPACITY];
  
  int status;
  status = snprintf(tmp, BUFFER_CAPACITY, "%s", frame_id.c_str());
  
  if ((status>0) && (status<=BUFFER_CAPACITY)) 
    content.at(1).at(1) = tmp;
}

void PoseData::set_origin(const double& x, const double& y, const double& z) {
  std::vector<double> origin;
  origin.resize(3);
  origin.at(0) = x;
  origin.at(1) = y;
  origin.at(2) = z;
  
  for (uint8_t i=0; i<origin.size(); ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    status = snprintf(tmp, BUFFER_CAPACITY, "%.5f", origin.at(i));
    
    if ((status>0) && (status<=BUFFER_CAPACITY)) 
      content.at(2).at(i) = tmp;  
  }
}

void PoseData::set_rotation(const double& roll, const double& pitch, const double& yaw) {
  std::vector<double> rotation;
  rotation.resize(3);
  rotation.at(0) = roll;
  rotation.at(1) = pitch;
  rotation.at(2) = yaw;
  
  for (uint8_t i=0; i<rotation.size(); ++i) {
    char tmp[BUFFER_CAPACITY];
    
    int status;
    status = snprintf(tmp, BUFFER_CAPACITY, "%.5f", rotation.at(i));
    
    if ((status>0) && (status<=BUFFER_CAPACITY)) 
      content.at(3).at(i) = tmp;  
  }
}
