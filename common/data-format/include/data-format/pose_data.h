// @author vektor dewanto
#ifndef POSE_DATA_H
#define POSE_DATA_H

#include <string>
#include <stdio.h> // for snprintf()
#include <stdint.h> // for uint16_t, uint8_t, etc
#include <vector>

#include "data-format/string_data.h"

namespace crim {

static const int BUFFER_CAPACITY = 100;

struct PoseData: public StringData {
  PoseData();
  ~PoseData();
  
  void set_parent_frame(const std::string& frame_id);
  void set_frame(const std::string& frame_id);
  void set_origin(const double& x, const double& y, const double& z);
  void set_rotation(const double& roll, const double& pitch, const double& yaw);
};

}// namespace crim

#endif
