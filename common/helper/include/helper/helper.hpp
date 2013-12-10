#ifndef HELPER_H
#define HELPER_H

#include <boost/lexical_cast.hpp>

namespace crim {

class Helper {
 public:
  /**
    @brief Decimal Degrees = Degrees + minutes/60 + seconds/3600
    See: http://andrew.hedges.name/experiments/convert_lat_long/
  */
  static double convert_dms_to_dec(const std::string& dms, const std::string& dir);

 private:
};

}// namespace crim

#endif
