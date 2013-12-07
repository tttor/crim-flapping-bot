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

double Helper::convert_dms_to_dec(const std::string& dms, const std::string& dir) {
  const size_t N_MINUTE_DIGIT = 8;// including the point

  double minute;
  minute = boost::lexical_cast<double>(dms.substr(dms.length()-N_MINUTE_DIGIT,dms.length()));

  double degree;
  degree = boost::lexical_cast<double>(dms.substr(0,dms.length()-N_MINUTE_DIGIT));

  double dec;
  dec = degree + minute/60;

  if (dir=="S" or dir=="W")
    dec *= -1;

  return dec;
}

}// namespace crim

#endif
