#include "helper/helper.hpp"

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

