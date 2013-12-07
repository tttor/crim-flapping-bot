#ifndef HELPER_H
#define HELPER_H

namespace crim {

class Helper {
 public:
  /**
    @brief Decimal Degrees = Degrees + minutes/60 + seconds/3600
    See: http://andrew.hedges.name/experiments/convert_lat_long/
  */
  static double convert_dms2dec(double dms, char dir);

 private:
};

Helper::convert_dms2dec(double dms, char dir) {
  double dec;
}


}// namespace crim

#endif
