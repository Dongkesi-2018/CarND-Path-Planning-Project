#ifndef HELPER_H_
#define HELPER_H_
#include <math.h>

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return std::atan(1)*4; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

#endif
