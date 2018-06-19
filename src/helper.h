#ifndef HELPER_H_
#define HELPER_H_
#include <math.h>
#include <iostream>
#include <string>

using std::cout;
using std::endl;
using std::string;
// For converting back and forth between radians and degrees.
inline constexpr double pi() { return std::atan(1) * 4; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double mph2mps(double v) { return v * 0.44704; }
inline double mps2mph(double v) { return v * 2.237136; }
template <typename T>
inline void print_bar(char c, T header) {
  string str(10, c);
  cout << str << " " << header << " " << str << endl;
}

#endif
