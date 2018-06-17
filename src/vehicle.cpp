#include "vehicle.h"
#include <iostream>
#include <vector>
#include "config.h"
using std::cout;
using std::endl;
using std::vector;

vector<double> Vehicle::PositionAt(const Vehicle &vehicle, double t) {
  /*
    Omit accelorator of non-ego vehicles when get following time position.
  */
  double x = vehicle.x + vehicle.vx * t + 0.5 * vehicle.ax * t * t;
  double y = vehicle.y + vehicle.vy * t + 0.5 * vehicle.ay * t * t;
  double vx = vehicle.vx + vehicle.ax * t;
  double vy = vehicle.vy + vehicle.ay * t;
  double theta = atan2(vehicle.vy, vehicle.vx);
  vector<double> frenet = Map::getInstance().getFrenet(x, y, theta);
  double s = frenet[0];
  double d = frenet[1];
  if (d < 0) d = 0;
  if (d > ParameterConfig::lanes_available * 4)
    d = ParameterConfig::lanes_available * 4 - 1;
  return {x, y, vx, vy, s, d};

  // vehicle.print("Before PositionAt");
  // cout << "vehicle.t:" << t << endl;
  // cout << "x: " << x << " y: " << y << " theta: " << theta << " s: " << s <<
  // " d: " << d << endl;
}

void Vehicle::print(string head) const {
  cout << "----------- " << head << " -----------" << endl;
  // if (type == "ego") {
  //   cout << "car.lane:  " << lane << endl;
  //   cout << "car.s:     " << s << endl;
  //   cout << "car.v:     " << v << endl;
  //   cout << "car.a:     " << a << endl;
  //   cout << "car.state: " << state << endl;
  //   cout << "car.type:  " << type << endl;
  // } else {
    cout << "car.lane:  " << lane << endl;
    cout << "car.x:     " << x << endl;
    cout << "car.y:     " << y << endl;
    cout << "car.vx:    " << vx << endl;
    cout << "car.vy:    " << vy << endl;
    cout << "car.ax:    " << ax << endl;
    cout << "car.ay:    " << ay << endl;
    cout << "car.yaw:   " << yaw << endl;
    cout << "car.s:     " << s << endl;
    cout << "car.d:     " << d << endl;
    cout << "car.v:     " << v << endl;
    cout << "car.a:     " << a << endl;
    cout << "car.state: " << state << endl;
    cout << "car.type:  " << type << endl;
  // }
}