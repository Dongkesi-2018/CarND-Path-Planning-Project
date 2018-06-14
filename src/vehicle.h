#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <cmath>
#include <string>
#include "helper.h"
#include "lane_map.h"

using std::sqrt;
using std::string;

struct Vehicle {
 public:
  Vehicle() {}

  // Instantiate non-ego
  Vehicle(double x, double y, double vx, double vy, double s, double d, int dummmy, string type="non-ego")
      : x(x), y(y), vx(vx), vy(vy), s(s), d(d), state("CS"), type(type) {
    this->v = sqrt(vx * vx + vy * vy);
    this->lane = (int)d / 4;
    this->a = 0;
  }

  // Instantiate ego
  Vehicle(double x, double y, double s, double d, double yaw, double speed, string type="ego")
      : x(x), y(y), s(s), d(d), lane(lane), state("CS"), type(type) {
    this->v = speed;
    this->a = 0;
    this->lane = int(d / 4);
    this->yaw = deg2rad(yaw);
  }

  // Instantiate Next state of ego
  Vehicle(int lane, double s, double v, double a, string state = "CS") {
    this->lane = lane;
    this->s = s;
    this->d = d;
    this->a = a;
    this->state = state;
  }

  static vector<double> PositionAt(const Vehicle &vehicle, double t);

  ~Vehicle() {}

 public:
  int lane;
  double x, y, vx, vy, s, d, v, a, yaw;
  string state;
  string type;
};

vector<double> Vehicle::PositionAt(const Vehicle &vehicle, double t) {
  /*
    Omit accelorator of non-ego vehicles when get following time position.
  */
  double x = vehicle.x + vehicle.vx * t;
  double y = vehicle.y + vehicle.vy * t;
  double theta = atan2(vehicle.vy, vehicle.vx);
  vector<double> frenet = Map::getInstance()->getFrenet(x, y, theta);
  double s = frenet[0];
  double d = frenet[1];
  return {s, d};
}

#endif
