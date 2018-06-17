#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <cmath>
#include <map>
#include <string>
#include "helper.h"
#include "lane_map.h"
#include "trace.h"
using std::map;
using std::sqrt;
using std::string;

struct Vehicle {
 public:
  Vehicle() : x(0), y(0), vx(0), vy(0), ax(0), ay(0), yaw(0), \
              s(0), d(0), v(0), a(0),  state("CS"), type("unknown") {}

  // Instantiate non-ego
  Vehicle(double x, double y, double vx, double vy, double s, double d, int dummmy)
          : x(x), y(y), vx(vx), vy(vy), ax(0), ay(0), \
            s(s), d(d), a(0), state("CS"), type("non-ego") {
    this->yaw = atan2(vy, vx);  
    this->v = sqrt(vx * vx + vy * vy);
    this->lane = d2lane(d);
  }

  // Instantiate ego
  Vehicle(double x, double y, double s, double d, double yaw, double speed) \
      : x(x), y(y), ax(0), ay(0), \
        s(s), d(d), a(0), state("CS"), type("ego") {
    this->v = mph2mps(speed);
    this->lane = d2lane(d);
    this->yaw = deg2rad(yaw);

    auto vc = speed2v(this->yaw, this->v);
    this->vx = vc[0];
    this->vy = vc[1];
  }

  // Instantiate Next state of ego
  Vehicle(int lane, double s, double v, double a, string state = "CS") {
    this->lane = lane;
    this->s = s;
    this->d = this->lane * 4 + 2;
    this->v = v;
    this->a = a;

    
    this->state = state;
    this->type = "ego";
  }

  static vector<double> PositionAt(const Vehicle &vehicle, double t);

  void print(string head) const;

  int d2lane(double d) {
    return (int)(d / 4.0);
  }

  vector<double> speed2v(double yaw, double speed) {
    double vx = cos(yaw) * speed;
    double vy = sin(yaw) * speed;
    return {vx, vy};
  }
  /* Unit:               m,        m,        m,        m,        deg,        mph,          s*/
  void update_ego(double x, double y, double s, double d, double yaw, double speed, double dt) {
    // Cartesian
    this->x = x;
    this->y = y; 
    this->yaw = deg2rad(yaw);
    double new_speed = mph2mps(speed);
    auto vc = speed2v(this->yaw, new_speed);
    double vx = vc[0];
    double vy = vc[1];
    this->ax = (vx - this->vx) / dt;
    this->ay = (vy - this->vy) / dt;
    this->vx = vx;
    this->vy = vy;

    // Frenet
    this->s = s;
    this->d = d;
    this->a = (new_speed - this->v) / dt;
    this->v = new_speed;

    // other
    this->lane = d2lane(d);
    this->type = "ego";
  }

  void update_ego(const Vehicle &ego, double dt) {
        // Cartesian
    this->x = ego.x;
    this->y = ego.y; 
    this->yaw = ego.yaw;
    this->ax = (ego.vx - this->vx) / dt;
    this->ay = (ego.vy - this->vy) / dt;
    this->vx = ego.vx;
    this->vy = ego.vy;

    // Frenet
    this->s = ego.s;
    this->d = ego.d;
    this->a = (ego.v - this->v) / dt;
    this->v = ego.v;

    // other
    this->lane = ego.lane;
    this->type = "ego";
  }

  /* Unit:                   m,        m,       m/s,       m/s,       m,         m,         s*/
  void update_non_ego(double x, double y, double vx, double vy, double s, double d, double dt) {
    // Cartesian
    this->x = x;
    this->y = y; 
    this->yaw = atan2(vy, vx);
    this->ax = (vx - this->vx) / dt;
    this->ay = (vy - this->vy) / dt;
    this->vx = vx;
    this->vy = vy;

    // Frenet
    this->s = s;
    this->d = d;
    double new_v = sqrt(vx * vx + vy * vy);
    this->a = (new_v - this->v) / dt;
    this->v = new_v;  

    // other
    this->lane = d2lane(d);  
    this->type = "non-ego";
  }

  ~Vehicle() {}

 public:
  int lane;
  double x, y, vx, vy, ax, ay, yaw;
  double s, d, v, a;
  string state;
  string type;
};

#endif
