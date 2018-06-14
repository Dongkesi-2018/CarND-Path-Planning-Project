#ifndef SENSOR_FUSION_H_
#define SENSOR_FUSION_H_
#include "json.hpp"
#include <vector>

using std::vector;
using nlohmann::json;

struct EgoVehicle {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  EgoVehicle(double x, double y, double s, double d, double yaw, double speed)
    : car_x(x), car_y(y), car_s(s), car_d(d), car_yaw(yaw), car_speed(speed) {}
};

struct NonEgoVehicle {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  NonEgoVehicle(int id, double x, double y, double vx, double vy, double s, double d)
    : id(id), x(x), y(y), vx(vx), vy(vy), s(s), d(d) {}
};

class SensorFusion {
public:
  void Update(json &sensor_data) {
    json &data = sensor_data;
    // update ego
    ego_ = EgoVehicle(data["x"], data["y"], data["s"], data["d"], data["yaw"], data["speed"]);
    // update non-ego
    non_ego_.clear();
    for (auto d : data["sensor_fusion"]) {
      non_ego_.push_back(NonEgoVehicle(d[0], d[1], d[2], d[3], d[4], d[5], d[6]));
    }
  }
  EgoVehicle & get_ego() { return ego_;}
  vector<NonEgoVehicle> & get_non_ego() {return non_ego_;}

private:
  EgoVehicle ego_;
  vector<NonEgoVehicle> non_ego_;
};

#endif