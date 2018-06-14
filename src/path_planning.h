#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_
#include <map>
#include <vector>
#include "json.hpp"
#include "behavior_planner.h"
#include "sensor_fusion.h"
#include "simulator.h"
#include "prediction.h"
#include "trajectory.h"

using nlohmann::json;
using std::map;
using std::vector;

class PathPlanning {
 public:
  PathPlanning() {}
  void Solver(json &sensor_data);
  void UpdateVehicles();
  void update_ego(EgoVehicle &ego);
  void update_non_ego(NonEgoVehicle &non_ego);
  vector<double> &get_next_x_vals() { return next_x_vals; }
  vector<double> &get_next_y_vals() { return next_y_vals; }

 private:
  SensorFusion sensor_;
  Simulator sim_;
  map<int, Vehicle> non_ego_;
  Vehicle ego_;
  Prediction pred_;
  BehaviorPlanner behavior_;
  Trajectory traj_;
  vector<double> next_x_vals;
  vector<double> next_y_vals;
};

#endif
