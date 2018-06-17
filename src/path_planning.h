#ifndef PATH_PLANNING_H_
#define PATH_PLANNING_H_
#include <map>
#include <vector>
#include <chrono>
#include "json.hpp"
#include "behavior_planner.h"
#include "sensor_fusion.h"
#include "simulator.h"
#include "prediction.h"
#include "trajectory.h"

using nlohmann::json;
using std::map;
using std::vector;


class PathPlanning
{
public:
  PathPlanning() {
    dt = 0;
    start = std::chrono::system_clock::now();
  }
  void Solver(json &sensor_data);
  void UpdateVehicles();
  void update_ego(EgoVehicle &ego);
  void update_non_ego(NonEgoVehicle &non_ego);
  vector<double> &get_next_x_vals() { return traj_.get_next_x_vals(); }
  vector<double> &get_next_y_vals() { return traj_.get_next_y_vals(); }
  void print_vehicle(string head, Vehicle &car);
  void print_vehicle();
  void print_vehicle(map<int, vector<Vehicle> > &vehicle_pred);
  double get_dt() {
    auto end = std::chrono::system_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000000.0;
    start = end;
    return (double)dt;
  }
private:
  double dt;
  std::chrono::system_clock::time_point start;
  SensorFusion sensor_;
  Simulator sim_;
  map<int, Vehicle> non_ego_;
  Prediction pred_;
  BehaviorPlanner behavior_;
  Trajectory traj_;
};

#endif
