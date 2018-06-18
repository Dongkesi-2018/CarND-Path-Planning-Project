#ifndef BEHAVIOR_FSM_H_
#define BEHAVIOR_FSM_H_
#include <map>
#include <string>
#include <vector>
#include "simulator.h"
#include "vehicle.h"
using std::map;
using std::string;
using std::vector;

class BehaviorFSM {
 public:
  BehaviorFSM() { configure(); prev_size = 0;}
  map<string, int> lane_direction = {
      {"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);
  vector<string> successor_states();
  vector<Vehicle> generate_trajectory(string state,
                                      map<int, vector<Vehicle>> predictions);
  vector<double> get_kinematics(map<int, vector<Vehicle>> predictions,
                                int lane);
  vector<Vehicle> constant_speed_trajectory();
  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);
  vector<Vehicle> lane_change_trajectory(string state,
                                         map<int, vector<Vehicle>> predictions);
  vector<Vehicle> prep_lane_change_trajectory(
      string state, map<int, vector<Vehicle>> predictions);
  void increment(double dt);
  bool get_vehicle_behind(const map<int, vector<Vehicle>> &predictions, int lane,
                          Vehicle &rVehicle) const;
  bool get_vehicle_ahead(const map<int, vector<Vehicle>> &predictions, int lane,
                         Vehicle &rVehicle) const;
  vector<Vehicle> generate_predictions(int horizon = 2);
  void refresh_ego(const Vehicle &ego, Simulator &sim, double dt);
  void realize_next_state(vector<Vehicle> trajectory);
  void configure();
  int find_goal_lane(map<int, vector<Vehicle>> predictions);
  double cal_safe_distance(double v) const;
  Vehicle &get_ego() {return ego_;}

 public:
  Vehicle ego_;
  double dt;
  double end_s;
  int prev_size;
  double target_speed;
  double max_acceleration;
  double goal_s;
  int goal_lane;
  int lanes_available;
};

#endif
