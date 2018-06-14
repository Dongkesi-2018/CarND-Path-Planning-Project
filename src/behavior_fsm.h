#ifndef BEHAVIOR_FSM_H_
#define BEHAVIOR_FSM_H_
#include <map>
#include <vector>
#include <string>
#include "vehicle.h"

using std::string;
using std::map;
using std::vector;

class BehaviorFSM {
 public:
  BehaviorFSM() {
    // configure();
  }
  map<string, int> lane_direction = {
      {"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

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
  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane,
                          Vehicle &rVehicle);
  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane,
                         Vehicle &rVehicle);
  vector<Vehicle> generate_predictions(int horizon = 2);
  void refresh_ego(Vehicle &ego);
  void realize_next_state(vector<Vehicle> trajectory);
  void configure();
  int find_goal_lane(map<int, vector<Vehicle>> predictions);

 public:
  Vehicle ego_;
  double dt;
  double target_speed;
  double max_acceleration;
  double goal_s;
  int goal_lane;
  int lanes_available;
};

#endif
