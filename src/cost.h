#ifndef COST_H_
#define COST_H_
#include <map>
#include <vector>
#include <string>
#include "behavior_fsm.h"
#include "vehicle.h"

using std::vector;
using std::map;
using std::string;


double calculate_cost(const BehaviorFSM & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory);

double goal_distance_cost(const BehaviorFSM & vehicle,  const vector<Vehicle> & trajectory,  const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double inefficiency_cost(const BehaviorFSM & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);

map<string, double> get_helper_data(const BehaviorFSM & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions);

#endif