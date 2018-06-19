#ifndef COST_H_
#define COST_H_
#include <map>
#include <vector>
#include "behavior_fsm.h"
#include "vehicle.h"

using std::map;
using std::vector;

double calculate_cost(const BehaviorFSM& vehicle,
                      const map<int, vector<Vehicle>>& predictions,
                      const vector<Vehicle>& trajectory);

#endif