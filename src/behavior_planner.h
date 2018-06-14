#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <map>
#include <vector>
#include "vehicle.h"
#include "behavior_fsm.h"

using std::map;
using std::vector;

class BehaviorPlanner {
 public:
  vector<double> Solver(map<int, vector<Vehicle>> &predictions, Vehicle &ego);

 private:
  BehaviorFSM fsm_;
};

#endif
