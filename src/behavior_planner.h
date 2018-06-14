#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <map>
#include <vector>
using std::map;
using std::vector;
class Vehicle;
class BehaviorFSM;

class BehaviorPlanner {
 public:
  vector<double> Solver(map<int, vector<Vehicle>> &predictions, Vehicle &ego);

 private:
  BehaviorFSM fsm_;
};

#endif
