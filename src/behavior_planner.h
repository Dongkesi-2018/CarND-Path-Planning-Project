#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <map>
#include <vector>
#include "behavior_fsm.h"
#include "simulator.h"
#include "vehicle.h"
using std::map;
using std::vector;

class BehaviorPlanner {
 public:
  vector<double> Solver(const map<int, vector<Vehicle>>& predictions);

  void update_ego(const Vehicle& ego, Simulator& sim, double dt) {
    fsm_.refresh_ego(ego, sim, dt);
  }

 private:
  BehaviorFSM fsm_;
};

#endif
