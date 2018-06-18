#include "behavior_planner.h"
#include <map>
#include <vector>
#include "vehicle.h"
#include "behavior_fsm.h"
#include "simulator.h"


using std::map;
using std::vector;

vector<double> BehaviorPlanner::Solver(map<int, vector<Vehicle> > &predictions) {
  //Get Best Prediciton by FSM
  vector<Vehicle> trajectory = fsm_.choose_next_state(predictions);
  fsm_.realize_next_state(trajectory);
  // TODO: After behavior planner, I want get lane and speed of ego car.
  double goal_lane = trajectory[1].lane;
  double ref_vel = trajectory[1].v;
  cout << "lane:" << goal_lane << " vel: " << ref_vel << endl;
  return {goal_lane, ref_vel};
}
