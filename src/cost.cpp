#include "cost.h"
#include <math.h>
#include <functional>
#include <iterator>
#include <map>
#include "behavior_fsm.h"
#include "vehicle.h"

using std::function;
using std::map;
using std::vector;
// TODO: change weights for cost functions.
const double REACH_GOAL = pow(10, 6);
const double EFFICIENCY = pow(10, 5);

/*
Here we have provided two possible suggestions for cost functions, but feel free
to use your own! The weighted cost over all cost functions is computed in
calculate_cost. The data from get_helper_data will be very useful in your
implementation of the cost functions below. Please see get_helper_data for
details on how the helper data is computed.
*/

double goal_distance_cost(const BehaviorFSM &fsm,
                          const vector<Vehicle> &trajectory,
                          const map<int, vector<Vehicle>> &predictions,
                          map<string, double> &data) {
  /*
  Cost increases based on distance of intended lane (for planning a lane change)
  and final lane of trajectory. Cost of being out of goal lane also becomes
  larger as vehicle approaches goal distance. This function is very similar to
  what you have already implemented in the "Implement a Cost Function in C++"
  quiz.
  */
  double cost;
  double distance = data["distance_to_goal"];
  if (distance > 0) {
    cost = 1 - 2 * exp(-(abs(/*2.0 * fsm.goal_lane - */data["intended_lane"] -
                             data["final_lane"]) / distance));
  } else {
    cost = 1;
  }
  return cost;
}

double inefficiency_cost(const BehaviorFSM &fsm,
                         const vector<Vehicle> &trajectory,
                         const map<int, vector<Vehicle>> &predictions,
                         map<string, double> &data) {
  /*
  Cost becomes higher for trajectories with intended lane and final lane that
  have traffic slower than vehicle's target speed. You can use the
  lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) function
  to determine the speed for a lane. This function is very similar to what you
  have already implemented in the "Implement a Second Cost Function in C++"
  quiz.
  */
  double proposed_speed_intended =
      lane_speed(predictions, data["intended_lane"]);
  // If no vehicle is in the proposed lane, we can travel at target speed.
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = fsm.target_speed;
  }

  double proposed_speed_final = lane_speed(predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
    proposed_speed_final = fsm.target_speed;
  }

  double cost = (2.0 * fsm.target_speed - proposed_speed_intended -
                 proposed_speed_final) /
                fsm.target_speed;

  return cost;
}

double lane_speed(const map<int, vector<Vehicle>> &predictions, int lane) {
  /*
  All non ego vehicles in a lane have the same speed, so to get the speed limit
  for a lane, we can just find one vehicle in that lane.
  */
  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    int key = it->first;
    Vehicle vehicle = it->second[0];
    if (vehicle.lane == lane && key != -1) {
      return vehicle.v;
    }
  }
  // Found no vehicle in the lane
  return -1.0;
}

double calculate_cost(const BehaviorFSM &vehicle,
                      const map<int, vector<Vehicle>> &predictions,
                      const vector<Vehicle> &trajectory) {
  /*
  Sum weighted cost functions to get total cost for trajectory.
  */
  map<string, double> trajectory_data =
      get_helper_data(vehicle, trajectory, predictions);
  double cost = 0.0;

  // Add additional cost functions here.
  vector<function<double(const BehaviorFSM &, const vector<Vehicle> &,
                         const map<int, vector<Vehicle>> &,
                         map<string, double> &)>>
      cf_list = {goal_distance_cost, inefficiency_cost};
  vector<double> weight_list = {REACH_GOAL, EFFICIENCY};

  for (int i = 0; i < cf_list.size(); i++) {
    double new_cost = weight_list[i] * cf_list[i](vehicle, trajectory,
                                                  predictions, trajectory_data);
    cost += new_cost;
  }

  return cost;
}

map<string, double> get_helper_data(
    const BehaviorFSM &fsm, const vector<Vehicle> &trajectory,
    const map<int, vector<Vehicle>> &predictions) {
  /*
  Generate helper data to use in cost functions:
  indended_lane: +/- 1 from the current lane if the ehicle is planning or
  executing a lane change. final_lane: The lane of the vehicle at the end of the
  trajectory. The lane is unchanged for KL and PLCL/PLCR trajectories.
  distance_to_goal: The s distance of the vehicle to the goal.

  Note that indended_lane and final_lane are both included to help differentiate
  between planning and executing a lane change in the cost functions.
  */
  map<string, double> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  double intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else {
    intended_lane = trajectory_last.lane;
  }

  double distance_to_goal = fsm.goal_s - trajectory_last.s;
  double final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
  return trajectory_data;
}