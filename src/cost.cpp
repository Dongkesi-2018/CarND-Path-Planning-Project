#include "cost.h"
#include <math.h>
#include <functional>
#include <iterator>
#include <map>
#include "behavior_fsm.h"
#include "config.h"
#include "vehicle.h"

using std::function;
using std::map;
using std::vector;
// TODO: change weights for cost functions.
const double REACH_GOAL = pow(10, 1);
const double EFFICIENCY = 0.9;   // pow(10, 1);
const double LANE_CHANGE = 0.0;  // pow(10, 1);
const double COLLISION = 2;
const double LANE_KEEP = 1;
const double TRAFFIC = 3;

static double goal_distance_cost(const BehaviorFSM& fsm,
                                 const vector<Vehicle>& trajectory,
                                 const map<int, vector<Vehicle>>& predictions,
                                 map<string, double>& data) {
  /*
  Cost increases based on distance of intended lane (for planning a lane change)
  and final lane of trajectory. Cost of being out of goal lane also becomes
  larger as vehicle approaches goal distance. This function is very similar to
  what you have already implemented in the "Implement a Cost Function in C++"
  quiz.
  */
  return 0;
  double cost;
  double distance = data["distance_to_goal"];
  if (distance > 0) {
    cost = 1 - 2 * exp(-(abs(/*2.0 * fsm.goal_lane - */ data["intended_lane"] -
                             data["final_lane"]) /
                         distance));
  } else {
    cost = 1;
  }

  return cost;
}

static double lane_change_cost(const BehaviorFSM& fsm,
                               const vector<Vehicle>& trajectory,
                               const map<int, vector<Vehicle>>& predictions,
                               map<string, double>& data) {
  /*
  Cost increases based on distance of intended lane (for planning a lane change)
  and final lane of trajectory. Cost of being out of goal lane also becomes
  larger as vehicle approaches goal distance. This function is very similar to
  what you have already implemented in the "Implement a Cost Function in C++"
  quiz.
  */
  double cost;
  double delta_d =
      abs(2.0 * fsm.goal_lane - data["intended_lane"] - data["final_lane"]);
  cout << "goal lane / intended / final:" << fsm.goal_lane << ", "
       << data["intended_lane"] << ", " << data["final_lane"] << endl;
  cost = 1 - exp(-(delta_d) / 1);
  cout << "lane_change_cost: " << cost << endl;
  return cost;
}

static double lane_keep_cost(const BehaviorFSM& fsm,
                             const vector<Vehicle>& trajectory,
                             const map<int, vector<Vehicle>>& predictions,
                             map<string, double>& data) {
  /*
  most of time keep lane is a good idea.
  */
  Vehicle vehicle_ahead;
  double cost = 0;
  if (data["intended_lane"] != fsm.ego_.lane &&
      fsm.get_vehicle_ahead(predictions, data["intended_lane"], vehicle_ahead,
                            true)) {
    cost = exp(-1);
  }

  cout << "lane_keep_cost: " << cost << endl;
  return cost;
}

static double lane_speed(const BehaviorFSM& fsm,
                         const map<int, vector<Vehicle>>& predictions,
                         int lane) {
  Vehicle vehicle_ahead;
  if (fsm.get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    return vehicle_ahead.v;
  }
  return -1.0;
}

static double inefficiency_cost(const BehaviorFSM& fsm,
                                const vector<Vehicle>& trajectory,
                                const map<int, vector<Vehicle>>& predictions,
                                map<string, double>& data) {
  /*
  Cost becomes higher for trajectories with intended lane and final lane that
  have traffic slower than vehicle's target speed. You can use the
  lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) function
  to determine the speed for a lane. This function is very similar to what you
  have already implemented in the "Implement a Second Cost Function in C++"
  quiz.
  */
  double proposed_speed_intended =  // trajectory[1].v;
      lane_speed(fsm, predictions, data["intended_lane"]);
  // If no vehicle is in the proposed lane, we can travel at target speed.
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = fsm.target_speed;
  }

  double proposed_speed_final =  // trajectory[0].v;
      lane_speed(fsm, predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
    proposed_speed_final = fsm.target_speed;
  }

  double cost = (2.0 * fsm.target_speed - proposed_speed_intended -
                 proposed_speed_final) /
                fsm.target_speed;

  cout << "inefficiency: gv, iv, fv: " << fsm.target_speed << ", "
       << proposed_speed_intended << ", " << proposed_speed_final << endl;
  cout << "inefficiency_cost: " << cost << endl;
  cost = cost < 0 ? 0 : cost;
  return cost;
}

static double collision_cost(const BehaviorFSM& fsm,
                             const vector<Vehicle>& trajectory,
                             const map<int, vector<Vehicle>>& predictions,
                             map<string, double>& data) {
  /*
  the distance between ego with ahead and behind vehicles, The cost decreases
  with distance.
  */
  double cost = 0;
  Vehicle vehicle_ahead;
  if (fsm.get_vehicle_ahead(predictions, data["intended_lane"], vehicle_ahead,
                            false)) {
    double distance = abs(vehicle_ahead.s - fsm.ego_.s);
    cost += exp(-(distance / fsm.cal_safe_distance(fsm.ego_.v)));
  }
  Vehicle vehicle_behind;
  if (fsm.get_vehicle_behind(predictions, data["intended_lane"], vehicle_behind,
                             true)) {
    double distance = abs(vehicle_behind.s - fsm.ego_.s);
    if (distance < ParameterConfig::safe_distance / 2)
      cost += exp(-(distance / fsm.cal_safe_distance(fsm.ego_.v)));
  }
  cout << "collision_cost: " << cost << endl;
  return cost;
}

static double traffic_cost(const BehaviorFSM& fsm,
                           const vector<Vehicle>& trajectory,
                           const map<int, vector<Vehicle>>& predictions,
                           map<string, double>& data) {
  /*
  blocked by side by side cars, keep lane is a good choice.
  */
  double cost = 0;
  int block = 0;
  Vehicle vehicle_ahead;
  for (int i = 0; i < ParameterConfig::lanes_available; i++) {
    if (fsm.get_vehicle_ahead(predictions, i, vehicle_ahead, true)) {
      block++;
    }
  }
  if (block == ParameterConfig::lanes_available &&
      data["intended_lane"] == fsm.ego_.lane)
    cost = -1;
  cout << "traffic_cost: " << cost << endl;
  return cost;
}

static map<string, double> get_helper_data(
    const BehaviorFSM& fsm, const vector<Vehicle>& trajectory,
    const map<int, vector<Vehicle>>& predictions) {
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
    intended_lane = trajectory_last.lane - 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane + 1;
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

double calculate_cost(const BehaviorFSM& vehicle,
                      const map<int, vector<Vehicle>>& predictions,
                      const vector<Vehicle>& trajectory) {
  /*
  Sum weighted cost functions to get total cost for trajectory.
  */
  map<string, double> trajectory_data =
      get_helper_data(vehicle, trajectory, predictions);
  double cost = 0.0;

  // Add additional cost functions here.
  vector<
      function<double(const BehaviorFSM&, const vector<Vehicle>&,
                      const map<int, vector<Vehicle>>&, map<string, double>&)>>
      cf_list = {traffic_cost, inefficiency_cost, lane_change_cost,
                 collision_cost, lane_keep_cost};
  vector<double> weight_list = {TRAFFIC, EFFICIENCY, LANE_CHANGE, COLLISION,
                                LANE_KEEP};

  for (int i = 0; i < cf_list.size(); i++) {
    double new_cost = weight_list[i] * cf_list[i](vehicle, trajectory,
                                                  predictions, trajectory_data);
    cost += new_cost;
  }

  return cost;
}