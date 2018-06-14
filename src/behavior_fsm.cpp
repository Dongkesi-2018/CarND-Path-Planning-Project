#include "behavior_fsm.h"
#include <map>
#include <vector>
#include <algorithm>
#include "vehicle.h"
#include "cost.h"
#include "config.h"
#include "helper.h"
#include "prediction.h"

using std::map;
using std::vector;
using std::min;


vector<Vehicle> BehaviorFSM::choose_next_state(
    map<int, vector<Vehicle>> predictions) {
  /*
    

    ***Here you can implement the transition_function code from the Behavior
    Planning Pseudocode classroom concept.***
    

    INPUT: A predictions map. This is a map using vehicle id as keys with
    predicted vehicle trajectories as values. A trajectory is a vector of
    Vehicle objects. The first item in the trajectory represents the vehicle at
    the current timestep. The second item in the trajectory represents the
    vehicle one timestep in the future. OUTPUT: The the best (lowest cost)
    trajectory for the ego vehicle corresponding to the next ego vehicle state.

    Functions that will be useful:
    1. successor_states() - Uses the current state to return a vector of
    possible successor states for the finite state machine.
    2. generate_trajectory(string state, map<int, vector<Vehicle>> predictions)
    - Returns a vector of Vehicle objects representing a vehicle trajectory,
    given a state and predictions. Note that trajectory vectors might have size
    0 if no possible trajectory exists for the state.
    3. calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions,
    vector<Vehicle> trajectory) - Included from cost.cpp, computes the cost for
    a trajectory.
    */

  // TODO: Your solution here.
  vector<string> states = successor_states();
  double cost;
  vector<double> costs;
  vector<string> final_states;
  vector<vector<Vehicle>> final_trajectories;

  for (auto it = states.begin(); it != states.end(); it++) {
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    if (trajectory.size() != 0) {
      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }

  vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  // TODO: Change return value here:
  return final_trajectories[best_idx];
}

vector<string> BehaviorFSM::successor_states() {
  /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
  vector<string> states;
  states.push_back("KL");
  string state = ego_.state;
  if (state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (ego_.lane != lanes_available - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (ego_.lane != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> BehaviorFSM::generate_trajectory(
    string state, map<int, vector<Vehicle>> predictions) {
  /*
    Given a possible next state, generate the appropriate trajectory to realize
    the next state.
    */
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

vector<double> BehaviorFSM::get_kinematics(map<int, vector<Vehicle>> predictions,
                                      int lane) {
  /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
  double max_velocity_accel_limit = this->max_acceleration * dt + ego_.v;
  double new_position;
  double new_velocity;
  double new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
      new_velocity = vehicle_ahead.v;  // must travel at the speed of traffic,
                                       // regardless of preferred buffer
    } else {
      double max_velocity_in_front =
          ((vehicle_ahead.s - ego_.s - ParameterConfig::preferred_buffer) +
          vehicle_ahead.v * dt - 0.5 * (ego_.a) * dt * dt) / dt;
      new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit),
                         this->target_speed);
    }
  } else {
    new_velocity = min(max_velocity_accel_limit, this->target_speed);
  }

  new_accel = (new_velocity - ego_.v) / dt;  // Equation: (v_1 - v_0)/t = acceleration
  new_position = ego_.s + new_velocity * dt + new_accel / 2.0 * dt * dt;
  
  return {new_position, new_velocity, new_accel};
}

vector<Vehicle> BehaviorFSM::constant_speed_trajectory() {
  /*
    Generate a constant speed trajectory.
    */
  // Predict the next 1 second
  vector<double> frenet = Vehicle::PositionAt(ego_, dt);
  double next_pos = frenet[0];
  vector<Vehicle> trajectory = {
      Vehicle(ego_.lane, ego_.s, ego_.v, ego_.a, ego_.state),
      Vehicle(ego_.lane, next_pos, ego_.v, 0, ego_.state)};
  return trajectory;
}

vector<Vehicle> BehaviorFSM::keep_lane_trajectory(
    map<int, vector<Vehicle>> predictions) {
  /*
    Generate a keep lane trajectory.
    */
  vector<Vehicle> trajectory = {
      Vehicle(ego_.lane, ego_.s, ego_.v, ego_.a, ego_.state)};
  vector<double> kinematics = get_kinematics(predictions, ego_.lane);
  double new_s = kinematics[0];
  double new_v = kinematics[1];
  double new_a = kinematics[2];
  trajectory.push_back(Vehicle(ego_.lane, new_s, new_v, new_a, "KL"));
  return trajectory;
}

vector<Vehicle> BehaviorFSM::prep_lane_change_trajectory(
    string state, map<int, vector<Vehicle>> predictions) {
  /*
    Generate a trajectory preparing for a lane change.
    */
  double new_s;
  double new_v;
  double new_a;
  Vehicle vehicle_behind;
  int new_lane = ego_.lane + lane_direction[state];
  vector<Vehicle> trajectory = {
      Vehicle(ego_.lane, ego_.s, ego_.v, ego_.a, ego_.state)};
  vector<double> curr_lane_new_kinematics =
      get_kinematics(predictions, ego_.lane);

  if (get_vehicle_behind(predictions, ego_.lane, vehicle_behind)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];
  } else {
    vector<double> best_kinematics;
    vector<double> next_lane_new_kinematics =
        get_kinematics(predictions, new_lane);
    // Choose kinematics with lowest velocity.
    //TODO: this should be ">"
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  trajectory.push_back(Vehicle(ego_.lane, new_s, new_v, new_a, state));
  return trajectory;
}

vector<Vehicle> BehaviorFSM::lane_change_trajectory(
    string state, map<int, vector<Vehicle>> predictions) {
  /*
    Generate a lane change trajectory.
    */
  int new_lane = ego_.lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies that
  // spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    // Security distance for change lane
    if (abs(next_lane_vehicle.s - ego_.s) < ParameterConfig::safe_distance && next_lane_vehicle.lane == new_lane) {
      // If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  trajectory.push_back(
      Vehicle(ego_.lane, ego_.s, ego_.v, ego_.a, ego_.state));
  vector<double> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(
      Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
  return trajectory;
}


bool BehaviorFSM::get_vehicle_behind(map<int, vector<Vehicle>> predictions,
                                 int lane, Vehicle &rVehicle) {
  /*
    Returns a true if a vehicle is found behind the current vehicle, false
    otherwise. The passed reference rVehicle is updated if a vehicle is found.
    */
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == lane && temp_vehicle.s < ego_.s &&
        temp_vehicle.s > max_s) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

bool BehaviorFSM::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane,
                                Vehicle &rVehicle) {
  /*
    Returns a true if a vehicle is found ahead of the current vehicle, false
    otherwise. The passed reference rVehicle is updated if a vehicle is found.
    */
  int min_s = this->goal_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == lane && temp_vehicle.s > ego_.s &&
        temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

int BehaviorFSM::find_goal_lane(map<int, vector<Vehicle>> predictions) {
  /*
    Returns a true if a vehicle is found ahead of the current vehicle, false
    otherwise. The passed reference rVehicle is updated if a vehicle is found.
    */
  Vehicle temp_vehicle;


  return goal_lane;
}

void BehaviorFSM::refresh_ego(Vehicle &ego) {
  ego_.x = ego.x;
  ego_.y = ego.y;
  ego_.s = ego.s;
  ego_.d = ego.d;
  ego_.yaw = ego.yaw;
  ego_.v = ego.v;
}

void BehaviorFSM::realize_next_state(vector<Vehicle> trajectory) {
  /*
    Sets state and kinematics for ego vehicle using the last state of the
    trajectory.
    */
  Vehicle next_state = trajectory[1];
  ego_.state = next_state.state;
  ego_.lane = next_state.lane;
  ego_.s = next_state.s;
  ego_.v = next_state.v;
  ego_.a = next_state.a;
}

void BehaviorFSM::configure() {
  /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
  target_speed = ParameterConfig::target_speed;
  lanes_available = ParameterConfig::lanes_available;
  goal_s = ParameterConfig::goal_s;
  goal_lane = ParameterConfig::goal_lane;
  max_acceleration = ParameterConfig::max_acceleration;



}
