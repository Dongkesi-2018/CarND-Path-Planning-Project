#include "behavior_fsm.h"
#include <algorithm>
#include <map>
#include <vector>
#include "config.h"
#include "cost.h"
#include "helper.h"
#include "prediction.h"
#include "trace.h"
#include "vehicle.h"

using std::map;
using std::min;
using std::vector;

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
  trace_enter();
  vector<string> states = successor_states();
  double cost;
  vector<double> costs;
  vector<string> final_states;
  vector<vector<Vehicle>> final_trajectories;

  cout << "new prediction" << endl;
  for (auto it = states.begin(); it != states.end(); it++) {
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    // for (auto it = trajectory.begin(); it != trajectory.end(); ++it) {
    //   it->print("traj:");
    // }
    if (trajectory.size() != 0) {
      cost = calculate_cost(*this, predictions, trajectory);
      cout << *it << " cost:" << cost << endl;
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }

  vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  for (auto it = final_trajectories[best_idx].begin();
       it != final_trajectories[best_idx].end(); ++it) {
    it->print("best one:");
  }
  trace_exit();
  // TODO: Change return value here:
  return final_trajectories[best_idx];
}

vector<string> BehaviorFSM::successor_states() {
  /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
  trace_enter();
  vector<string> states;
  states.push_back("KL");
  string state = ego_.state;
  if (state.compare("KL") == 0) {
    if (ego_.lane != 0)
      states.push_back("PLCL");
    if (ego_.lane != lanes_available - 1)
      states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (ego_.lane != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (ego_.lane != lanes_available - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  trace_exit();
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> BehaviorFSM::generate_trajectory(
    string state, map<int, vector<Vehicle>> predictions) {
  /*
    Given a possible next state, generate the appropriate trajectory to realize
    the next state.
    */
  trace_enter();
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    cout << "keep_lane_trajectory" << endl;
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    cout << "lane_change_trajectory:" << state << endl;
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    cout << "prep_lane_change_trajectory:" << state << endl;
    trajectory = prep_lane_change_trajectory(state, predictions);
  }
  trace_exit();
  return trajectory;
}

vector<double> BehaviorFSM::get_kinematics(
    map<int, vector<Vehicle>> predictions, int lane) {
  /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
  trace_enter();
  double max_velocity_accel_limit = this->max_acceleration * dt + ego_.v;
  double new_position;
  double new_velocity;
  double new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;
  string path = "get_kinematics: ";
  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    path += "1, ";
    vehicle_ahead.print("vehicle_ahead");
    if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
      vehicle_behind.print("vehicle_behind");
      path += "2, ";
      new_velocity = vehicle_ahead.v;  // must travel at the speed of traffic,
                                       // regardless of preferred buffer
      // should have a safe distance, if the ego was blocked by an ahead vehicle
      // suddenly
      if (vehicle_ahead.s - ego_.s > cal_safe_distance(ego_.v)) {
        new_velocity *= 0.9;
      }
    } else {
      path += "3, ";
      double max_velocity_in_front =
          ((vehicle_ahead.s - ego_.s - cal_safe_distance(ego_.v)) +
           vehicle_ahead.v * dt - 0.5 * (ego_.a) * dt * dt) /
          dt;
      if (max_velocity_in_front < 0) {
        cout << "max_velocity_in_front: collision:" << vehicle_ahead.v << endl;
        max_velocity_in_front = vehicle_ahead.v * 0.9;
      }
      cout << max_velocity_in_front << ", " << max_velocity_accel_limit << ", "
           << this->target_speed << endl;
      new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit),
                         this->target_speed);
    }
  } else {
    path += "4, ";
    new_velocity = min(max_velocity_accel_limit, this->target_speed);
  }
  path += "5";
  cout << path << endl;
  new_accel =
      (new_velocity - ego_.v) / dt;  // Equation: (v_1 - v_0)/t = acceleration
  if (new_accel > ParameterConfig::max_acceleration)
    new_accel = ParameterConfig::max_acceleration;
  if (new_accel < -ParameterConfig::max_acceleration)
    new_accel = -ParameterConfig::max_acceleration;
  new_position = ego_.s + new_velocity * dt + new_accel / 2.0 * dt * dt;
  cout << "Before s, v, a, dt:" << ego_.s << "," << ego_.v << "," << ego_.a
       << "," << dt << endl;
  cout << "After  s, v, a, dt:" << new_position << "," << new_velocity << ","
       << new_accel << endl;
  trace_exit();
  return {new_position, new_velocity, new_accel};
}

vector<Vehicle> BehaviorFSM::constant_speed_trajectory() {
  /*
    Generate a constant speed trajectory.
    */
  trace_enter();
  // Predict the next 1 second
  vector<double> frenet = Vehicle::PositionAt(ego_, dt);  // TODO: dt
  double next_pos = frenet[0];
  vector<Vehicle> trajectory = {
      Vehicle(ego_.lane, ego_.s, ego_.v, ego_.a, ego_.state),
      Vehicle(ego_.lane, next_pos, ego_.v, 0, ego_.state)};
  trace_exit();
  return trajectory;
}

vector<Vehicle> BehaviorFSM::keep_lane_trajectory(
    map<int, vector<Vehicle>> predictions) {
  /*
    Generate a keep lane trajectory.
    */
  trace_enter();
  vector<Vehicle> trajectory = {
      Vehicle(ego_.lane, ego_.s, ego_.v, ego_.a, ego_.state)};
  vector<double> kinematics = get_kinematics(predictions, ego_.lane);
  double new_s = kinematics[0];
  double new_v = kinematics[1];
  double new_a = kinematics[2];
  trajectory.push_back(Vehicle(ego_.lane, new_s, new_v, new_a, "KL"));
  trace_exit();
  return trajectory;
}

vector<Vehicle> BehaviorFSM::prep_lane_change_trajectory(
    string state, map<int, vector<Vehicle>> predictions) {
  /*
    Generate a trajectory preparing for a lane change.
    */
  trace_enter();
  double new_s;
  double new_v;
  double new_a;
  Vehicle vehicle_behind;
  int new_lane = ego_.lane + lane_direction[state];
  vector<Vehicle> trajectory = {
      Vehicle(ego_.lane, ego_.s, ego_.v, ego_.a, ego_.state)};
  vector<double> curr_lane_new_kinematics =
      get_kinematics(predictions, ego_.lane);
  int lane = ego_.lane;
  string path = state + ":";
  if (get_vehicle_behind(predictions, ego_.lane, vehicle_behind) /*&&
      vehicle_behind.v > ego_.v*/) {
    path += "1,";
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];
  } else {
    path += "2, ";
    vector<double> best_kinematics;
    vector<double> next_lane_new_kinematics =
        get_kinematics(predictions, new_lane);
    // Choose kinematics with lowest velocity.
    // TODO: this should be ">"
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      path += "3, ";
      lane = new_lane;
      best_kinematics = next_lane_new_kinematics;
    } else {
      path += "4, ";
      best_kinematics = curr_lane_new_kinematics;
    }

    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }
  path += "5";
  cout << path << endl;
  trajectory.push_back(Vehicle(ego_.lane, new_s, new_v, new_a, state));
  trace_exit();
  return trajectory;
}

vector<Vehicle> BehaviorFSM::lane_change_trajectory(
    string state, map<int, vector<Vehicle>> predictions) {
  /*
    Generate a lane change trajectory.
    */
  trace_enter();
  int new_lane = ego_.lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies that
  // spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    // Security distance for change lane
    if (abs(next_lane_vehicle.s - ego_.s) < 15/*cal_safe_distance(next_lane_vehicle.v)*/ &&
        next_lane_vehicle.lane == new_lane) {
      // If lane change is not possible, return empty trajectory.
      cout << "lane_change_trajectory: empty" << endl;
      trace_exit();
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(ego_.lane, ego_.s, ego_.v, ego_.a, ego_.state));
  vector<double> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(
      Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
  trace_exit();
  return trajectory;
}

bool BehaviorFSM::get_vehicle_behind(const map<int, vector<Vehicle>>& predictions,
                                     int lane, Vehicle &rVehicle, bool in_safe) const{
  /*
    Returns a true if a vehicle is found behind the current vehicle, false
    otherwise. The passed reference rVehicle is updated if a vehicle is found.
    */
  trace_enter();
  double max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (auto it = predictions.cbegin();
       it != predictions.cend(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == lane && temp_vehicle.s < ego_.s &&
        temp_vehicle.s > max_s) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  if (in_safe && found_vehicle && ((ego_.s - max_s) > cal_safe_distance(ego_.v))) {
    cout << "B: found_vehicle = false" << endl;
    found_vehicle = false;
  }
  trace_exit();
  return found_vehicle;
}

bool BehaviorFSM::get_vehicle_ahead(const map<int, vector<Vehicle>> &predictions,
                                    int lane, Vehicle &rVehicle, bool in_safe) const {
  /*
    Returns a true if a vehicle is found ahead of the current vehicle, false
    otherwise. The passed reference rVehicle is updated if a vehicle is found.
    */
  trace_enter();
  double min_s = this->goal_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (auto it = predictions.cbegin();
       it != predictions.cend(); ++it) {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane == lane && temp_vehicle.s > ego_.s &&
        temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  if (in_safe && found_vehicle && ((min_s - ego_.s) > cal_safe_distance(ego_.v))) {
    cout << "A: found_vehicle = false: " << min_s - ego_.s << endl;
    found_vehicle = false;
  }
  trace_exit();
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

void BehaviorFSM::refresh_ego(const Vehicle &ego, Simulator &sim, double dt) {
  trace_enter();
  PrevPathData &prev = sim.get_prev_path();
  this->prev_size = prev.x.size();
  this->end_s = prev.end_s;
  // this->dt = dt;
  // ego.print("new ego:");
  // ego_.print("refresh_before:");
  ego_.update_ego(ego, dt);
  // ego_.print("refresh_after:");

  trace_exit();
}

void BehaviorFSM::realize_next_state(vector<Vehicle> trajectory) {
  /*
    Sets state and kinematics for ego vehicle using the last state of the
    trajectory.
    */
  trace_enter();
  Vehicle next_state = trajectory[1];
  ego_.state = next_state.state;
  ego_.lane = next_state.lane;
  ego_.d = next_state.d;
  // ego_.s = next_state.s;
  // ego_.v = next_state.v;
  // ego_.a = next_state.a;
  trace_exit();
}

void BehaviorFSM::configure() {
  /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
  trace_enter();
  target_speed = ParameterConfig::target_speed;
  lanes_available = ParameterConfig::lanes_available;
  goal_s = ParameterConfig::goal_s;
  goal_lane = ParameterConfig::goal_lane;
  max_acceleration = ParameterConfig::max_acceleration;
  dt = ParameterConfig::prediction_interval * ParameterConfig::horizon;
  trace_exit();
}

double BehaviorFSM::cal_safe_distance(double v) const{
  double safe_distance =
      v / (0.278 * 2);  // 1kmph save distance is 1, 80kmph->80m. 22.2/0.278 = 80m

  return safe_distance < 20 ? 20: safe_distance;
}
