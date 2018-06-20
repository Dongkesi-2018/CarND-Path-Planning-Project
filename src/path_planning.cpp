#include "path_planning.h"
#include <iostream>
#include <string>
#include "config.h"
#include "json.hpp"
#include "prediction.h"
#include "sensor_fusion.h"
#include "simulator.h"
#include "trajectory.h"
#include "vehicle.h"
using std::cout;
using std::endl;
using std::string;

using nlohmann::json;

void PathPlanning::Solver(json& sensor_data) {
  /*
    step 1: get sensor fusion data
  */
  dt = get_dt();
  cout << "dt:: " << dt << endl;
  sensor_.Update(sensor_data);
  sim_.Update(sensor_data);

  /*
    step2: update vehicles location
  */
  this->UpdateVehicles();
  // print_vehicle(string("Ego:"), ego_);
  // print_vehicle();

  /*
    step 3: prediction
    input: sensor fusion data;
    output: prediction trajectory
  */
  map<int, vector<Vehicle> >& vehicle_pred =
      pred_.GeneratePredictions(non_ego_);
  // print_vehicle(vehicle_pred);

  /*
    step 3: behavior
    input: prediction, sensor fusion
    output: goal_lane, ref_vel
  */

  vector<double> behavior_output = behavior_.Solver(vehicle_pred);
  int goal_lane = (int)(behavior_output[0]);
  double goal_speed = behavior_output[1];
  cout << "goal_lane: " << goal_lane;
  cout << "  goal_speed: " << goal_speed << endl;
  /*
    step4: trajectory
    input: prediction, behavior output
    output: trajectory x, y
  */
  traj_.GenerateTrajectory(vehicle_pred, sim_, sensor_, goal_speed, goal_lane);
}

void PathPlanning::update_non_ego(NonEgoVehicle& non_ego) {
  if (non_ego.d > 0 && non_ego.d < ParameterConfig::lanes_available * 4) {
    if (non_ego_.find(non_ego.id) == non_ego_.end()) {
      this->non_ego_.insert(std::pair<int, Vehicle>(
          non_ego.id, Vehicle(non_ego.x, non_ego.y, non_ego.vx, non_ego.vy,
                              non_ego.s, non_ego.d, 0)));
    } else {
      non_ego_[non_ego.id].update_non_ego(non_ego.x, non_ego.y, non_ego.vx,
                                          non_ego.vy, non_ego.s, non_ego.d, dt);
    }
  } else {
    auto it = non_ego_.find(non_ego.id);
    if (it != non_ego_.end()) {
      non_ego_.erase(non_ego.id);
    }
  }
}

void PathPlanning::update_ego(EgoVehicle& ego) {
  auto new_ego = Vehicle(ego.car_x, ego.car_y, ego.car_s, ego.car_d,
                         ego.car_yaw, ego.car_speed);
  new_ego.print("new_ego::");
  behavior_.update_ego(new_ego, dt);
}

void PathPlanning::UpdateVehicles() {
  vector<NonEgoVehicle>& non_ego = sensor_.get_non_ego();
  for (auto it = non_ego.begin(); it != non_ego.end(); ++it) {
    update_non_ego(*it);
  }
  update_ego(sensor_.get_ego());
}

void PathPlanning::print_vehicle(string head, Vehicle& car) {
  cout << "----------- " << head << " -----------" << endl;
  car.print("ego:");
}

void PathPlanning::print_vehicle() {
  int i = 0;
  for (auto it = non_ego_.begin(); it != non_ego_.end(); ++it) {
    cout << "=========== " << i++ << " ============" << endl;
    it->second.print("non_ego:");
  }
}

void PathPlanning::print_vehicle(map<int, vector<Vehicle> >& vehicle_pred) {
  int i = 0;
  for (auto it = vehicle_pred.begin(); it != vehicle_pred.end(); ++it) {
    cout << "=========== " << i++ << " ============" << endl;
    cout << "non_ego.id: " << it->first << endl;

    int j = 0;
    for (auto v_it = it->second.begin(); v_it != it->second.end(); ++v_it) {
      v_it->print("pred:");
    }
  }
}