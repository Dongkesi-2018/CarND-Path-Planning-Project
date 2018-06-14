#include "path_planning.h"
#include "json.hpp"
#include "prediction.h"
#include "sensor_fusion.h"
#include "simulator.h"
#include "trajectory.h"
#include "vehicle.h"

using nlohmann::json;

void PathPlanning::Solver(json &sensor_data) {
  /*
    step 1: get sensor fusion data
  */
  sensor_.Update(sensor_data);
  sim_.Update(sensor_data);

  /*
    step2: update vehicles location
  */
  this->UpdateVehicles();
  /*
    step 3: prediction
    input: sensor fusion data;
    output: prediction trajectory
  */
  pred_.GeneratePredictions(non_ego_);

  /*
    step 3: behavior
    input: prediction, sensor fusion
    output: goal_lane, ref_vel
  */
  vector<double> behavior_output =
      behavior_.Solver(pred_.get_predictions(), ego_);
  int goal_lane = int(behavior_output[0]);
  double goal_speed = behavior_output[1];

  /*
    step4: trajectory
    input: prediction, behavior output
    output: trajectory x, y
  */
  traj_.UpdateLocationData(sim_, sensor_);
  traj_.UpdateBehaviorData(goal_lane, goal_speed);
  traj_.GenerateTrajectory(next_x_vals, next_y_vals);
}

void PathPlanning::update_non_ego(NonEgoVehicle &non_ego) {
  if (non_ego_.find(non_ego.id) == non_ego_.end()) {
    this->non_ego_.insert(std::pair<int, Vehicle>(
        non_ego.id, Vehicle(non_ego.x, non_ego.y, non_ego.vx, non_ego.vy,
                            non_ego.s, non_ego.d, 0)));
  } else {
  }
}

void PathPlanning::update_ego(EgoVehicle &ego) {
  ego_ = Vehicle(ego.car_x, ego.car_y, ego.car_s, ego.car_d, ego.car_yaw, ego.car_speed);
}

void PathPlanning::UpdateVehicles() {
  vector<NonEgoVehicle> &non_ego = sensor_.get_non_ego();
  // didn't track every car now. just remove all and get all cars again.
  // TODO: if this doesn't work, change to track strategy.
  this->non_ego_.clear();
  for (auto it = non_ego.begin(); it != non_ego.end(); ++it) {
    update_non_ego(*it);
  }
  update_ego(sensor_.get_ego());
}