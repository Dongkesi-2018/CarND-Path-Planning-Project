#include "prediction.h"
#include <vector>
#include <map>
#include "config.h"
#include "vehicle.h"
#include "lane_map.h"
#include "helper.h"

using std::vector;
using std::map;


vector<Vehicle> Prediction::GenerateOneVehiclePredictions(Vehicle &vehicle, int horizon) {
  /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
  vector<Vehicle> predictions;
  for (int i = 0; i < horizon; i++) {
    // TODO: prediction next t seconds location
    // print_bar('%', i);
    vector<double> next_state = Vehicle::PositionAt(vehicle, (i+0) * ParameterConfig::prediction_interval);
    double next_x = next_state[0];
    double next_y = next_state[1];
    double next_vx = next_state[2];
    double next_vy = next_state[3];
    double next_s = next_state[4];
    double next_d = next_state[5];

    predictions.push_back(Vehicle(next_x, next_y, next_vx, next_vy, next_s, next_d, 0));
  }
  return predictions;
}

map<int, vector<Vehicle> >& Prediction::GeneratePredictions(map<int, Vehicle> &non_ego, int prev_size) {
  map<int, Vehicle>::iterator it = non_ego.begin();
  predictions_.clear();
  while (it != non_ego.end()) {
    int v_id = it->first;
    vector<Vehicle> preds;
    double next_vx = it->second.vx;
    double next_vy = it->second.vy;
    double next_s = it->second.s + prev_size * 0.02 * it->second.v;
    double next_d = it->second.d;
    double next_x = it->second.x;
    double next_y = it->second.y;
    preds.push_back(Vehicle(next_x, next_y, next_vx, next_vy, next_s, next_d, 0));
    predictions_[v_id] = preds;
    it++;
  }
  return predictions_;
}

map<int, vector<Vehicle> >& Prediction::GeneratePredictions(map<int, Vehicle> &non_ego) {
  map<int, Vehicle>::iterator it = non_ego.begin();
  predictions_.clear();
  while (it != non_ego.end()) {
    int v_id = it->first;
    vector<Vehicle> preds = GenerateOneVehiclePredictions(it->second, ParameterConfig::horizon);
    predictions_[v_id] = preds;
    it++;
  }
  return predictions_;
}

