#include "prediction.h"
#include <vector>
#include <map>
#include "config.h"
#include "vehicle.h"
#include "lane_map.h"

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
    vector<double> frenet = Vehicle::PositionAt(vehicle, i * ParameterConfig::prediction_interval);
    double next_s = frenet[0];
    double next_d = frenet[1];
    double next_v = 0;
    int new_lane = int(next_d / 4);

    if (i < horizon - 1) {
      next_v = vehicle.v;
    }
    predictions.push_back(Vehicle(new_lane, next_s, next_v, 0));
  }
  return predictions;
}

map<int, vector<Vehicle> >& Prediction::GeneratePredictions(map<int, Vehicle> &non_ego) {
  map<int, Vehicle>::iterator it = non_ego.begin();
  while (it != non_ego.end()) {
    int v_id = it->first;
    vector<Vehicle> preds = GenerateOneVehiclePredictions(it->second, 2);
    predictions_[v_id] = preds;
    it++;
  }
  return predictions_;
}


