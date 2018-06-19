#ifndef PREDICTION_H_
#define PREDICTION_H_
#include <vector>
#include "vehicle.h"

using std::vector;

class Prediction {
 public:
  vector<Vehicle> GenerateOneVehiclePredictions(const Vehicle& vehicle,
                                                int horizon);
  map<int, vector<Vehicle> >& GeneratePredictions(
      const map<int, Vehicle>& non_ego);
  map<int, vector<Vehicle> >& get_predictions() { return predictions_; }

 private:
  map<int, vector<Vehicle> > predictions_;
};
#endif
