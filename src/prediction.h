#ifndef PREDICTION_H_
#define PREDICTION_H_
#include <vector>
using std::vector;
class Vehicle;
class Map;
class Prediction {
public:
  vector<Vehicle> GenerateOneVehiclePredictions(Vehicle &vehicle, int horizon);
  map<int, vector<Vehicle> >& GeneratePredictions(map<int, Vehicle> &non_ego);
  map<int, vector<Vehicle> >& get_predictions() { return predictions_; }
private:
  map<int, vector<Vehicle> > predictions_;
};
#endif
