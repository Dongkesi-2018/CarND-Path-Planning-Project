#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include "spline.h"
#include "simulator.h"
#include "sensor_fusion.h"
#include "lane_map.h"

using std::vector;
using tk::spline;

class Trajectory {
 public:
  void UpdateLocationData(Simulator &simulator, SensorFusion &sensor);
  void UpdatePredictionData();
  void UpdateBehaviorData(double v, int lane);

  void CoordinateMap2Car(double &x_point, double &y_point);
  void CoordinateCar2Map(double &x_point, double &y_point);

  void Fit();
  void GenerateTrajectory(vector<double> &next_x_vals,
                          vector<double> &next_y_vals);

 private:
  double ref_yaw;
  double ref_x;
  double ref_y;
  double ref_vel;

  double car_x;
  double car_y;
  double car_s;
  vector<double> previous_path_x;
  vector<double> previous_path_y;


  int lane;

  tk::spline s;
};

#endif
