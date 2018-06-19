#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <map>
#include <vector>
#include "lane_map.h"
#include "sensor_fusion.h"
#include "simulator.h"
#include "spline.h"
#include "vehicle.h"

using std::map;
using std::vector;

class Trajectory {
 public:
  Trajectory() : car_speed(0), startup(true), accl_w(0.01) {}
  void UpdateLocationData(Simulator& simulator, SensorFusion& sensor);
  void UpdatePredictionData(const map<int, vector<Vehicle> >& prediction);
  void UpdateBehaviorData(double v, int lane, double dt);

  void CoordinateMap2Car(double& x_point, double& y_point);
  void CoordinateCar2Map(double& x_point, double& y_point);

  vector<double>& get_next_x_vals() { return next_x_vals; }
  vector<double>& get_next_y_vals() { return next_y_vals; }
  void Fit();
  double SmoothSpeedSlow(double cur_v, double goal_v);
  double SmoothSpeedFast(double cur_v, double goal_v);
  double SmoothSpeed(double cur_v, double goal_v);
  void GenerateTrajectory(const map<int, vector<Vehicle> >& prediction,
                          Simulator& simulator, SensorFusion& sensor,
                          double goal_v, int goal_lane, double dt);
  bool EmergencyBraking();

 private:
  double ref_yaw;
  double ref_x;
  double ref_y;
  double ref_vel;
  bool is_accl;
  bool startup;
  double accl_w;

  double car_x;
  double car_y;
  double car_s;
  int car_lane;
  double end_path_s;
  double car_speed;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  map<int, vector<Vehicle> > predictions;
  int lane;
  tk::spline s;

  void _GenerateTrajectory();
};

#endif
