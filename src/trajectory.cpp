#include "trajectory.h"
#include <algorithm>
#include <vector>
#include "config.h"
#include "helper.h"
#include "lane_map.h"
#include "sensor_fusion.h"
#include "simulator.h"
#include "spline.h"

using std::vector;

void Trajectory::UpdateLocationData(Simulator &simulator,
                                    SensorFusion &sensor) {
  EgoVehicle &ego = sensor.get_ego();
  PrevPathData &prev_path = simulator.get_prev_path();

  this->ref_x = ego.car_x;
  this->ref_y = ego.car_y;
  // Sensor output yaw is deg
  this->ref_yaw = deg2rad(ego.car_yaw);
  this->car_x = ego.car_x;
  this->car_y = ego.car_y;
  this->car_s = ego.car_s;
//   this->car_speed = mph2mps(ego.car_speed);  // unit: m/s
  this->previous_path_x = prev_path.x;
  this->previous_path_y = prev_path.y;
}

void Trajectory::UpdatePredictionData() {}

// Input unit: m/s
double Trajectory::SmoothSpeed(double cur_v, double goal_v, double dt) {
  //   double ref_v = ParameterConfig::smooth_w * cur_v + (1 -
  //   ParameterConfig::smooth_w) * goal_v; //unit: m/s
  double ref_v = goal_v;
  double max_acc_v = dt * 10;  // v = v+at; current speed unit is m/s, the max
                               // accelaration is 10m/s2.
  double gap_v = ref_v - cur_v;
  if (gap_v > max_acc_v) {
    ref_v = cur_v + max_acc_v;
  }
  if (gap_v < -max_acc_v) {
    ref_v = cur_v - max_acc_v;
  }
  cout << "real ref_v: " << ref_v << endl;
  return ref_v;  // m/s
                 // return goal_v;
}

double Trajectory::SmoothSpeed(double cur_v, double goal_v) {
  double new_v = cur_v;
  if (this->is_accl) {
    if (cur_v < goal_v)
      new_v += ParameterConfig::max_acceleration * 0.01;
    // else
    //   new_v = goal_v;
  } else {
    if (cur_v > goal_v * 0.5)
      new_v -= ParameterConfig::max_acceleration * 0.01;
    // else
    //   new_v = goal_v;
  }
  if (new_v > ParameterConfig::target_speed) {
      new_v = ParameterConfig::target_speed;
  }
  if (new_v < 0) {
      new_v = 0.1;
  }
  cout << "real ref_v: " << new_v << endl;
  return new_v;
}
// goal_v unit: m/s
void Trajectory::UpdateBehaviorData(double goal_v, int goal_lane, double dt) {
  this->is_accl = goal_v > this->car_speed ? true : false;
  this->ref_vel = goal_v;//mps2mph(SmoothSpeed(car_speed, goal_v));
  this->lane = goal_lane;
}

void Trajectory::GenerateTrajectory(Simulator &simulator, SensorFusion &sensor,
                                    double goal_v, int goal_lane, double dt) {
  UpdateLocationData(simulator, sensor);
  UpdateBehaviorData(goal_v, goal_lane, dt);
  _GenerateTrajectory();
}

void Trajectory::CoordinateMap2Car(double &x_point, double &y_point) {
  double shift_x = x_point - ref_x;
  double shift_y = y_point - ref_y;

  x_point = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
  y_point = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
}

void Trajectory::CoordinateCar2Map(double &x_point, double &y_point) {
  double x_ref = x_point;
  double y_ref = y_point;

  x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
  y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

  x_point += ref_x;
  y_point += ref_y;
}

void Trajectory::Fit() {
  vector<double> ptsx;
  vector<double> ptsy;
  auto prev_size = previous_path_x.size();
//   prev_size = 0;
  if (prev_size < 2) {
    double prev_car_x = car_x - cos(ref_yaw);
    double prev_car_y = car_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  Map &lm = Map::getInstance();
  vector<double> next_wp0 = lm.getXY(car_s + 30, (2 + 4 * lane));
  vector<double> next_wp1 = lm.getXY(car_s + 60, (2 + 4 * lane));
  vector<double> next_wp2 = lm.getXY(car_s + 90, (2 + 4 * lane));

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++) {
    CoordinateMap2Car(ptsx[i], ptsy[i]);
  }

  s.set_points(ptsx, ptsy);
}

void Trajectory::_GenerateTrajectory() {
  Fit();
  next_x_vals.clear();
  next_y_vals.clear();
  for (int i = 0; i < previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0;

  for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
    this->car_speed = SmoothSpeed(this->car_speed, this->ref_vel);
    // double vel = mps2mph(this->car_speed);
    // double N = target_dist / (0.02 * ref_vel);
    // double x_point = x_add_on + target_x / N;
    double x_point = x_add_on + (0.02 * this->car_speed);
    if (x_point > target_x) {
        cout << "break!!!" << endl;
        break;
    }
    double y_point = s(x_point);

    x_add_on = x_point;

    CoordinateCar2Map(x_point, y_point);

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
//   for(size_t i = 0; i != next_x_vals.size(); i++) {
//       cout << "next_x: " << next_x_vals[i] << " next_y: " << next_y_vals[i] << endl;
//   }
}