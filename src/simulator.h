#ifndef SIMULATOR_H_
#define SIMULATOR_H_
#include <vector>
#include "json.hpp"
using nlohmann::json;
using std::vector;

struct PrevPathData {
  vector<double> x;
  vector<double> y;
  double end_s;
  double end_d;
  PrevPathData() {}
  PrevPathData(vector<double> x, vector<double> y, double s, double d)
      : x(x), y(y), end_s(s), end_d(d) {}
};

class Simulator {
 public:
  void Update(json& sensor_data) {
    prev_path_ = PrevPathData(
        sensor_data["previous_path_x"], sensor_data["previous_path_y"],
        sensor_data["end_path_s"], sensor_data["end_path_d"]);
  }
  PrevPathData& get_prev_path() { return prev_path_; }

 private:
  PrevPathData prev_path_;
};

#endif
