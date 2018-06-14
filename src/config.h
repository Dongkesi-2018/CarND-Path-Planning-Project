#ifndef CONFIG_H_
#define CONFIG_H_
#include <array>
using std::array;

class ParameterConfig {
public:
static const double safe_distance; //safe distance for change lane  
static const double prediction_interval; //prediction interval for generator prediction trajectory
static const double preferred_buffer;
static const array<string, 2> vehicle_type;
static const double target_speed;
static const int lanes_available;
static const double goal_s;
static const int goal_lane;
static const double max_acceleration;
};
const double ParameterConfig::safe_distance = 5.0;
const double ParameterConfig::prediction_interval = 0.2;
const double preferred_buffer = 30;
const array<string, 2> vehicle_type = {"ego", "non-ego"};
const double target_speed = 49.9;
const int lanes_available = 3;
const double goal_s = 6945.554;
const double max_acceleration = 10;
const int goal_lane = 1;
#endif
