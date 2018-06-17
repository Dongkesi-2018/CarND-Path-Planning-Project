#ifndef CONFIG_H_
#define CONFIG_H_

#include <string>
using std::string;

class ParameterConfig {
public:
static const double safe_distance; //safe distance for change lane  
static const double prediction_interval; //prediction interval for generator prediction trajectory
static const double preferred_buffer;
static const double target_speed;
static const int lanes_available;
static const double goal_s;
static const int goal_lane;
static const double max_acceleration;
static const int horizon;
static const double smooth_w;
};
#endif
