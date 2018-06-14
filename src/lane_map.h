#ifndef LANE_MAP_H_
#define LANE_MAP_H_
#include <vector>
using std::vector;

class Map{
public:
  static Map &getInstance()
  {
    static Map instance;
    return instance;
  }

  double distance(double x1, double y1, double x2, double y2);
  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
  vector<double> getFrenet(double x, double y, double theta);
  vector<double> getXY(double s, double d);
  void addWaypoint(double x, double y, double s, double dx, double dy);

private:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

private:
  Map() = default;
  Map(const Map &) = delete;
  Map & operator=(const Map &) = delete;
};

#endif
