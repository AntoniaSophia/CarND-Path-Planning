#pragma once

#include <vector>
#include "EgoVehicle.h"
using namespace std;

// For converting back and forth between radians and degrees.
extern double pi();
extern double deg2rad(double x);
extern double rad2deg(double x);
extern double distance(double x1, double y1, double x2, double y2);

class TrajectoryPlanner {
public:
  void loadMap();
  vector<vector<double>> calcTrajFromQA(EgoVehicle egoVehicle, double target_speed, int target_lane);
    
protected:
  string map_file_ = "../data/highway_map.csv";
  double max_s = 6945.554;
  vector<double> maps_x;
  vector<double> maps_y;
  vector<double> maps_s;
  vector<double> maps_dx;
  vector<double> maps_dy;
  vector<double> getFrenet(double x, double y);
  vector<double> getXY(double s, double d);
  int NextWaypoint(double x, double y);  
  int ClosestWaypoint(double x, double y);  
};