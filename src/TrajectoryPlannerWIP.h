#pragma once

#include <vector>
#include "EgoVehicle.h"
#include "spline.h"
using namespace std;
using namespace tk; // spline


// For converting back and forth between radians and degrees.
extern double pi();
extern double deg2rad(double x);
extern double rad2deg(double x);
extern double distance(double x1, double y1, double x2, double y2);


class TrajectoryPlannerWIP {

public:
  void loadMap();
  vector<vector<double>> getNextPathTrajectory(double start_s, double start_d, double target_lane, double current_speed, double ref_speed, double Time);
  vector<vector<double>> mergeTrajectories(vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d, vector<vector<double>> newTraj);

  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;

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
  vector<double> JMT(vector< double> start, vector <double> end, double T);
  vector<double> predictNextEndpoint(double start_s, double start_d, double lane, double current_speed, double ref_speed, double Time);
  int NextWaypoint(double x, double y);  
  int ClosestWaypoint(double x, double y);
   
  vector<double> start_s_JMT{-1,-1,-1};
  vector<double> end_s_JMT;
  vector<double> start_d_JMT;
  vector<double> end_d_JMT;

    // define wp spline trajectory
    spline wp_spline_x;
    spline wp_spline_y;
    spline wp_spline_dx;
    spline wp_spline_dy;  
    vector<double> getXY_JMT(double s, double d);
      
  
};
