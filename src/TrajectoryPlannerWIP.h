/*
    Copyright (c) 2017 Antonia Reiter

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
    OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <vector>
#include "EgoVehicle.h"
#include "spline.h"
using namespace std;
using tk::spline; // spline


// For converting back and forth between radians and degrees.
extern double pi();
extern double deg2rad(double x);
extern double rad2deg(double x);
extern double distance(double x1, double y1, double x2, double y2);

/**
 * JMT based trajectory planner 
 */
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

  vector<double> getFrenet(double x, double y);
  vector<double> getXY(double s, double d);
  vector<double> JMT(vector< double> start, vector <double> end, double T);
  vector<double> predictNextEndpoint(double start_s, double start_d, double lane, double current_speed, double ref_speed, double Time);
  int NextWaypoint(double x, double y);  
  int ClosestWaypoint(double x, double y);
  vector<double> getXY_JMT(double s, double d);
  
  string map_file_ = "../data/highway_map.csv";

  /**Definitions of track */
  double max_s = 6945.554;
  double max_d = 12.0;

  /**Store map points in class not in main */
  vector<double> maps_x;
  vector<double> maps_y;
  vector<double> maps_s;
  vector<double> maps_dx;
  vector<double> maps_dy;

  /**JMT vectors for s and d */
  vector<double> start_s_JMT{-1,-1,-1};
  vector<double> end_s_JMT;
  vector<double> start_d_JMT;
  vector<double> end_d_JMT;

    /**define wp spline trajectory - use of spline causes gcc warning "whose type uses the anonymous namespace" */
    spline wp_spline_x;        
    spline wp_spline_y;
    spline wp_spline_dx;
    spline wp_spline_dy;  

};
