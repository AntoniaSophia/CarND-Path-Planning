/*
    Copyright (c) 2017 Udacity

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

#include "TrajectoryPlanner.h"
#include "helper.h"
#include "spline.h"
#include <fstream>
#include <cmath>
#include "Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void TrajectoryPlanner::loadMap() {
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    maps_x.push_back(x);
    maps_y.push_back(y);
    maps_s.push_back(s);
    maps_dx.push_back(d_x);
    maps_dy.push_back(d_y);
  }
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> TrajectoryPlanner::getXY(double s, double d) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading =
      atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> TrajectoryPlanner::getFrenet(double x, double y) {
  
  int next_wp = NextWaypoint(x, y);
  
  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
  prev_wp = maps_x.size() - 1;
  }
  
  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];
  
  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;
  
  double frenet_d = distance(x_x, x_y, proj_x, proj_y);
  
  // see if d value is positive or negative by comparing it to a center point
  
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);
  
  if (centerToPos <= centerToRef) {
  frenet_d *= -1;
  }
  
  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
  frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }
  
  frenet_s += distance(0, 0, proj_x, proj_y);
  
  return {frenet_s, frenet_d};
  }
  
  
  int TrajectoryPlanner::NextWaypoint(double x, double y) {
  
    int closestWaypoint = ClosestWaypoint(x, y);
  
    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];
  
    // heading vector
    double hx = map_x - x;
    double hy = map_y - y;
  
    // Normal vector:
    double nx = maps_dx[closestWaypoint];
    double ny = maps_dy[closestWaypoint];
  
    // Vector into the direction of the road (perpendicular to the normal vector)
    double vx = -ny;
    double vy = nx;
  
    // If the inner product of v and h is positive then we are behind the waypoint
    // so we do not need to
    // increment closestWaypoint, otherwise we are beyond the waypoint and we need
    // to increment closestWaypoint.
  
    double inner = hx * vx + hy * vy;
    if (inner < 0.0) {
      closestWaypoint++;
    }
  
    return closestWaypoint;
  }
  
  int TrajectoryPlanner::ClosestWaypoint(double x, double y) {
    double closestLen = 100000; // large number
    int closestWaypoint = 0;
  
    for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  
  return closestWaypoint;
  }
  


  vector<vector<double>> TrajectoryPlanner::calcTrajFromQA(EgoVehicle egoVehicle, double target_speed, int target_lane){

    vector<double> previous_path_x = egoVehicle.getPrevious_path_x();
    vector<double> previous_path_y = egoVehicle.getPrevious_path_y();
          int prev_size = previous_path_x.size();

          int next_wp = -1;
          double car_x = egoVehicle.getCartesian_x();
          double car_y = egoVehicle.getCartesian_y();
          double car_yaw = egoVehicle.getYawDeg();
          double ref_yaw = egoVehicle.getYawRad();
          double car_speed = egoVehicle.getSpeed();
          double end_path_s = egoVehicle.getEnd_Path_s();
          double car_s = egoVehicle.getFrenet_s();
          double ref_x = car_x;
          double ref_y = car_y;

          // If there is no data in the previous path, initialize starting point
          // with the current car space
          if (prev_size < 2) {
            next_wp = NextWaypoint(ref_x, ref_y);
            // Otherwise, populate with previous data path
            // Way to avoid crerating a new path from scratch. Just add
            // waypoints to it.
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            next_wp = NextWaypoint(ref_x, ref_y);

            // this is required in order to avoid jerks!!
            // TODO: refactor this and make it more transparent!!                       
            car_s = end_path_s;
            car_speed = (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) +
                              (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) /
                         .02) *
                        2.237;
          }

          // TODO!!
          // car_speed = target_speed;--> this problem has to be adressed 

          // TRACETORY!

          vector<double> ptsx;
          vector<double> ptsy;

          // If the prev_path is empty, use the current car space
          if (prev_size < 2) {
            // we need at least two waypoints for a spline calculation !!
            // so at the beginning chose two points
            // - current x,y position of car
            // - infinitesimal point from "past" 
            double prev_car_x = car_x - cos(car_yaw)*0.5;  // assume a small
            double prev_car_y = car_y - sin(car_yaw)*0.5;

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          } else {
            // If there is a previous path, use it as a start reference
            // for the next spline calculation
            ptsx.push_back(previous_path_x[prev_size - 2]);
            ptsx.push_back(previous_path_x[prev_size - 1]);

            ptsy.push_back(previous_path_y[prev_size - 2]);
            ptsy.push_back(previous_path_y[prev_size - 1]);
          }

          vector<double> next_wp0 =
          getXY(car_s + 30, (2 + 4 * target_lane));

          vector<double> next_wp1 =
          getXY(car_s + 60, (2 + 4 * target_lane));

          vector<double> next_wp2 =
          getXY(car_s + 90, (2 + 4 * target_lane));

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Transformation (shift in rotation): Make sure the last point of the
          // previous path is at the origin (0,0), and its angle is at 0 degres.
          // Basically, take the car's refeerence frame at the starting
          // reference points.
          // Make the math much easier after.
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          tk::spline s;

          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist =
              sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;

          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

            // small adaptation of speed to desired velocity
            // this is required in order to avoid acceleration overshoot!!
            if (target_speed > car_speed) {
              car_speed += .224;
            } else if (target_speed < car_speed) {
              car_speed -= .224;
            }

            double N = (target_dist / (.02 * car_speed / 2.24));
            
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          vector<vector<double>> next_vals{{},{}};
          next_vals[0]=next_x_vals;
          next_vals[1]=next_y_vals;
          return next_vals;
  }

  