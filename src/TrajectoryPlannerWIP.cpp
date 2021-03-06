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

#include "TrajectoryPlannerWIP.h"
#include "helper.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include "Dense"
#include "spdlog/spdlog.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @brief load map into class
 * 
 */
void TrajectoryPlannerWIP::loadMap() {
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

  // set points
  this->wp_spline_x.set_points(maps_s, maps_x);
  this->wp_spline_y.set_points(maps_s, maps_y);
  this->wp_spline_dx.set_points(maps_s, maps_dx);
  this->wp_spline_dy.set_points(maps_s, maps_dy);  
}


/**
 * @brief predict next end point
 * 
 * @param start_s 
 * @param start_d 
 * @param lane 
 * @param current_speed 
 * @param ref_speed 
 * @param Time 
 * @return vector<double> 
 */
vector<double> TrajectoryPlannerWIP::predictNextEndpoint(double start_s, double start_d, 
                                                      double lane, double current_speed,
                                                      double ref_speed, double Time) {
  double result_s;
  double result_d;

  if (lane == 0) {
    result_d = 2;
  } else if (lane == 1) {
    result_d = 6;
  } else if (lane == 2) {
    result_d = 10;
  } else {
    // TODO: proper error handling
    spdlog::get("console")->error("Error in TrajectoryPlannerWIP::predictNextEndpoint, got invalid lane {}", lane);
  }

  double acceleration = (ref_speed - current_speed)/Time;

  spdlog::get("console")->info("Current Speed: {} | Ref_Speed: {} --> Acceleration: {}",current_speed, ref_speed, acceleration);

  result_s = start_s + (Time * current_speed) + 0.5*acceleration*pow(Time,2);

  vector<double> result{result_s,result_d};
  return result;
}

/**
 * @brief get next path trajectory
 * 
 * @param start_s 
 * @param start_d 
 * @param target_lane 
 * @param current_speed 
 * @param ref_speed 
 * @param Time 
 * @return vector<vector<double>> 
 */
vector<vector<double>> TrajectoryPlannerWIP::getNextPathTrajectory(double start_s, double start_d, 
                                             double target_lane, double current_speed, 
                                             double ref_speed, double Time) {

  vector<double> nextEndpoint;
  double target_speed;

  // limit the maximum speed
  if (ref_speed > 20.5) {
    target_speed = 20.5;
  } else if (ref_speed < 0) {
    target_speed = 0;
  } else {
    target_speed = ref_speed;
  }                                            

  double acceleration = (target_speed - current_speed)/Time;
  
  // limit the maximum acceleration
  if (acceleration > 6) {
    target_speed = current_speed + 6;
  } else if (acceleration < -6) {
    target_speed = current_speed - 6;
  }

  // now do the JMT calculation
  vector<double> coeffs_s;
  vector<double> coeffs_d;
  int numberOfPoints = Time * 50 + 1;

  if (start_s_JMT[0] == -1) {
    start_s_JMT = {start_s,0,0};
    start_d_JMT = {start_d, 0 , 0};
    nextEndpoint = predictNextEndpoint(start_s , start_d , target_lane , current_speed , target_speed , Time);
  } else {
    start_s_JMT = {end_s_JMT[0],end_s_JMT[1],end_s_JMT[2]};
    start_d_JMT = {end_d_JMT[0], end_d_JMT[1] , end_d_JMT[2]};
    nextEndpoint = predictNextEndpoint(start_s_JMT[0] , start_d_JMT[0] , target_lane , current_speed , target_speed , Time);
  }
  
  end_s_JMT = {nextEndpoint[0] , target_speed , 0};
  end_d_JMT = {nextEndpoint[1] , 0 , 0};
  
  spdlog::get("console")->debug("start_s = {} | target_speed = {} | ACC = {}",start_s, target_speed, acceleration);
  spdlog::get("console")->debug("start_d = {} | end_d = {}", start_d, nextEndpoint[1]);

  spdlog::get("console")->debug("start_s = {} | start_s_dot = {} | start_s_dot_dot = {}",start_s_JMT[0], start_s_JMT[1], start_s_JMT[2]);
  spdlog::get("console")->debug("end_s = {} | end_s_dot = {} | end_s_dot_dot = {}",end_s_JMT[0], end_s_JMT[1], end_s_JMT[2]);
  
  spdlog::get("console")->debug("start_d = {} | start_d_dot = {} | start_d_dot_dot = {}",start_d_JMT[0], start_d_JMT[1], start_d_JMT[2]);
  spdlog::get("console")->debug("end_d = {} | end_d_dot = {} | end_d_dot_dot = {}",end_d_JMT[0], end_d_JMT[1], end_d_JMT[2]);

  coeffs_s = this->JMT(start_s_JMT,end_s_JMT,Time);
  coeffs_d = this->JMT(start_d_JMT,end_d_JMT,Time);

  double t = 0.0;
  double steps = 0.02;
  double result;
  
  
  vector<double> next_s_vals;
  vector<double> next_d_vals;
  
  for( int i = 1 ; i < numberOfPoints ; i++) {
    t += steps;
    double next_s_val = coeffs_s[0] + coeffs_s[1]*t + coeffs_s[2]*pow(t,2.0) 
                        + coeffs_s[3]*pow(t,3.0) + coeffs_s[4]*pow(t,4.0) + coeffs_s[5]*pow(t,5.0);
    next_s_vals.push_back(fmod(next_s_val,max_s)); // fmod is needed because of the max length of the course

    double next_d_val = coeffs_d[0] + coeffs_d[1]*t + coeffs_d[2]*pow(t,2.0) 
                        + coeffs_d[3]*pow(t,3.0) + coeffs_d[4]*pow(t,4.0) + coeffs_d[5]*pow(t,5.0);
    next_d_vals.push_back(next_d_val);
  }

  vector<vector<double>> next_vals{{},{}};
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  for(int i = 0 ; i < next_s_vals.size() ; i++) {
    vector<double> xy;
    vector<double> xy1;
    xy = getXY_JMT(next_s_vals[i],next_d_vals[i]);
    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);

    if (next_s_vals[i] > 6800 || next_s_vals[i] < 100) {
      spdlog::get("console")->info("x: {} | y = {}", xy[0], xy[1]);
      spdlog::get("console")->info("s: {} | d = {}", next_s_vals[i],next_d_vals[i]);
    }
  }

  next_vals[0] = next_x_vals;
  next_vals[1] = next_y_vals;
  return next_vals;
}

/**
 * @brief merge the trajectories 
 * 
 * @param previous_path_x 
 * @param previous_path_y 
 * @param end_path_s 
 * @param end_path_d 
 * @param newTraj 
 * @return vector<vector<double>> 
 */
vector<vector<double>> TrajectoryPlannerWIP::mergeTrajectories(vector<double> previous_path_x,
                                          vector<double> previous_path_y,
                                          double end_path_s , double end_path_d,    
                                          vector<vector<double>> newTraj) {
  vector<vector<double>> result; 
  vector<double> result_x;
  vector<double> result_y;
  
  
  if (previous_path_x.size() > 0) {
    for (int i = 0; i < previous_path_x.size() ; i++) {
        result_x.push_back(previous_path_x[i]);
        result_y.push_back(previous_path_y[i]);
    }

    for (int i = 0; i < newTraj[0].size(); i++) {
        result_x.push_back(newTraj[0][i]);
        result_y.push_back(newTraj[1][i]);
    }

  } else {
    for (int i = 0; i < newTraj[0].size() ; i++) {
      result_x.push_back(newTraj[0][i]);
      result_y.push_back(newTraj[1][i]);
      spdlog::get("console")->info("adding point x: {} | y: {}", newTraj[0][i] ,newTraj[0][i]);
      //cout << 
    }
  }
  

  result.push_back(result_x);
  result.push_back(result_y);

  if (end_path_s > 6800) {
    spdlog::get("console")->info("Merged Trajectory consists of {} points",result_x.size());

    for (int i = 0; i < result[0].size() ; i++) {
      spdlog::get("console")->info("Merged Trajectory: x = {} | y = {} ",result[0][i],result[1][i]);
    }
  }


  return result;
}



/**
 * @brief JMT from Udacity 
 * 
 * @param start 
 * @param end 
 * @param T 
 * @return vector<double> 
 */
vector<double> TrajectoryPlannerWIP::JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    
    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
		
	MatrixXd B = MatrixXd(3,1);	    
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
			    
	MatrixXd Ai = A.inverse();
	
	MatrixXd C = Ai*B;
	
	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}
	
    return result;
    
}


/**
 * @brief 
 * 
 * @param s 
 * @param d 
 * @return vector<double> 
 */
vector<double> TrajectoryPlannerWIP::getXY_JMT(double s, double d){
  double wp_x, wp_y, wp_dx, wp_dy, next_x, next_y;

  // spline interpolation
  wp_x = this->wp_spline_x(s);
  wp_y = this->wp_spline_y(s);
  wp_dx = this->wp_spline_dx(s);
  wp_dy = this->wp_spline_dy(s);

  next_x = wp_x + wp_dx * d;
  next_y = wp_y + wp_dy * d;

  return {next_x, next_y};
}
