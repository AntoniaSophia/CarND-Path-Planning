#include "TrajectoryPlannerWIP.h"
#include "EgoVehicle.h"
#include "helper.h"
#include "spline.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>
#include <cmath>
#include "Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> TrajectoryPlannerWIP::getXY(double s, double d) {
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
    cout << "Error in TrajectoryPlannerWIP::predictNextEndpoint" << endl;
    exit(0);
  }

  double acceleration = (ref_speed - current_speed)/Time;

  cout << "Current Speed: " << current_speed << " | Ref_Speed: " << ref_speed << "  --> Acceleration: " << acceleration << endl;

  result_s = start_s + (Time * current_speed) + 0.5*acceleration*pow(Time,2);

  vector<double> result{result_s,result_d};
  return result;
}

vector<vector<double>> TrajectoryPlannerWIP::getNextPathTrajectory(double start_s, double start_d, 
                                             double target_lane, double current_speed, 
                                             double ref_speed, double Time) {

  vector<double> nextEndpoint;
  double target_speed;

  if (ref_speed > 20.5) {
    target_speed = 20.5;
  } else if (ref_speed < 0) {
    target_speed = 0;
  } else {
    target_speed = ref_speed;
  }                                            

  double acceleration = (target_speed - current_speed)/Time;
  
  //cout << "Acceleration : "  << acceleration << endl;

  if (acceleration > 6) {
    target_speed = current_speed + 6;
  } else if (acceleration < -6) {
    target_speed = current_speed - 6;
  }


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
  
  //cout << "start_s = " << start_s << "  |  target_speed = " << target_speed << "  | ACC = " << acceleration << endl;
  //cout << "start_d = " << start_d << "  |  end_d = " << nextEndpoint[1] << endl;

  //cout << "start_s = " << start_s_JMT[0] << "  |  start_s_dot = " <<  start_s_JMT[1] << "  | start_s_dot_dot = " <<  start_s_JMT[2] << endl;
  //cout << "end_s = " << end_s_JMT[0] << "  |  end_s_dot = " <<  end_s_JMT[1] << "  | end_s_dot_dot = " <<  end_s_JMT[2] << endl;
  
  //cout << "start_d = " << start_d_JMT[0] << "  |  start_d_dot = " <<  start_d_JMT[1] << "  | start_d_dot_dot = " <<  start_d_JMT[2] << endl;
  //cout << "end_d = " << end_d_JMT[0] << "  |  end_d_dot = " <<  end_d_JMT[1] << "  | end_d_dot_dot = " <<  end_d_JMT[2] << endl;

  coeffs_s = this->JMT(start_s_JMT,end_s_JMT,Time);
  coeffs_d = this->JMT(start_d_JMT,end_d_JMT,Time);

  double t = 0.0;
  double steps = 0.02;
  double result;
  
  
  vector<double> next_s_vals;
  vector<double> next_d_vals;
  
  for(int i = 1 ; i < numberOfPoints ; i++) {
    t += steps;
    double next_s_val = coeffs_s[0] + coeffs_s[1]*t + coeffs_s[2]*pow(t,2.0) 
                        + coeffs_s[3]*pow(t,3.0) + coeffs_s[4]*pow(t,4.0) + coeffs_s[5]*pow(t,5.0);
    next_s_vals.push_back(fmod(next_s_val,max_s));

    double next_d_val = coeffs_d[0] + coeffs_d[1]*t + coeffs_d[2]*pow(t,2.0) 
                        + coeffs_d[3]*pow(t,3.0) + coeffs_d[4]*pow(t,4.0) + coeffs_d[5]*pow(t,5.0);
    next_d_vals.push_back(fmod(next_d_val,max_d));
  }

  vector<vector<double>> next_vals{{},{}};
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  for(int i = 0 ; i < next_s_vals.size() ; i++) {
    vector<double> xy;
    xy = getXY_JMT(next_s_vals[i],next_d_vals[i]);
    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);

    //cout << "x: " << xy[0] << " | y:" << xy[1] << endl;
    //cout << "s: " << next_s_vals[i] << " | d:" << next_d_vals[i] << endl;
  }

  next_vals[0] = next_x_vals;
  next_vals[1] = next_y_vals;
  return next_vals;
}


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
      //cout << "adding point x: " << newTraj[0][i] << " | y:" << newTraj[0][i] << endl;
    }
  }
  

  result.push_back(result_x);
  result.push_back(result_y);

  //cout << "Merged Trajectory consists of  " << result_x.size()  << " points " << endl;

  for (int i = 0; i < result[0].size() ; i++) {
    //cout << "Merged Trajectory: x = " << result[0][i]  << " | y = " << result[1][i] << endl;
  }


  return result;
}




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


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> TrajectoryPlannerWIP::getFrenet(double x, double y) {

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


int TrajectoryPlannerWIP::NextWaypoint(double x, double y) {

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

int TrajectoryPlannerWIP::ClosestWaypoint(double x, double y) {
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
