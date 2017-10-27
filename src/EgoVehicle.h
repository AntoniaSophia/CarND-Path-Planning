#pragma once

#include "json.hpp"
#include <math.h>
#include <vector>
#include "AbstractVehicle.h"
#include "json.hpp"
#include <string>

using namespace std;
using nlohmann::json;

class EgoVehicle: public AbstractVehicle {

 public:
  EgoVehicle();

  double getSpeed();
  double getSpeed_x();
  double getSpeed_y();
  string display();

  void setRefSpeed(double refSpeed = 10);
  double getRefSpeed();

  void update(json j);

  vector<double> getPrevious_path_x() {return previous_path_x;};
  vector<double> getPrevious_path_y() {return previous_path_y;};
  
  double getYawRad() {return yaw * M_PI / 180;};
  double getYawDeg() {return yaw;};
    double getEnd_Path_s() {return end_path_s;};
  double getEnd_Path_d() {return end_path_d;};

  vector<double> getPrediction(int second);
  void calculatePredictions(int predictionHorizon);
  
 protected:
    double refSpeed;
    double yaw;
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    double end_path_s;
    double end_path_d;
    double speed_from_sim;
};
