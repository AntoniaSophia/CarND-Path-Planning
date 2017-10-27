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

#include "AbstractVehicle.h"

using namespace std;
using nlohmann::json;

class EgoVehicle : public AbstractVehicle {

public:
  EgoVehicle();

  double getSpeed();
  double getSpeed_x();
  double getSpeed_y();
  string display();

  void setRefSpeed(double refSpeed = 10);
  double getRefSpeed();

  void update(json j);

  vector<double> getPrevious_path_x() { return previous_path_x; };
  vector<double> getPrevious_path_y() { return previous_path_y; };

  double getYawRad() { return yaw * M_PI / 180; };
  double getYawDeg() { return yaw; };
  double getEnd_Path_s() { return end_path_s; };
  double getEnd_Path_d() { return end_path_d; };

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
