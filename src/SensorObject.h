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

using std::vector;
using std::string;
using nlohmann::json;

/**
 * Sensorobjects
 */
class SensorObject : public AbstractVehicle {

public:
  SensorObject();

  double getSpeed();
  double getSpeed_x();
  double getSpeed_y();
  string display();

  void update(json j);

  vector<double> getPrediction(int second);
  void calculatePredictions(int predictionHorizon);

protected:
  double speed_x; /**< speed in x direction */
  double speed_y; /**< speed in y direction */

  double getProbabilityOfLaneChangeLeft();
  double getProbabilityOfLaneChangeRight();
  double getProbabilityOfLaneKeep();
};
