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

#include <string>
#include <vector>
#include "json.hpp"
#include <math.h>

using nlohmann::json;

/**
 * Abstract class for Vehicles: ego and other
 **/
class AbstractVehicle {
 public:
  virtual double getSpeed() = 0; 
  virtual double getSpeed_x() = 0;
  virtual double getSpeed_y() = 0;

  virtual std::string display() = 0;
  virtual void update(json j) = 0;
  virtual std::vector<double> getPrediction(int second) = 0;

  int getLane();
  double getFrenet_s() { return frenet_s; };
  double getFrenet_d() { return frenet_d; };
  double getCartesian_x() { return cartesian_x; };
  double getCartesian_y() { return cartesian_y; };

  double getID() { return id; };

  virtual void calculatePredictions(int predictionHorizon) = 0;

  protected:
  double id;                          /**< unique id of vehicle */
  double frenet_s;                    /**< s-frenet value of vehicle */
  double frenet_d;                    /**< d-frenet value of vehicle */
  double cartesian_x;                 /**< x-value of vehicle */
  double cartesian_y;                 /**< y-value of vehicle */
  int lastLane;                       /**< number of last lane before change */
  int currentLane;                    /**< current lane before */
  std::vector<double> last_frenet_d;  /**< last values of d-freenet values */
  std::vector<double> last_frenet_s;  /**< last values of s-freenet values */
  int historyLength = 10;             /**< length of buffer for frenet values */

  int predictionHorizonInSeconds = 10; /**< how many seconds the planner should predict the best maneuvr */
  std::vector<std::vector<double>> predictions; /**< store the predictions in vector */

};