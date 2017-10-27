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

#include "helper.h"

#include "EgoVehicle.h"

#include "spdlog/spdlog.h"

using std::vector;
using nlohmann::json;

/**
 * @brief 
 * 
 */
EgoVehicle::EgoVehicle() {
  this->id = -1; 
  this->speed_from_sim = -1;
}

/**
 * @brief 
 * 
 * @return double 
 */
double EgoVehicle::getSpeed() { 
  return this->speed_from_sim; 
}

/**
 * @brief 
 * 
 * @return double 
 */
double EgoVehicle::getSpeed_x() { 
  return this->speed_from_sim * cos(this->yaw); 
}

/**
 * @brief 
 * 
 * @return double 
 */
double EgoVehicle::getSpeed_y() { 
  return this->speed_from_sim * sin(this->yaw); 
}

/**
 * @brief 
 * 
 * @param refSpeed 
 */
void EgoVehicle::setRefSpeed(double refSpeed) { 
  this->refSpeed = refSpeed; 
}

/**
 * @brief 
 * 
 * @return double 
 */
double EgoVehicle::getRefSpeed() { 
  return this->refSpeed; 
}

/**
 * @brief 
 * 
 * @param j 
 */
void EgoVehicle::update(json j) {
  this->frenet_s = j["s"];
  this->frenet_d = j["d"];
  this->yaw = j["yaw"];
  this->cartesian_x = j["x"];
  this->cartesian_y = j["y"];
  this->speed_from_sim = j["speed"];
  this->speed_from_sim = this->speed_from_sim * 0.447;  // convert from miles per hour to meters per second
  this->end_path_s = j["end_path_s"];
  this->end_path_d = j["end_path_d"];

  this->previous_path_x.clear();
  this->previous_path_y.clear();

  for (int i = 0; i < j["previous_path_x"].size(); i++)
    this->previous_path_x.push_back(j["previous_path_x"][i]);

  for (int i = 0; i < j["previous_path_y"].size(); i++)
    this->previous_path_y.push_back(j["previous_path_y"][i]);
}

/**
 * @brief 
 * 
 * @param predictionHorizon 
 */
void EgoVehicle::calculatePredictions(int predictionHorizon) {
  // first clear all current prediction
  predictions.clear();
  cout << "size of EgoVehicle predictions : "  << predictions.size() << endl;
}

/**
 * @brief 
 * 
 * @param second 
 * @return vector<double> 
 */
vector<double> EgoVehicle::getPrediction(int second) {
  if (second == 0) {
    vector<double> result;
    result.push_back(this->frenet_d);
    result.push_back(this->frenet_s);
    result.push_back(this->getLane());
    result.push_back(this->getSpeed());
    result.push_back(0);
    result.push_back(-1);
    return result;
  }

  vector<double> result;
  return result;
}

/**
 * @brief 
 * 
 * @return string 
 */
string EgoVehicle::display() {
  string result = "";

  result += "\nEgoVehicle";
  result += " | speed_from_sim = ";
  result += to_string(this->speed_from_sim);
  result += " | s = ";
  result += to_string(frenet_s);
  result += " | d = ";
  result += to_string(frenet_d);
  result += " | lane = ";
  result += to_string(this->getLane());
  result += " | size(previous_path) = ";
  result += to_string(previous_path_x.size());

  return result;
}
