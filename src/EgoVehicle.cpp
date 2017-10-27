#include "helper.h"
#include <algorithm>
#include <math.h>
#include <vector>

#include "EgoVehicle.h"

using namespace std;
using nlohmann::json;

EgoVehicle::EgoVehicle() {
  //this->id = "EGO"; 
  this->id = -1; 
  this->speed_from_sim = -1;
}

double EgoVehicle::getSpeed() { 
  // if (previous_path_x.size() > 2 && this->speed_from_sim > 10) {
  //   double dx = previous_path_x[0]-previous_path_x[1];
  //   double dy = previous_path_y[0]-previous_path_y[1];
  //   return sqrt(pow(dx,2)+pow(dy,2));
  // } else {
  //   return this->speed_from_sim; 
  // }

  return this->speed_from_sim; 
  
}

double EgoVehicle::getSpeed_x() { 
  return this->speed_from_sim * cos(this->yaw); 
}

double EgoVehicle::getSpeed_y() { 
  return this->speed_from_sim * sin(this->yaw); 
}

void EgoVehicle::setRefSpeed(double refSpeed) { 
  this->refSpeed = refSpeed; 
}

double EgoVehicle::getRefSpeed() { 
  return this->refSpeed; 
}

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

  
void EgoVehicle::calculatePredictions(int predictionHorizon) {
  // first clear all current prediction
  predictions.clear();
  cout << "size of EgoVehicle predictions : "  << predictions.size() << endl;
}

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

  // for (int i = 0; i < previous_path_x.size(); i++) {
  //   result += "x = ";
  //   result += to_string(this->previous_path_x[i]);
  //   result += " ; y = ";
  //   result += to_string(this->previous_path_y[i]);
  //   result += " | ";
  // }

  return result;
}
