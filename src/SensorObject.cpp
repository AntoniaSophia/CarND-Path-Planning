#include "helper.h"
#include <algorithm>
#include <math.h>
#include <vector>

#include "SensorObject.h"

using namespace std;
using nlohmann::json;

SensorObject::SensorObject() {
  this->currentLane = -1;
  this->lastLane = -1;

}

// returns speed in unit meters/second
double SensorObject::getSpeed() { 
  return sqrt(pow(this->speed_x,2.0) + pow(this->speed_y,2.0)); 
}

double SensorObject::getSpeed_x() { 
  return this->speed_x;
}

double SensorObject::getSpeed_y() { 
  return this->speed_y;
}

//[car's unique ID, car's x position in map coordinates, car's y position in map coordinates, 
// car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, 
// car's d position in frenet coordinates. 
void SensorObject::update(json j) {
  this->id = j[0];
  this->cartesian_x = j[1];
  this->cartesian_y = j[2];
  this->speed_x = j[3];
  this->speed_y = j[4];
  this->frenet_s = j[5];
  this->frenet_d = j[6];

  this->last_frenet_d.push_back(this->frenet_d);

  if (this->last_frenet_d.size() > historyLength) {
    last_frenet_d.erase(last_frenet_d.begin());
  }

  this->last_frenet_s.push_back(this->frenet_s);
  
  if (this->last_frenet_s.size() > historyLength) {
    last_frenet_s.erase(last_frenet_s.begin());
  }

  if (this->currentLane == -1 && this->lastLane == -1) {
    this->lastLane = this->getLane();
    this->currentLane = this->getLane();
  } else if (this->lastLane != -1) {
    if (this->getLane() != this->lastLane) {
      cout << "Lane change of Object " << this->id << " from lane " << this->lastLane << " to lane " << this->getLane() << endl;
      this->lastLane = this->getLane();
    }
  }

  Timer tmr;
  double t = tmr.elapsed();
  tmr.reset();
  this->calculatePredictions(this->predictionHorizonInSeconds);
  t = tmr.elapsed();
  //std::cout << "Time for calculating predictions in (ms) " <<  t/1e-6 << endl;
}

void SensorObject::calculatePredictions(int predictionHorizon) {
  // first clear all current prediction
  predictions.clear();

  // what is required for a prediction?
  // frenet_d
  // frenet_s
  // lane
  // speed
  // second
  // evaluation
  int currentLane = this->getLane();
  int predictedLane = currentLane;
  double predictedSpeed = this->getSpeed();


  double predictedFrenet_d = this->frenet_d;
  double predictedFrenet_s = this->frenet_s;
  
  for (int i = 1; i < predictionHorizon ; i++) {
    if (i == 1) {
      if (getProbabilityOfLaneChangeRight() > 0.5 && currentLane < 2) {
        predictedLane = predictedLane + 1;
        predictedFrenet_d = predictedFrenet_d + 4;
      } else if (getProbabilityOfLaneChangeLeft() > 0.5 && currentLane > 0) {
        predictedLane = predictedLane - 1;
        predictedFrenet_d = predictedFrenet_d - 4;
      } else {
        // do nothing, because predicted lane is current lane anyway...
      }
    }

    predictedFrenet_s = predictedFrenet_s + (i * predictedSpeed);
    vector<double> nextPrediction;
    nextPrediction.push_back(predictedFrenet_d);
    nextPrediction.push_back(predictedFrenet_s);
    nextPrediction.push_back(predictedLane);
    nextPrediction.push_back(predictedSpeed);
    nextPrediction.push_back(i);
    nextPrediction.push_back(-1);
    predictions.push_back(nextPrediction);
  }
}

vector<double> SensorObject::getPrediction(int second) {
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

  if (predictions.size() < second) {
    // TODO: proper error handling !!
    cout << "Error in getting predictions -- too less predictions available" << endl;
    exit(0);
  }

  return predictions[second-1];
}

double SensorObject::getProbabilityOfLaneChangeLeft() {
  // to be implemented via naive Bayes approach
  return 0.0;
}

double SensorObject::getProbabilityOfLaneChangeRight() {
  // to be implemented via naive Bayes approach
  return 0.0;
}

double SensorObject::getProbabilityOfLaneKeep() {
  // to be implemented via naive Bayes approach
  return 1.0;
}


string SensorObject::display() {
  string result = "";

  result += "SensorObject ID = ";
  result += to_string(this->id);
  result += " | speed = ";
  // factor 2.23694 is to convert meters/second to miles/hour
  result += to_string(this->getSpeed() * 2.23694);
  result += " | lane = ";
  result += to_string(this->getLane());
  result += " | pos_cart = (";
  result += to_string(cartesian_x);
  result += ",";
  result += to_string(cartesian_y);
  result += ") ";
  result += " | s = ";
  result += to_string(this->frenet_s);

  return result;
}
