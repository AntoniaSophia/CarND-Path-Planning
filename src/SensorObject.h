#pragma once

#include "AbstractVehicle.h"
#include "json.hpp"
#include <math.h>
#include <string>
#include <vector>
using namespace std;
using nlohmann::json;

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
  double speed_x;
  double speed_y;

  double getProbabilityOfLaneChangeLeft();
  double getProbabilityOfLaneChangeRight();
  double getProbabilityOfLaneKeep();
};
