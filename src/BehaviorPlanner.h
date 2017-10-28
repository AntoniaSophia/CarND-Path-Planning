#pragma once

#include "json.hpp"
#include <math.h>
#include <vector>
#include "AbstractVehicle.h"
#include "EgoVehicle.h"
#include "SensorObject.h"
#include "json.hpp"
#include <string>

using namespace std;
using nlohmann::json;

class BehaviorPlanner{
 public:
   BehaviorPlanner(int initialLane);
   double evaluateSituation();
   double evaluateSituation(vector<double> egoPrediction , int predictionHorizon);
   vector<double> predictSituation(int predictionHorizon);
   
   double evaluateLane(vector<double> egoPrediction);
   double evaluateSafetyDistance(vector<double> egoPrediction , vector<double> objectPrediction);
   double evaluateSpeed(vector<double> egoPrediction, double refSpeed);
   double evaluateFrenet_d(vector<double> egoPrediction);
   int evaluateFreeLaneBonus(vector<double> egoPrediction , vector<double> objectPrediction);
   std::map<double,SensorObject> getSensorObjects() { return sensorObjects;} ;
   std::map<double,SensorObject> sensorObjects;
   EgoVehicle egoVehicle;

   vector<double> getManeuvrDataForTrajectory(vector<double> proposedPath);

   bool stm_lane_change_completed();
   
 
  protected:
    bool stm_isReadyForLaneChange();
    void stm_initiateLaneChange(int targetLane);
    int stm_target_lane;
    bool stm_is_in_lane_change;
    bool stm_ready_for_lane_change;
 
    bool isLaneChangeSafe(int lane);

    double evaluateLane_Weight = 0.0;
    double evaluateSafetyDistance_Weight = 10.0;
    double evaluateSpeed_Weight = 1.0;
    double evaluateFrenet_d_Weight = 0.0;

    vector<vector<vector<double>>> iteratePredictions(vector<double> start , vector<vector<double>> history, int predictionHorizon);
  
};