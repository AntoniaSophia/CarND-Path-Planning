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

#include "EgoVehicle.h"
#include "SensorObject.h"

using std::vector;
using nlohmann::json;

/**
 * Base class for the behavior planning 
 */
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
    vector<vector<vector<double>>> iteratePredictions(vector<double> start , vector<vector<double>> history, int predictionHorizon);

    int stm_target_lane;                          /**< Statemachine variable to keep the target lane for take over maneuvr */
    bool stm_is_in_lane_change;                   /**< Statemachine variable which indicates an ongoing take over maneuvr */
    bool stm_ready_for_lane_change;               /**< Statemachine variable to indicate the readiness of the egovehicle for lane changing */
 
    bool isLaneChangeSafe(int lane);

    double evaluateLane_Weight = 1.0;
    double evaluateSafetyDistance_Weight = 100.0;
    double evaluateSpeed_Weight = 1.0;
    double evaluateFrenet_d_Weight = 0.0;

};