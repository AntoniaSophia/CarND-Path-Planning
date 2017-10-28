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

#include "BehaviorPlanner.h"
#include <set>

#include "spdlog/spdlog.h"

extern vector<string> man_str;

/**
 * @brief Constructor, instaniate with initial lane
 *
 * @param initialLane
 */
BehaviorPlanner::BehaviorPlanner(int initialLane) {
  stm_target_lane = initialLane;
  stm_is_in_lane_change = false;
  stm_ready_for_lane_change = true;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool BehaviorPlanner::stm_isReadyForLaneChange() {
  return stm_ready_for_lane_change;
}

/**
 * @brief
 *
 * @param targetLane
 */
void BehaviorPlanner::stm_initiateLaneChange(int targetLane) {
  if (stm_isReadyForLaneChange() == true) {
    stm_target_lane = targetLane;
    stm_ready_for_lane_change = false;
    stm_is_in_lane_change = true;
  }
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool BehaviorPlanner::stm_lane_change_completed() {
  if (stm_is_in_lane_change == false) {
    return true;
  }

  double threshold = 0.10;

  if (stm_target_lane == 0) {
    double diff_frenet_d = egoVehicle.getFrenet_d() - 2;
    if (abs(diff_frenet_d) < threshold) {
      stm_ready_for_lane_change = true;
      stm_is_in_lane_change = false;
      spdlog::get("console")->debug("lane change finished on to lane 0  !! ");
      return true;
    }
  } else if (stm_target_lane == 1) {
    double diff_frenet_d = egoVehicle.getFrenet_d() - 6;
    if (abs(diff_frenet_d) < threshold) {
      stm_ready_for_lane_change = true;
      stm_is_in_lane_change = false;
      spdlog::get("console")->debug("lane change finished on to lane 1  !! ");
      return true;
    }
  } else if (stm_target_lane == 2) {
    double diff_frenet_d = egoVehicle.getFrenet_d() - 10;
    if (abs(diff_frenet_d) < threshold) {
      stm_ready_for_lane_change = true;
      stm_is_in_lane_change = false;
      spdlog::get("console")->debug("lane change finished on to lane 2  !! ");
      return true;
    }
  }

  return false;
}

/**
 * @brief
 *
 * @param predictionHorizon
 * @return vector<double>
 */
vector<double> BehaviorPlanner::predictSituation(int predictionHorizon) {
  stm_lane_change_completed();

  vector<vector<double>> history;
  vector<vector<vector<double>>> predictedPaths;
  predictedPaths = iteratePredictions(egoVehicle.getPrediction(0), history,
                                      predictionHorizon);

  vector<double> bestEvaluations;
  int predictionDepth = 0;

  spdlog::get("console")->debug("Visualize next prediction");
  for (int predictionDepth = 0; predictionDepth < predictionHorizon;
       predictionDepth++) {
    double bestEvaluation = 10;
    double nextEvaluation;
    for (int path = 0; path < predictedPaths.size(); path++) {
      vector<vector<double>> nextPath = predictedPaths[path];
      nextEvaluation = nextPath[predictionDepth][6];

      if (nextEvaluation < bestEvaluation) {
        bestEvaluation = nextEvaluation;
      }
    }

    bestEvaluations.push_back(bestEvaluation);
  }

  map<int, double> pathEvalMap;

  for (int path = 0; path < predictedPaths.size(); path++) {
    vector<vector<double>> nextPath = predictedPaths[path];
    double evalDiff = 0.0;
    for (int maneuvr = 0; maneuvr < nextPath.size(); maneuvr++) {
      evalDiff += pow(nextPath[maneuvr][6] - bestEvaluations[maneuvr], 2);
    }
    pathEvalMap[path] = evalDiff;
  }
  typedef std::function<bool(std::pair<int, double>, std::pair<int, double>)>
      Comparator;

  // Defining a lambda function to compare two pairs. It will compare two pairs
  // using second field
  Comparator compFunctor = [](std::pair<int, double> elem1,
                              std::pair<int, double> elem2) {
    return elem1.second < elem2.second;
  };

  // Declaring a set that will store the pairs using above comparision logic
  std::set<std::pair<int, double>, Comparator> sortpathEvalMap(
      pathEvalMap.begin(), pathEvalMap.end(), compFunctor);

  int topPaths = 1;
  int counter = 0;
  for (auto it = sortpathEvalMap.begin(); it != sortpathEvalMap.end(); ++it) {
    counter++;
    if (counter > topPaths) {
      break;
    }

    vector<vector<double>> nextPath = predictedPaths[it->first];

    spdlog::get("console")->debug("Path Number {}", it->first);

    for (int maneuvr = 0; maneuvr < nextPath.size(); maneuvr++) {
      vector<double> nextManeuvr = nextPath[maneuvr];

      spdlog::get("console")->debug(
          "--> maneuvr {} | d: {} | s: {} | lane: {} | speed: {} | second: {} "
          "| maneuvr: {} | eval: {}",
          maneuvr, nextManeuvr[0], nextManeuvr[1], nextManeuvr[2],
          nextManeuvr[3], nextManeuvr[4], man_str[(int)nextManeuvr[5]],
          nextManeuvr[6]);
    }

    if (nextPath[0][3] > (egoVehicle.getRefSpeed() - 0.3)) {
      nextPath[0][3] = egoVehicle.getRefSpeed() - 0.3;
    }
    return nextPath[0];
  }
}

/**
 * @brief
 *
 * @param start
 * @param history
 * @param predictionHorizon
 * @return vector<vector<vector<double>>>
 */
vector<vector<vector<double>>>
BehaviorPlanner::iteratePredictions(vector<double> start,
                                    vector<vector<double>> history,
                                    int predictionHorizon) {
  /**
   * \note what is required for a prediction?
   * - frenet_d   [0]
   * - frenet_s   [1]
   * - lane       [2]
   * - speed      [3]
   * - second     [4]
   * - maneuvr    [5]  (0 = do nothing , 1 = acc , 2 = decc , 3 = left , 4 =
   * right )
   * - evaluation [6]
   */
  double currentFrenet_d = start[0];
  double currentFrenet_s = start[1];
  int currentLane = start[2];
  double currentSpeed = start[3];
  int currentSecond = start[4];

  vector<vector<vector<double>>> result;
  vector<double> nextPrediction;

  /*****************************
  // do nothing
  *****************************/
  nextPrediction.push_back(currentFrenet_d);
  nextPrediction.push_back(currentFrenet_s +
                           currentSpeed * 1); // speed is in meters per second!
  nextPrediction.push_back(currentLane);
  nextPrediction.push_back(currentSpeed);
  nextPrediction.push_back(currentSecond + 1);
  nextPrediction.push_back(0);
  nextPrediction.push_back(
      evaluateSituation(nextPrediction, currentSecond + 1) - 0.001);
  history.push_back(nextPrediction);
  result.push_back(vector<vector<double>>(history));
  history.pop_back();
  nextPrediction.clear();

  /*****************************
  // accelerate (maximum 10m/s²)
  *****************************/
  if (currentSpeed < egoVehicle.getRefSpeed()) {
    double acc = 7;
    double speedDiff = egoVehicle.getRefSpeed() - currentSpeed;
    if (speedDiff < 7) {
      acc = speedDiff;
    }

    nextPrediction.push_back(currentFrenet_d);
    nextPrediction.push_back(currentFrenet_s + currentSpeed * 1 +
                             acc / 2.0); // speed is in meters per second!
    nextPrediction.push_back(currentLane);
    nextPrediction.push_back(currentSpeed + acc);
    nextPrediction.push_back(currentSecond + 1);
    nextPrediction.push_back(1);
    nextPrediction.push_back(
        evaluateSituation(nextPrediction, currentSecond + 1));

    history.push_back(nextPrediction);
    result.push_back(vector<vector<double>>(history));
    history.pop_back();
    nextPrediction.clear();
  }

  /*****************************
  // decelerate (maximum 10m/s²)
  *****************************/
  if (currentSpeed > 7) {
    double dec = -7;
    nextPrediction.push_back(currentFrenet_d);
    nextPrediction.push_back(currentFrenet_s + currentSpeed * 1 +
                             dec / 2.0); // speed is in meters per second!
    nextPrediction.push_back(currentLane);
    nextPrediction.push_back(currentSpeed + dec);
    nextPrediction.push_back(currentSecond + 1);
    nextPrediction.push_back(2);
    nextPrediction.push_back(
        evaluateSituation(nextPrediction, currentSecond + 1) + 0.1);

    history.push_back(nextPrediction);
    result.push_back(vector<vector<double>>(history));
    history.pop_back();
    nextPrediction.clear();
  }

  /*****************************
  // change to left lane
  *****************************/
  if (currentLane > 0) {
    if (currentSecond > 0 || (currentSecond = 0 && isLaneChangeSafe(currentLane-1) == true)) {
      nextPrediction.push_back(currentFrenet_d - 4);
      nextPrediction.push_back(currentFrenet_s + currentSpeed - 1); // speed is in meters per second!
      nextPrediction.push_back(currentLane-1);
      nextPrediction.push_back(currentSpeed);
      nextPrediction.push_back(currentSecond+1);
      nextPrediction.push_back(3);
      nextPrediction.push_back(evaluateSituation(nextPrediction,currentSecond+1)+0.05);
        
      history.push_back(nextPrediction);
      result.push_back(vector<vector<double>>(history));
      history.pop_back();
      nextPrediction.clear();
    }
  }

  /*****************************
  // change to right lane
  *****************************/
  if (currentLane < 2) {
    if (currentSecond > 0 || (currentSecond = 0 && isLaneChangeSafe(currentLane+1) == true)) {
      nextPrediction.push_back(currentFrenet_d + 4);
      nextPrediction.push_back(currentFrenet_s + currentSpeed - 1); // speed is in meters per second!
      nextPrediction.push_back(currentLane+1);
      nextPrediction.push_back(currentSpeed);
      nextPrediction.push_back(currentSecond+1);
      nextPrediction.push_back(4);
      nextPrediction.push_back(evaluateSituation(nextPrediction,currentSecond+1)+0.05);
        
      history.push_back(nextPrediction);
      result.push_back(vector<vector<double>>(history));
      history.pop_back();
      nextPrediction.clear();
    }
  }

  if (currentSecond + 1 < predictionHorizon) {
    // loop through all elements of result and call this function again
    vector<vector<vector<double>>> sumOfResults;
    for (int path = 0; path < result.size(); path++) {
      vector<vector<double>> nextPath = result[path];
      vector<double> lastManeuvr = nextPath.back();
      vector<vector<vector<double>>> newResult =
          iteratePredictions(lastManeuvr, nextPath, predictionHorizon);
      for (int i = 0; i < newResult.size(); i++) {
        sumOfResults.push_back(vector<vector<double>>(newResult[i]));
      }
    }
    return sumOfResults;
  } else {
    return result;
  }
}

bool BehaviorPlanner::isLaneChangeSafe(int lane) {
  if (egoVehicle.getLane() - lane > 1) {
    // TODO: error handling --> no jump in lane change....
    return false;
  } else if (egoVehicle.getLane() - lane == 0) {
    // TODO: error handling --> no lane change !?
    return false;
  }

  // First check wheter we are too close to a vehicle in front of us
  double closestDistanceInLane = 7000;

  for(auto it = sensorObjects.begin(); it != sensorObjects.end(); ++it) {
    if (it->second.getLane() == egoVehicle.getLane()) {

    }
  }   
  
  return true;
}


double BehaviorPlanner::evaluateSituation() {
  return evaluateSituation(egoVehicle.getPrediction(0), 0);
}

/**
 * @brief
 *
 * @param egoPrediction
 * @param predictionHorizon
 * @return double
 */
double BehaviorPlanner::evaluateSituation(vector<double> egoPrediction,
                                          int predictionHorizon) {
  double result = 0.0;
  int freeLaneBonus = 0;

  spdlog::get("console")->debug("sensorObjects contains {} elements.",
                                sensorObjects.size());
  for (auto it = sensorObjects.begin(); it != sensorObjects.end(); ++it) {
    freeLaneBonus += evaluateFreeLaneBonus(
        egoPrediction, it->second.getPrediction(predictionHorizon));

    double safetyDistanceCost = evaluateSafetyDistance(
        egoPrediction, it->second.getPrediction(predictionHorizon));
    result += safetyDistanceCost;
    if (safetyDistanceCost > 0) {
      spdlog::get("console")->debug("! S = {}", to_string(safetyDistanceCost));
    }
  }

  double bonus = 0.0;
  if (freeLaneBonus == 0) {
    bonus = -0.1;
    result += bonus;

    if (result < 0) {
      result = 0.0;
    }
  }

  double laneCost = evaluateLane(egoPrediction);
  double speedCost = evaluateSpeed(egoPrediction, egoVehicle.getRefSpeed());
  result += speedCost;

  double frenet_dCost = evaluateFrenet_d(egoPrediction);
  result += frenet_dCost;
  spdlog::get("console")->debug(
      "BehaviorPlanner reports value {}  | LaneCost = {} | SpeedCost = {} | d "
      "Cost = {} | Free Lane Bonus = {}",
      result, laneCost, speedCost, frenet_dCost, bonus);

  return result;
}

/**
 * @brief
 *
 * @param egoPrediction
 * @return double
 */
double BehaviorPlanner::evaluateLane(vector<double> egoPrediction) {
  double result = 0.0;
  result = (2 - egoPrediction[2]) / 200.0; // punish driving left lane slightly
  return result * evaluateLane_Weight;
}

/**
 * @brief
 *
 * @param egoPrediction
 * @param objectPrediction
 * @return int
 */
int BehaviorPlanner::evaluateFreeLaneBonus(vector<double> egoPrediction,
                                           vector<double> objectPrediction) {
  if (egoPrediction[2] != objectPrediction[2]) {
    return 0;
  }

  double distance = abs(objectPrediction[1] - egoPrediction[1]);

  if (distance > 80) {
    return 0;
  }
  return 1;
}


/**
 * @brief
 *
 * @param egoPrediction
 * @param objectPrediction
 * @return double
 */
double
BehaviorPlanner::evaluateSafetyDistance(vector<double> egoPrediction,
                                        vector<double> objectPrediction) {

  // check whether both objects are on same lane
  if (egoPrediction[2] != objectPrediction[2]) {
    return 0.0;
  }

  double result = 0.0;
  double safetyDistance;

  // calculate the safety distance always from the faster vehicle!!
  if (egoPrediction[3] > objectPrediction[3]) {
    safetyDistance = egoPrediction[3]  * 3.6 * 0.5; // 3.6 = meters per second into kilometers per hour
  } else {
    safetyDistance = objectPrediction[3]  * 3.6 * 0.5; // 3.6 = meters per second into kilometers per hour
  }

  double distance = abs(egoPrediction[1] - objectPrediction[1]);

  spdlog::get("console")->debug(
      "Safety distance = {} | Distance = {} | EgoSpeed: {}", safetyDistance,
      distance, egoPrediction[3]);

  if (distance > safetyDistance) {
    return 1.0/distance;
  }

  // ok, now assume the distance is smaller than the safety distance
  result = 1.0/distance;
  return result * evaluateSafetyDistance_Weight;
}

/**
 * @brief
 *
 * @param egoPrediction
 * @param refSpeed
 * @return double
 */
double BehaviorPlanner::evaluateSpeed(vector<double> egoPrediction,
                                      double refSpeed) {
  double result = 0.0;

  double speedDiff = abs(refSpeed - egoPrediction[3]);

  if (refSpeed > egoPrediction[3]) {
    result = abs(speedDiff) / 100.0;
  } else {
    result = abs(speedDiff) / 100.0;
  }
  return result * evaluateSpeed_Weight;
}

/**
 * @brief
 *
 * @param egoPrediction
 * @return double
 */
double BehaviorPlanner::evaluateFrenet_d(vector<double> egoPrediction) {
  double result = 0.0;
  int lane = egoPrediction[2];
  double frenet_d = egoPrediction[0];

  if (frenet_d < 0) {
    return 1.0;
  } else if (frenet_d < 1) {
    return 0.5;
  } else if (frenet_d > 11) {
    return 0.5;
  } else if (frenet_d > 12) {
    return 1.0;
  }

  if (lane == 0) {
    result += abs((frenet_d - 2)) / 10;
  } else if (lane == 1) {
    result += abs((frenet_d - 6)) / 10;
  } else if (lane == 2) {
    result += abs((frenet_d - 10)) / 10;
  }

  return result * evaluateFrenet_d_Weight;
}

/**
 * @brief
 *
 * @param proposedPath
 * @return vector<double>
 */
vector<double>
BehaviorPlanner::getManeuvrDataForTrajectory(vector<double> proposedPath) {
  vector<double> result;
  int lane;
  double speed = proposedPath[3];

  if ((int)proposedPath[5] == 3 || (int)proposedPath[5] == 4) {
    spdlog::get("console")->debug(
        "planner proposes lane change from lane {} to lane {} ",
        egoVehicle.getLane(), (int)proposedPath[2]);

    if ((int)proposedPath[2] != egoVehicle.getLane()) {
      spdlog::get("console")->debug(
          "stm_isReadyForLaneChange() = {} | stm_is_in_lane_change = {} | "
          "stm_target_lane = {}",
          stm_isReadyForLaneChange(), stm_is_in_lane_change, stm_target_lane);

      if (stm_isReadyForLaneChange() == true) {
        spdlog::get("console")->debug("executing lane change");
        stm_initiateLaneChange((int)proposedPath[2]);
        lane = (int)proposedPath[2];
      } else {
        lane = stm_target_lane;
      }
    } else {
      lane = egoVehicle.getLane();
    }
  } else {
    if (stm_isReadyForLaneChange() == true) {
      lane = egoVehicle.getLane();
    } else {
      lane = stm_target_lane;
    }
  }

  result.push_back(lane);
  result.push_back(speed);

  return result;
}