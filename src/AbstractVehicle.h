#pragma once

#include <math.h>
#include <vector>
#include <string>
#include "json.hpp"
#include <string>

using namespace std;
using nlohmann::json;

class AbstractVehicle {
  public:
    virtual double getSpeed() = 0;
    virtual double getSpeed_x() = 0;
    virtual double getSpeed_y() = 0;

    virtual string display() = 0;
    virtual void update(json j) = 0;
    virtual vector<double> getPrediction(int second) = 0;

    int getLane();
    double getFrenet_s() {return frenet_s;};
    double getFrenet_d() {return frenet_d;};
    double getCartesian_x() {return cartesian_x;};
    double getCartesian_y() {return cartesian_y;};  

    double getID() { return id;};

    virtual void calculatePredictions(int predictionHorizon) = 0;
    
protected:
    double id;
    double frenet_s;
    double frenet_d;
    double cartesian_x;
    double cartesian_y;
    int lastLane;
    int currentLane;
    vector<double> last_frenet_d;
    vector<double> last_frenet_s;
    int historyLength = 10;

    int predictionHorizonInSeconds = 10;
    vector<vector<double>> predictions;
    
};