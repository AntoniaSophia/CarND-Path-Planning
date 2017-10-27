#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>
#include "AbstractVehicle.h"
#include "EgoVehicle.h"
#include "SensorObject.h"
#include "BehaviorPlanner.h"
#include "TrajectoryPlanner.h"
#include "TrajectoryPlannerWIP.h"
#include "helper.h"

using namespace std;

// for convenience
using json = nlohmann::json;

vector<string> man_str{"do nothing","acc" ,"decc" ,"left","right"};

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// For converting back and forth between radians and degrees.
double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int counter = 0;

int main() {
  uWS::Hub h;

    
  BehaviorPlanner planner(1); // initiate planner on lane 1
  planner.egoVehicle.setRefSpeed(22);
  
  TrajectoryPlanner trajectory;
  TrajectoryPlannerWIP trajWIP;
  trajWIP.loadMap();
  trajectory.loadMap();

  int counter = 0;

  h.onMessage([&trajWIP,&counter, &trajectory,&planner](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          planner.egoVehicle.update(j[1]);
          cout << planner.egoVehicle.display() << endl; 

          std::map<double,SensorObject>::iterator it;

          for (int i = 0; i < sensor_fusion.size(); i++) {
            it = planner.sensorObjects.find(sensor_fusion[i][0]);
            // already known Object
            if (it != planner.sensorObjects.end()){
              it->second.update(sensor_fusion[i]);
            }else{ 
              SensorObject so;
              so.update(sensor_fusion[i]);
              planner.sensorObjects[sensor_fusion[i][0]]=so;
            }
          }    

          Timer tmr;
          double t = tmr.elapsed();
          tmr.reset();
          planner.evaluateSituation();
          
          vector<double> proposedPath = planner.predictSituation(4);
          cout << "Planner --> "; 
          cout << " | d:  "  << proposedPath[0];
          cout << " | s:  "  << proposedPath[1];
          cout << " | lane:  "  << proposedPath[2];
          cout << " | speed:  "  << proposedPath[3];
          cout << " | second:  "  << proposedPath[4];
          cout << " | maneuvr:  "  << man_str[(int)proposedPath[5]];
          cout << " | eval:  "  << proposedPath[6];
          cout << endl;

          vector<double> maneuvrData;
          maneuvrData = planner.getManeuvrDataForTrajectory(proposedPath);

          double ref_vel = maneuvrData[1];
          int lane = maneuvrData[0];    
          cout << "Maneuvr --> "; 
          cout << " lane:  "  << lane;
          cout << " | speed:  "  << ref_vel;
          cout << endl;

          

          vector<vector<double>> next_vals{{},{}};
          bool takeSpline = true;

          if (takeSpline == false) {
            if (previous_path_x.size()==0 && counter == 0) {
              next_vals = trajWIP.getNextPathTrajectory(car_s, car_d,lane , 0, ref_vel, 2);
              next_vals = trajWIP.mergeTrajectories(previous_path_x, previous_path_y, end_path_s, end_path_d, next_vals);
            } else if (previous_path_x.size() < 20) {
              int time = 1;
              if (planner.stm_lane_change_completed() == false) {
                time = 3;
              }

              next_vals = trajWIP.getNextPathTrajectory(car_s, car_d,lane , planner.egoVehicle.getSpeed() , ref_vel, time);
              next_vals = trajWIP.mergeTrajectories(previous_path_x, previous_path_y, end_path_s, end_path_d, next_vals);

//              cout << "exiting...." << endl;
//              exit(0);
            } else {
              next_vals = trajWIP.mergeTrajectories(previous_path_x, previous_path_y, end_path_s, end_path_d, next_vals);
            }
          } else {
            next_vals = trajectory.calcTrajFromQA(planner.egoVehicle, ref_vel*2.237, lane);
          }

          // counter++;
          // if (counter > 1000) {
          //   cout << "exiting...." << endl;
          //   exit(0);
          // }
     

          t = tmr.elapsed();
          std::cout << "Time for trajectory update in (ms) " <<  t << endl;


    		  json msgJson;

          msgJson["next_x"] = next_vals[0];
          msgJson["next_y"] = next_vals[1];

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}