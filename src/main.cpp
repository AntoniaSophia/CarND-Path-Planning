#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <chrono>
#include <thread>
#include <uWS/uWS.h>
#include "BehaviorPlanner.h"
#include "TrajectoryPlanner.h"
#include "TrajectoryPlannerWIP.h"
#include "helper.h"
#include "spdlog/spdlog.h"

using std::vector;
using std::string;

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

  auto console = spdlog::basic_logger_mt("console","log.txt");
  //auto console = spdlog::stdout_color_mt("console");
  console->set_level(spdlog::level::info);

  uWS::Hub h;

    
  BehaviorPlanner planner(1); // initiate planner on lane 1
  planner.egoVehicle.setRefSpeed(22); // set the maximum and desired speed of 22 meters per second
  
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
          spdlog::get("console")->info("{}",planner.egoVehicle.display()); 

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
          
          vector<double> proposedPath = planner.predictSituation(4);  // predicts the future of 4 seconds
          spdlog::get("console")->info("Planner --> d:  {} | s:  {} | lane:  {} | speed: {} | second: {} | maneuvr: {} | eval: {}",
            proposedPath[0], proposedPath[1], proposedPath[2], proposedPath[3], proposedPath[4], man_str[(int)proposedPath[5]], proposedPath[6]
        );

          vector<double> maneuvrData;
          maneuvrData = planner.getManeuvrDataForTrajectory(proposedPath); // choose best maneuvr sequence for trajectory calculation


          double ref_vel = maneuvrData[1];
          int lane = maneuvrData[0];    
          spdlog::get("console")->info("Maneuvr --> lane: {} | speed: {}", lane, ref_vel);
          

          vector<vector<double>> next_vals{{},{}};
          bool takeSpline = true;  // set this variable to "false" in order to use the JMT trajectory

          if (takeSpline == false) {
            if (previous_path_x.size()==0 && counter == 0) {
              // this is the starting sequence
              next_vals = trajWIP.getNextPathTrajectory(car_s, car_d,lane , 0, ref_vel, 2);
              next_vals = trajWIP.mergeTrajectories(previous_path_x, previous_path_y, end_path_s, end_path_d, next_vals);
            } else if (previous_path_x.size() < 25) {
              // re-generate new points if only 25 are available

              // use 2 seconds for a trajectory, in case of a lane change take 3 seconds
              int time = 2;
              if (planner.stm_lane_change_completed() == false) {
                time = 3;
              }

              next_vals = trajWIP.getNextPathTrajectory(car_s, car_d,lane , planner.egoVehicle.getSpeed() , ref_vel, time);
              next_vals = trajWIP.mergeTrajectories(previous_path_x, previous_path_y, end_path_s, end_path_d, next_vals);

            } else {
              next_vals = trajWIP.mergeTrajectories(previous_path_x, previous_path_y, end_path_s, end_path_d, next_vals);
            }
          } else {
            next_vals = trajectory.calcTrajFromQA(planner.egoVehicle, ref_vel*2.237, lane);
          }

          t = tmr.elapsed();
          spdlog::get("console")->debug("Time for trajectory update: {} ms",t);

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
    spdlog::get("console")->info("Connected!!!");
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    spdlog::get("console")->info("Disconnected");
  });

  int port = 4567;
  if (h.listen(port)) {
    spdlog::get("console")->info("Listening to port {}",port);
  } else {
    spdlog::get("console")->error("Failed to listen to port {}",port);
    return -1;
  }
  h.run();
}