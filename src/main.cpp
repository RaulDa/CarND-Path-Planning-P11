#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "PathPlanner.h"

using namespace std;

// For convenience
using json = nlohmann::json;

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

int main() {
  uWS::Hub h;

  // Create path planner
  PathPlanner path_plan;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // Lane: 0 -> left, 1 -> center, 2 -> right
  int lane = 1;

  // Reference velocity to target in mph
  double ref_vel = 0;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &path_plan](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	carCoordinates car_coord;
        	// Main car's localization Data
          	car_coord.car_x = j[1]["x"];
          	car_coord.car_y = j[1]["y"];
          	car_coord.car_s = j[1]["s"];
          	car_coord.car_d = j[1]["d"];
          	car_coord.car_yaw = j[1]["yaw"];
          	car_coord.car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto prev_path_x = j[1]["previous_path_x"];
          	auto prev_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];
          	vector<vector<double>> sensor_fusion_ = sensor_fusion;

          	// Put previous path data into structure for sending to PathPlanner
          	previousPath previous_path;
          	vector<double> p_x = prev_path_x;
          	vector<double> p_y = prev_path_y;
          	previous_path.previous_path_x = p_x;
          	previous_path.previous_path_y = p_y;

          	// Get size of previous path
          	int prev_size = previous_path.previous_path_x.size();

            // Store last s value if previous path available
          	if(prev_size > 0){
          		car_coord.car_s = end_path_s;
          	}

          	// Behavior planning implemented via state machine
          	path_plan.behavior_planning(sensor_fusion_, car_coord, prev_size, &lane, &ref_vel);

            // Map waypoints to vector of vectors
          	vector<vector<double>> map_waypoints;
          	map_waypoints.push_back(map_waypoints_s);
          	map_waypoints.push_back(map_waypoints_x);
          	map_waypoints.push_back(map_waypoints_y);

          	// Generate 5 spline points
          	splinePointsReturn splinePointsStr = path_plan.generate_spline_points(prev_size, car_coord, previous_path, lane, map_waypoints);

          	// Generate trajectory from calculated spline points
          	vector<vector<double>> next_vals = path_plan.generate_trajectory(splinePointsStr, previous_path, ref_vel);

          	json msgJson;

          	// Send generated trajectory to simulator
          	msgJson["next_x"] = next_vals[0]; // x points
          	msgJson["next_y"] = next_vals[1]; // y points

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
