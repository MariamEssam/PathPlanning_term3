#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "PathPlanningManager.h"
#include "Helper.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  //Manage all the path planning functionlity
  PathPlanningManager manager = PathPlanningManager(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
#ifdef UWS_VCPKG
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
	  &map_waypoints_dx, &map_waypoints_dy,&manager]
	  (uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length,
		  uWS::OpCode opCode)
#else
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
	  &map_waypoints_dx, &map_waypoints_dy, &manager]
	  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
		  uWS::OpCode opCode)
#endif
 {
	  Helper helper;
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
	  
      auto s = helper.hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  double last_s = car_s;

		  int prev_size = previous_path_x.size();

		  if (prev_size > 0) {
			  last_s = end_path_s;
		  }
		  manager.updatewatchinglist(sensor_fusion);
		  manager.update_auto_car_state(last_s, car_x, car_y, car_yaw, car_s, car_d, car_speed);
		  manager.generatePath(previous_path_x, previous_path_y);
		/*  double dist_inc = 0.5;
		  for (int i = 0; i < 50; ++i) {
			  next_x_vals.push_back(car_x + (dist_inc*i)*cos(car_yaw* M_PI / 180));
			  next_y_vals.push_back(car_y + (dist_inc*i)*sin(car_yaw* M_PI / 180));
		  }*/
		 /* msgJson["next_x"] = next_x_vals;
		  msgJson["next_y"] = next_y_vals;*/
		  msgJson["next_x"] = manager.get_x_values();
		  msgJson["next_y"] = manager.get_y_values();

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
#ifdef UWS_VCPKG
		  ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
          
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef UWS_VCPKG
		ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
        
      }
    }  // end websocket if
  }); // end h.onMessage
#ifdef UWS_VCPKG
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req)
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
#endif
  {
    std::cout << "Connected!!!" << std::endl;
  });
#ifdef UWS_VCPKG
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code,
                         char *message, size_t length)
#else
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
	  char *message, size_t length)
#endif
  {
#ifdef UWS_VCPKG
	  ws->close();
#else
	  ws.close();
#endif
    
    std::cout << "Disconnected" << std::endl;
  });
#ifdef UWS_VCPKG
  int port = 4567;
  auto host = "127.0.0.1";
  if (h.listen(host, port))
#else
  int port = 4567;
  if (h.listen(port))
#endif
   {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}