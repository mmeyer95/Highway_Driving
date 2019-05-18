#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

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
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

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
		  //initialize for a straight-ahead move
          //float max_S_change = 0.4;
          if (end_path_s!=0 && end_path_d!=0){
            double d = end_path_d;
            double s = end_path_s;
          }
          else {
            double d = car_d;
          	double s = car_s;
          }
          vector<double> spline_x;
          vector<double> spline_y;
          vector<double> spline_time;
          //point at the end of last move
          vector<double> XY = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          spline_x.push_back(XY[0]);
          spline_y.push_back(XY[1]);
          spline_time.push_back(0);
          //point 1 down the road
          s = s+1;
          XY = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          spline_x.push_back(XY[0]);
          spline_y.push_back(XY[1]);
          spline_time.push_back(0.5);
          //point 2 down the road
          s = s+1;
          XY = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          spline_x.push_back(XY[0]);
          spline_y.push_back(XY[1]);
          spline_time.push_back(1);
          //find next points in s & d for straight-away, convert to x*y, add to spline arrays
          //for (int i=0; i<10; i++){
          //s = s + max_S_change;
          //vector<double> XY = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //std::cout << "s=" << s <<std::endl;
          //std::cout << "d=" << d <<std::endl;
          //std::cout << "X=" << XY[0] <<std::endl;
          //std::cout << "Y=" << XY[1] <<std::endl;
          //spline_x.push_back(XY[0]);
          //spline_y.push_back(XY[1]);
          //}
          //compute the spline and fill next x & y based on calculated positions at 0.2 s intervals
          tk::spline s_x;
          s_x.set_points(spline_time,spline_x);
          tk::spline s_y;
          s_y.set_points(spline_time, spline_y);
          for (int i=1; i<11; i++){
            next_x_vals.push_back(s_x(0.2*i));
            //std::cout << "next_x: " << s_x(0.2*i) << std::endl;
            next_y_vals.push_back(s_y(0.2*i));
            //std::cout << "next_y: " << s_y(0.2*i) << std::endl;
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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