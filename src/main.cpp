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
          
          //////**TRAJECTORY GENERATION**/////
		  //Vectors to sent to the simulator as next moves for the car
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //Initialize constants
          double safety = 1.1;
          double max_speed = 22.352; //maximum speed in m/s (50 MPH)
          double time_step = 0.02; //seconds
          double dist_step = max_speed*time_step/safety; //max change in distance allowed between each point
          vector<double> lane_mults = {2,6,10}; //multipliers of d vector to be in the center of lanes 0, 1, 2 resp.
          int current_lane;	//the lane the car is currently in (1, 2, or 3)

          //Collect values for spline
          vector<double> spline_x;
          vector<double> spline_y;
          //Count number of points left
          int points_left = previous_path_x.size();
          std::cout << "Points left: " << points_left << std::endl;
          //Add un-processed points to path to send to simulator
          if (points_left>0){
          	for (int i=0; i<points_left;i++){
          		next_x_vals.push_back(previous_path_x[i]);
          		next_y_vals.push_back(previous_path_y[i]);
          	}
          }  
          //Use previous points for smoothness in spline calc
          if (points_left<2) {
    	  	//Use 1 points straight behind and current position if <2 un-processed points
    		double prev_car_x = car_x - cos(car_yaw);
    		double prev_car_y = car_y - sin(car_yaw);

    		spline_x.push_back(prev_car_x);
    		spline_x.push_back(car_x);

    		spline_y.push_back(prev_car_y);
    		spline_y.push_back(car_y);
		  }
		  else {
            for (int i=0; i<points_left;i++){
              spline_x.push_back(previous_path_x[i]);
          	  spline_y.push_back(previous_path_y[i]);
            }
          }
          //Use points down the road to calculate spline
          vector<double> wp1 = getXY(car_s + 25, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp2 = getXY(car_s + 50, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //double wp1_x = map_waypoints_x[wp1] + map_waypoints_dx[wp1]*lane_mults[current_lane];
          //double wp1_y = map_waypoints_y[wp1] + map_waypoints_dy[wp1]*lane_mults[current_lane];
          //double wp2_x = wp2[0];
          //double wp2_y = wp2[1];
          spline_x.push_back(wp1[0]);
          spline_x.push_back(wp2[0]);
          spline_y.push_back(wp1[1]);
          spline_y.push_back(wp2[1]);
          for (int i=0; i<spline_x.size();i++){
          	std::cout << "spline x: "<< spline_x[i] <<std::endl;
          	std::cout << "spline y: "<< spline_y[i] <<std::endl;
          }
          //Compute the spline and fill next x & y based on calculated positions at 0.02 s intervals
          tk::spline path;
          path.set_points(spline_x,spline_y);

          //Get 1 second of path data, including any un-processed points from last trajectory
          double last_x;
          if (points_left !=0){
            last_x = next_x_vals.back(); //last x point in trajectory
          }
          else {last_x = car_x;}
          double dist_to = distance(wp1[0], wp1[1], car_x, car_y); //distance to next reference
          double n_steps = dist_to/dist_step; //# steps to get to target at desired speed
          double x_step = (wp1[0]-car_x)/n_steps;
          for (int i=0; i<50-points_left; i++){
            //find the x value corresponding to s value of allowable speed
            last_x += x_step;
            next_x_vals.push_back(last_x);
            std::cout << "next_x: " << last_x << std::endl;
            next_y_vals.push_back(path(last_x));
            std::cout << "next_y: " << path(last_x) << std::endl;
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