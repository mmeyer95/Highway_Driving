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
          
          /////**BEHAVIOR PLANNING**/////
		  //vector<int> moves = {-1,0,1};	//lane change values corresponding to possible moves (left switch, stay, right)
          double max_speed = 22.352; //maximum speed in m/s (50 MPH)
          vector<int> behavior; //previously decided upon behaviors in terms of moving lanes
          vector<double> lane_mults = {2,6,10}; //multipliers of d vector to be in the center of lanes 0, 1, 2 resp.
          int current_lane=0;	//the lane the car is currently in (0, 1, or 2)
          //figure out what lane the car is in based on the current d value
		  if (car_d>=0 && car_d<4){current_lane=0;}
          if (car_d>=4 && car_d<8){current_lane=1;}
          if (car_d>=8 && car_d<12){current_lane=2;}
          //Find the closest cars to determine whether there are any in the same lane or left/right
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;
          vector<double> front_neighbor;
          vector<double> left_neighbor;
          vector<double> right_neighbor;
          for (int i=0; i<sensor_fusion.size();i++){
            vector<double> other_car = sensor_fusion[i];
            std::cout << "other car: " << other_car << std::endl;
            int other_lane=0;
            if (other_car[6]>=0 && other_car[6]<4){other_lane=0;}
          	if (other_car[6]>=4 && other_car[6]<8){other_lane=1;}
          	if (other_car[6]>=8 && other_car[6]<12){other_lane=2;}
            if (other_lane==current_lane){
              car_ahead = other_car[5]>car_s && (other_car[5]-car_s)<30;
              if (car_ahead==true){front_neighbor=other_car;}
            }
            else if (other_lane==current_lane-1){
              car_left = other_car[5]>car_s && (other_car[5]-car_s)<30;
              if (car_left==true){left_neighbor=other_car;}
            }
            else if (other_lane==current_lane+1){
              car_right = other_car[5]>car_s && (other_car[5]-car_s)<30;
              if (car_right==true){right_neighbor=other_car;}
            }
          }
          //decide what to do based on determined proximity to other cars
          if (car_ahead==false){behavior.push_back(0);}
          else if (car_ahead==true){
            std::cout << "Car Ahead" <<std::endl;
            double ahead_car_velocity = sqrt(front_neighbor[3]*front_neighbor[3]+front_neighbor[4]*front_neighbor[4]);
            //if (ahead_car_velocity >= car_speed){behavior.push_back(0);} //stay in lane
            if(car_left==false && current_lane>0){behavior.push_back(-1);} //merge left
            else if(car_right==false && current_lane<1){behavior.push_back(1);} //merge right
            else {behavior.push_back(0); max_speed = ahead_car_velocity;} //stay in line, slow down
          }
          //define target lane
		  int desired_lane = current_lane + behavior.back(); //where the car should be going
		  //std::cout << "Desired lane: " << desired_lane << std::endl;
          
          //////**TRAJECTORY GENERATION**/////
		  //Vectors to sent to the simulator as next moves for the car
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //Initialize constants
          double safety = 1.1; //factor of safety for speed
          double time_step = 0.02; //seconds time step
          double dist_step = max_speed*time_step/safety; //max change in distance allowed between each point
          
          //Collect values for spline
          vector<double> spline_x;
          vector<double> spline_y;
          //Count number of points left
          int points_left = previous_path_x.size();
          //--std::cout << "Points left: " << points_left << std::endl;
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
		  else { //add the points from last trajectory to points for spline calc
            for (int i=0; i<points_left;i++){
              spline_x.push_back(previous_path_x[i]);
          	  spline_y.push_back(previous_path_y[i]);
            }
          }
          
          //Use points down the road to calculate spline
          vector<double> wp1 = getXY(car_s + 25, lane_mults[desired_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp2 = getXY(car_s + 50, lane_mults[desired_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);
          spline_x.push_back(wp1[0]);
          spline_x.push_back(wp2[0]);
          spline_y.push_back(wp1[1]);
          spline_y.push_back(wp2[1]);

          //Compute the spline and fill next x & y based on calculated positions at 0.02 s intervals
          tk::spline path;
          path.set_points(spline_x,spline_y);

          //Get 1 second of path data, including any un-processed points from last trajectory
          double last_x; //last x position of car as calculated- either end of last traj, or current pos.
          if (points_left !=0){
            last_x = next_x_vals.back(); //last x point in previous trajectory
          }
          else {last_x = car_x;}
          double dist_to = distance(wp1[0], wp1[1], car_x, car_y); //distance to next reference
          double n_steps = dist_to/dist_step; //# steps to get to target at desired speed
          double x_step = (wp1[0]-car_x)/n_steps; // delta x for desired speed
          for (int i=0; i<50-points_left; i++){
            last_x += x_step;
            next_x_vals.push_back(last_x); //x value for next point
            //std::cout << "next_x: " << last_x << std::endl;
            next_y_vals.push_back(path(last_x)); //corresponding y value for next point
            //std::cout << "next_y: " << path(last_x) << std::endl;
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