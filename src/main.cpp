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

//Initialize variables once
double max_accel = 10; //  m/s^2         
double max_speed = 22.352; //maximum speed in m/s (50 MPH)
double desired_min = 17.8; //~40 mph
double speed_diff = max_accel*0.02;
double target_speed = 0; //initialize target speed to 0
double time_step = 0.02; //seconds time step

//Initialize vectors once
vector<double> ref_vels = {max_speed, max_speed, max_speed};	//keep track of velocities of travel of each lane
vector<double> ref_dists = {100, 100, 100};		//keep track of distance of each lane
vector<double> lane_mults = {2,6,10}; //multipliers of d vector to be in the center of lanes 0, 1, 2 resp.
vector<int> behaviors = {0}; //keep track of prior moves

//Define function set_speed
double set_speed(double current_speed, double compare_speed, int prev_len) {
  //set speed: add, subtract, or stay the same
  double speed;
  double speed_diff_factor = 40;
  if (prev_len>25){speed_diff_factor=50-prev_len;}
  if (compare_speed > current_speed) {
    speed = current_speed + speed_diff*speed_diff_factor;}
  else if (compare_speed < current_speed){speed = current_speed - speed_diff*speed_diff_factor;}
  else {speed = current_speed;}
  //do not go above 50  
  if (speed>max_speed){speed=max_speed;}
  return speed;
}

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
  

//------------------------------------------------------------------------------------------------------------------   
          /////**BEHAVIOR PLANNING**/////
          //set up variables for this cycle
  		  int current_lane;	//the lane the car is currently in (0, 1, or 2)
  		  int target_lane; //the lane the car should aim to be in
          double current_speed = car_speed*0.447; //in m/s
          double turn_flag = 0; //keep track of whether a lane change is happening
          
          //Figure out what lane the car is in based on the current d value
		  if (car_d>=0 && car_d<4){current_lane=0;}
          if (car_d>=4 && car_d<8){current_lane=1;}
          if (car_d>=8 && car_d<12){current_lane=2;}
          
          //Count number of points left
          int prev_len = previous_path_x.size();
          
          //Find the nearby cars and determine what their speeds & relative locations are
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;
          
          //Check each car found with sensor fusion
          for (int i=0; i<sensor_fusion.size();i++){     
            
            vector<double> other_car = sensor_fusion[i];
            
            //Find lane the other car is in
            int other_lane=-1;
            if (other_car[6]>=0 && other_car[6]<4){other_lane=0;}
          	if (other_car[6]>=4 && other_car[6]<8){other_lane=1;}
          	if (other_car[6]>=8 && other_car[6]<12){other_lane=2;}
            
            //Calculate the distance to the sensed car and its velocity
            double other_velocity = sqrt(other_car[3]*other_car[3]+other_car[4]*other_car[4]);
            double car_distance = (other_car[5]+other_velocity*0.02*prev_len)-(car_s+current_speed*0.02*prev_len);

			//Is there a car in the same lane
            if (other_lane==current_lane && car_ahead==false){
              car_ahead = car_distance>0 && car_distance<30;
              if (car_ahead==true){
                ref_vels[current_lane] = other_velocity;
                ref_dists[current_lane] = car_distance;
              }
            }
            
            //Is there a car in the left lane to me
            else if (other_lane==current_lane-1 && car_left==false){
              car_left = car_distance>-8 && car_distance<30;
              double min_d_left = 20000;
              if (car_left==true && car_distance<min_d_left){
                ref_vels[current_lane-1] = other_velocity;
                ref_dists[current_lane-1] = car_distance;
                min_d_left = car_distance;
              }
            }
            //Is there a car in the right lane to me
            else if (other_lane==current_lane+1 && car_right==false){
              car_right = car_distance>-8 && (car_distance)<30;
              double min_d_right = 20000;
              if (car_right==true && car_distance<min_d_right) {
                min_d_right = car_distance;
                ref_vels[current_lane+1] = other_velocity;
                ref_dists[current_lane+1] = car_distance;
              }
            }
          }
          
          //Update reference velocities vector based on sensor fusion data
          if(!car_ahead){ref_vels[current_lane]=max_speed;}
          if(!car_left && current_lane>0){ref_vels[current_lane-1]=max_speed;}
          if(!car_right && current_lane<2){ref_vels[current_lane+1]=max_speed;}

//-------------------------------------------------------------------
		  //Based on car proximities, pick a move
		  bool left_move = current_lane>0 && (!car_left || ((ref_vels[current_lane-1] > ref_vels[current_lane]+4) && ref_dists[current_lane-1]>20)) && behaviors.back()!=1; //left move possible if conditions met
          bool right_move = current_lane<2 && (!car_right || ((ref_vels[current_lane+1] > ref_vels[current_lane]+4) && ref_dists[current_lane+1]>20)) && behaviors.back()!=-1; //right move possible if conditions met
          
          //If I'm in the middle lane with a car ahead and I can move either left or right, pick the move either without a car ahead(priority), or the one moving faster (secondary)
          if (car_ahead && left_move && right_move){
            //If no car on the right, but car on the left-- don't go left
            if (!car_right && car_left){left_move = 0;}
            //If no car on the left, but car on the right-- don't go right
            else if (!car_left && car_right){right_move = 0;}
            //If car on both sides out of movement range, choose lane going fastest
            else if (ref_vels[current_lane-1]>ref_vels[current_lane+1]){right_move=0;}
            else if (ref_vels[current_lane-1]<ref_vels[current_lane+1]){left_move=0;}
          }
          
  		  //If there is a car ahead and I can move right, do that
          if (car_ahead && right_move){
            target_lane = current_lane + 1;
            target_speed = set_speed(target_speed, ref_vels[target_lane], prev_len);
            turn_flag = 1;
            behaviors.push_back(1);
          }
          //Move left if a car is ahead and I can do so
          else if (car_ahead && left_move){
            target_lane = current_lane -1;
            target_speed = set_speed(target_speed, ref_vels[target_lane], prev_len);
            turn_flag = 1;
            behaviors.push_back(-1);
          }
          //If movement isn't possible, or no car is ahead, set speed based on either speed limit or car ahead
          else {
            target_lane = current_lane;
            target_speed = set_speed(target_speed, ref_vels[current_lane], prev_len);
            behaviors.push_back(0);
          }
                                              
  //****************************************************//
          //////**TRAJECTORY GENERATION**/////
          
		  //Vectors to sent to the simulator as next moves for the car
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //Values for spline calc
          vector<double> spline_x = {};
          vector<double> spline_y = {};
          vector<double> spline_time = {};
          
          //Add un-processed points to path to send to simulator
          if (prev_len>0){
          	for (int i=0; i<prev_len;i++){
          		next_x_vals.push_back(previous_path_x[i]);
          		next_y_vals.push_back(previous_path_y[i]);
          	}
          }   
          
          //Use previous points for smoothness in spline calc
          /////When the length is <2, this is **probably** the first sample
          if (prev_len<2) {
			vector<double> previous = getXY(car_s - 30, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            spline_x.push_back(previous[0]);
    		spline_x.push_back(car_x);
            
            spline_y.push_back(previous[1]);
    		spline_y.push_back(car_y);
            
            spline_time.push_back(-30/target_speed);
            spline_time.push_back(0);
		  }
          
		  else { 
            //add the points from last trajectory to points for spline calc if at least 2 points
              spline_x.push_back(previous_path_x[prev_len-2]);
              spline_x.push_back(previous_path_x[prev_len-1]);
          	  spline_y.push_back(previous_path_y[prev_len-2]);
              spline_y.push_back(previous_path_y[prev_len-1]);
              spline_time.push_back(-1*time_step);
              spline_time.push_back(0);
          }

          
          //Determine velocity safety factor based on movement
          double safety;
          if (turn_flag) {safety=1.2;}
          else {safety=1.1;} //about 45 MPH
          
          //Find s to base future waypoints on
          double s;
          if (prev_len>0) {s = end_path_s;}
          else {s = car_s;}

          //Find XY values of points down the road, calculate time, and fill spline vectors
          for (int i=1; i<4; i++){
            vector<double> wp = getXY(s + 30*i, lane_mults[target_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            double dist_to = distance (wp[0], wp[1], spline_x.back(), spline_y.back()); //dist from prev pt in m
            double time_point;
            if (target_speed<10) {time_point = spline_time.back() + safety*dist_to;}
            else {time_point = spline_time.back() + safety*dist_to/target_speed;}
            spline_time.push_back(time_point);
            spline_x.push_back(wp[0]);
            spline_y.push_back(wp[1]);  
          }
          
          //Compute the splines for x & y
          tk::spline path_x;
          tk::spline path_y;
          path_x.set_points(spline_time,spline_x);
          path_y.set_points(spline_time,spline_y);
          
          //Finish 1 second of path data, including any un-processed points from last trajectory
          for (int i=1; i<51-prev_len; i++){
            double next_x = path_x(i*0.02);
            double next_y = path_y(i*0.02);
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }
          
//--------------------------------------------------------------------------------------------------------------------------          
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