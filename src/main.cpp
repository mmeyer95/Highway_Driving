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

double max_accel = 10; //  m/s^2         
double max_speed = 22.352; //maximum speed in m/s (50 MPH)
double desired_min = 17.8; //~40 mph
double speed_diff = max_accel*0.02*5;

double set_speed(double current_speed, double compare_speed) {
  //set speed: add, subtract, or stay the same
  double speed;
  if (compare_speed > current_speed) {
    speed = current_speed + speed_diff;}
  else if (compare_speed < current_speed){speed = current_speed - speed_diff;}
  else {speed = current_speed;}
  //do not go above 50 or too far below 40  
  if (speed>max_speed){speed=max_speed;}
  if (speed<desired_min){speed = desired_min;}
  std::cout << "Current speed = " << current_speed << ". Target speed: " << speed << std::endl;
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
  
//-----------------------------------------------------------------------------------------------------------------------          
          /////**BEHAVIOR PLANNING**/////
          //set up one-time defined variables
  		  int current_lane;	//the lane the car is currently in (0, 1, or 2)
  		  int target_lane; //the lane the car should aim to be in
		  double target_speed = max_speed;
          double current_speed = car_speed*0.447; //in m/s
          double turn_flag = 0; //keep track of whether a lane change is happening
          vector<double> lane_mults = {2,6,10}; //multipliers of d vector to be in the center of lanes 0, 1, 2 resp.
          
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
          vector<double> ahead_neighbor;
          vector<double> left_neighbor;
          vector<double> right_neighbor;
          double ahead_velocity = max_speed;
		  double right_velocity = max_speed;
		  double left_velocity = max_speed;
          
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
              car_ahead = car_distance>0 && car_distance<40;
              if (car_ahead==true){
                ahead_velocity = other_velocity;
              }
            }
            
            //Is there a car in the left lane to me
            else if (other_lane==current_lane-1 && car_left==false){
              car_left = car_distance>-5 && car_distance<40;
              double min_d_left = 20000;
              if (car_left==true && car_distance<min_d_left){
                left_velocity = other_velocity;
                min_d_left = car_distance;
              }
            }
            //Is there a car in the right lane to me
            else if (other_lane==current_lane+1 && car_right==false){
              car_right = car_distance>-5 && (car_distance)<40;
              double min_d_right = 20000;
              if (car_right==true && car_distance<min_d_right) {
                min_d_right = car_distance;
                right_velocity = other_velocity;
              }
            }
          }
          

//-----------------------------------------------------------
		  //Based on car proximities, pick a move
          bool move_flag = 0; //flag set tp high if changing lanes, used for velocity calc later
		  bool left_move = current_lane>0 && !car_left; //left move possible if 2 conditions met
          bool right_move = current_lane<2 && !car_right; //right move possible if 2 conditions met
          
          //if car ahead, but right move is possible
          if (car_ahead && right_move){
            target_lane = current_lane + 1;
            target_speed = set_speed(target_speed, max_speed);
            move_flag = 1;
          }
          //if car ahead, but left move is possible
          else if (car_ahead && left_move){
            target_lane = current_lane -1;
            target_speed = set_speed(target_speed, max_speed);
            move_flag = 1;
          }
          //other wise, stay in lane and match speed
          else {
            target_lane = current_lane;
            target_speed = set_speed(target_speed, ahead_velocity);
          }
  //****************************************************//
          //////**TRAJECTORY GENERATION**/////
          
		  //Vectors to sent to the simulator as next moves for the car
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //Collect values for spline
          vector<double> spline_x = {};
          vector<double> spline_y = {};
          
          //std::cout << "Points remaining: " << prev_len << std::endl;
          
          //Add un-processed points to path to send to simulator
          //int prev_used = 25;
          if (prev_len>0){
          	for (int i=0; i<prev_len;i++){
          		next_x_vals.push_back(previous_path_x[i]);
          		next_y_vals.push_back(previous_path_y[i]);
          	}
          }  
          
          //Use previous points for smoothness in spline calc
          if (prev_len<2) {
    	  	//Use 1 points straight behind and current position if <2 un-processed points
            //std::cout << "CAR YAW: " << car_yaw << std::endl;
    		double prev_car_x = car_x - cos((car_yaw));
    		double prev_car_y = car_y - sin((car_yaw));

    		spline_x.push_back(prev_car_x);
    		spline_x.push_back(car_x);

    		spline_y.push_back(prev_car_y);
    		spline_y.push_back(car_y);
		  }
          
		  else { 
            //add the points from last trajectory to points for spline calc
            //for (int i=0; i<points_left;i++){
              spline_x.push_back(previous_path_x[prev_len-2]);
              spline_x.push_back(previous_path_x[prev_len-1]);
          	  spline_y.push_back(previous_path_y[prev_len-2]);
              spline_y.push_back(previous_path_y[prev_len-1]);
            //}
          }
          //std::cout << "Checkpoint 2." <<std::endl;
          
          //Use points down the road to calculate spline
          //double mid_move_d = (lane_mults[target_lane] + car_d)/2;
          //std::cout << mid_move_d <<std::endl;
          //double ref_s = std::max(car_s, end_path_s);
          if (end_path_s==0){end_path_s=car_s;}
          vector<double> wp1 = getXY(end_path_s + 30, lane_mults[target_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp2 = getXY(end_path_s + 60, lane_mults[target_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp3 = getXY(end_path_s + 90, lane_mults[target_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);

          if (wp1[0]<spline_x.back()){wp1[0]=spline_x.back();} //avoid the spline error
          if (wp2[0]<wp1[0]){wp2[0]=wp1[0];}
          if (wp3[0]<wp2[0]){wp3[0]=wp2[0];}
          spline_x.push_back(wp1[0]);
          spline_x.push_back(wp2[0]);
          spline_x.push_back(wp3[0]);
          spline_y.push_back(wp1[1]);
          spline_y.push_back(wp2[1]);
          spline_y.push_back(wp3[1]);

          //std::cout << "Checkpoint 3." << std::endl;
          //check for rollover
            //for (int k=0; k<spline_x.size();k++){
              //std::cout << "Spline x: " << spline_x[k] << ".Spline y: " << spline_y[k] << std::endl;
            //}
     
          //std::cout << "Target speed: " << target_speed << std::endl;
          for (int k=0; k<spline_x.size(); k++){
            std::cout << "Spline: " << spline_x[k] << ". " << spline_y[k] << std::endl;
          }
          //std::sort(spline_x.begin(),spline_x.end());
          
          //Compute the spline and fill next x & y based on calculated positions at 0.02 s intervals
          tk::spline path;
          path.set_points(spline_x,spline_y);

          //Get 1 second of path data, including any un-processed points from last trajectory
          
          //Factor of safety for speed since distance calc is based on shorest path
          double safety;
          if (turn_flag) {safety=1.2;}
          else if (abs(car_yaw)>=45) {safety=1.2;}
          else {safety=1.15;}
          
		  //find delta x for values to be pushed back
          std::cout << "CAR YAW: " << car_yaw << std::endl;
          double time_step = 0.02; //seconds time step
		  double dist_step = target_speed*time_step/safety; //max change in distance allowed between each point
          double last_x; //last x position of car as calculated- either end of last traj, or current pos.
          if (prev_len !=0){
            last_x = next_x_vals.back(); //last x point in previous trajectory
          }
          else {last_x = car_x;}
          double dist_to = distance(wp1[0], wp1[1], car_x, car_y); //distance to next reference
          double n_steps = dist_to/dist_step; //# steps to get to target at desired speed
          double x_step = (wp1[0]-car_x)/n_steps; // delta x for desired speed
      	  //Push the x & y values to the vectors
          //int curr_size = std::min(prev_len,prev_used);
          for (int i=0; i<50-prev_len; i++){
			last_x += x_step;
            //std::cout << last_x << std::endl;
            next_x_vals.push_back(last_x); //x value for next point
            next_y_vals.push_back(path(last_x)); //corresponding y value for next point
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