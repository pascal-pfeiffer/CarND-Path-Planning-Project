#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
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

  // set starting lane (1)
  int lane = 1;
  
  // define a starting reference velocity
  double ref_vel = 0;  // mph
  
  h.onMessage([&ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &lane]
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

          // used code and ideas from aaron from the lecture here
          // store remaining previous path size in variable
          int prev_path_size = previous_path_x.size();
          
          if (prev_path_size > 0){
            car_s = end_path_s;
          }
          
          bool too_close = false;
          bool check_for_lanechange = false;
          // bool lane_change_left_possible = false;
          // bool lane_change_right_possible = false;
          double speed_ahead = 99;
          // double cost_lane0 = 99999;
          // double cost_lane1 = 99999;
          // double cost_lane2 = 99999;
          
          // This is the emergency slow down if a vehicle in front of us is coming to close
          // find a reference velocity to use (car in front)
          for (int i = 0; i < sensor_fusion.size(); i++){
            // car is in my lane
            float d = sensor_fusion[i][6];
            if (d < (2+4*lane+2) && d > (2+4*lane-2)){
              // std::cout << "Checking a car in our lane!" << std::endl;
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              // we project s in the next timestep
              check_car_s += ((double)prev_path_size * 0.02 * check_speed);
              
              // is s value of other car greater than ego s value?
              // is the gap less than 50 m, we should look for alternatives
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 50)){
                // check for possible lane changes
                // std::cout << "check for possible lane changes" << std::endl;
                check_for_lanechange = true;
                
                // if nothing else works and we are already close to the front vehicle
                // set flag for us being to close and to slow down
                if ((check_car_s > car_s) && ((check_car_s - car_s) < 20)){
                  // std::cout << "Collision warning, reduce speed!" << std::endl;
                  too_close = true;
                  // std::cout << "Speed ahead" << check_speed << std::endl;
                  if (check_speed < speed_ahead){
                    speed_ahead = check_speed;
                  }
                }
              }
            }
          }
          
          // too close and slower than our ref_vel (in m/s)
          if (too_close and (speed_ahead < (ref_vel/2.24))){
            ref_vel -= 1.00;
          }
          else{
            if (ref_vel < 49.5){
              ref_vel += 0.50;
            }
          }
          
          // we should change lanes to get forward faster
          if (check_for_lanechange){
            
            // we are in the middle lane
            if (lane == 1){
              double range_behind_left = 99999;
              double range_behind_right = 99999;
              double range_ahead_left = 99999;
              double range_ahead_right = 99999;
              
              // find vehicles in the adjacent lanes (0, 2)
              for (int i = 0; i < sensor_fusion.size(); i++){
                // car is in in lane 0 (left)
                float d = sensor_fusion[i][6];
                if (d < (2+4*(lane-1)+2) && d > (2+4*(lane-1)-2)){
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx*vx+vy*vy);
                  double check_car_s = sensor_fusion[i][5];

                  // we project s in the next timestep
                  check_car_s += ((double)prev_path_size * 0.02 * check_speed);
                  
                  // we found a nearest car ahead
                  if ((check_car_s > car_s) && (check_car_s - car_s) < range_ahead_left){
                    range_ahead_left = check_car_s - car_s;
                  }
                  
                  // we found a nearest car behind
                  if ((check_car_s < car_s) && (car_s - check_car_s) < range_behind_left){
                    range_behind_left = car_s - check_car_s;
                  }
                }
                if (d < (2+4*(lane+1)+2) && d > (2+4*(lane+1)-2)){
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx*vx+vy*vy);
                  double check_car_s = sensor_fusion[i][5];

                  // we project s in the next timestep
                  check_car_s += ((double)prev_path_size * 0.02 * check_speed);
                  
                  // we found a nearest car ahead
                  if ((check_car_s > car_s) && (check_car_s - car_s) < range_ahead_right){
                    range_ahead_right = check_car_s - car_s;
                  }
                  
                  // we found a nearest car behind
                  if ((check_car_s < car_s) && (car_s - check_car_s) < range_behind_right){
                    range_behind_right = car_s - check_car_s;
                  }
                }
              }
              // we gain more space ahead and we have plenty of room for a safe lanechange
              // 55 m ahead, 8 m behind
              if ((range_ahead_right > 55) && (range_behind_right > 8)){
                lane = 2;
              }
              if ((range_ahead_left > 55) && (range_behind_left > 8)){
                lane = 0;
              }
            }
            
            // we are leftmost or rightmost and can only go to mid
            if ((lane == 0) or (lane == 2)){
              double range_behind = 99999;
              double range_ahead = 99999;
              
              // find vehicles in the adjacent lane (1)
              for (int i = 0; i < sensor_fusion.size(); i++){
                // car is in in the adjacent lane (1)
                float d = sensor_fusion[i][6];
                if (d < (2+4*(lane+1)+2) && d > (2+4*(lane+1)-2)){
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx*vx+vy*vy);
                  double check_car_s = sensor_fusion[i][5];

                  // we project s in the next timestep
                  check_car_s += ((double)prev_path_size * 0.02 * check_speed);
                  
                  // we found a nearest car ahead
                  if ((check_car_s > car_s) && (check_car_s - car_s) < range_ahead){
                    range_ahead = check_car_s - car_s;
                  }
                  
                  // we found a nearest car behind
                  if ((check_car_s < car_s) && (car_s - check_car_s) < range_behind){
                    range_behind = car_s - check_car_s;
                  }
                }
              }
              // we gain more space ahead and we have plenty of room for a safe lanechange
              // 55 m ahead, 8 m behind
              if ((range_ahead > 55) && (range_behind > 8)){
                    lane = 1;
              }
            }
          }
          
          vector<double> pts_x;
          vector<double> pts_y;
          
          // ego state at the last point of the previous calculated path
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // if previous path size is almost empty, use the current cars pose as a starting point
          if(prev_path_size < 2){
            std::cout << "Previous path size is less than 2!" << std::endl;
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            pts_x.push_back(prev_car_x);
            pts_x.push_back(car_x);
            
            pts_y.push_back(prev_car_y);
            pts_y.push_back(car_y);
            
            // std::cout << "car_yaw: " << car_yaw << ", " << cos(car_yaw) << ", " << sin(car_yaw) << std::endl;
            // std::cout << "prev_car x, y: " << prev_car_x << ", " << prev_car_y << std::endl;
            // std::cout << "car x, y: " << car_x << ", " << car_y << std::endl;
          }
          // use the previous path's end point as starting reference
          else{
            // std::cout << "Previous path size is larger than 2!" << std::endl;
            // redefine reference state as previous path end points
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];
            
            // also save the second to last path point to calculate the tangent
            double ref_x_prev = previous_path_x[prev_path_size - 2];
            double ref_y_prev = previous_path_y[prev_path_size - 2];
            
            // calculate the yaw using the last two point -> redefine
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            // use the two points to make the path tangent to the previous path's end point
            pts_x.push_back(ref_x_prev);
            pts_x.push_back(ref_x);
            
            pts_y.push_back(ref_y_prev);
            pts_y.push_back(ref_y);
          }
          
          // std::cout << pts_x.size() << ", " << pts_y.size() << std::endl;
          
          // get 40 m spaced frenet coordinates ahead of current ego state
          vector<double> next_wp0 = getXY(car_s + 40, (2+(4*lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 80, (2+(4*lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 120, (2+(4*lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          // std::cout << "WP0: " << next_wp0[0] << ", " << next_wp0[1] << std::endl;
          // std::cout << "WP1: " << next_wp1[0] << ", " << next_wp1[1] << std::endl;
          // std::cout << "WP2: " << next_wp2[0] << ", " << next_wp2[1] << std::endl;
          
          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);
          
          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);
          
          // std::cout << pts_x.size() << ", " << pts_y.size() << std::endl;
          
          for (int i = 0; i < pts_x.size(); i++){
            // shift and rotate the car to 0/0 with zero yaw
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;
            pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }
          
          // for (int i = 0; i < pts_x.size(); i++){
          //   std::cout << pts_x[i] << ", " << pts_y[i] << std::endl;
          // }
          
          // init a spline
          tk::spline s;
          
          // feed the support points to the spline
          // std::cout << pts_x.size() << ", " << pts_y.size() << std::endl;
          s.set_points(pts_x, pts_y);
          
          // define the actual (x, y) coordinates for the path planer
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // push all previous path points from the last calculation timestep into the new trajectory
          for (int i = 0; i < prev_path_size; i++){
            // std::cout << "Adding previous path point: " << previous_path_x[i] << ", " << previous_path_y[i] << std::endl;
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // calculate the spacing between path points to get our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y));
          // std::cout << "target_x: " << target_x << std::endl;
          // std::cout << "target_y: " << target_y << std::endl;
          // std::cout << "target_dist: " << target_dist << std::endl;
          
          // we start at origin
          double x_add_on = 0;
          // fill the remaining points of out path planner with points interpolated from the spline
          // will always give 50 points total
          // start at i = 1
          for (int i = 1; i <= 50 - prev_path_size; i++){
            // how many points are needed for the reference velocity
            // 2.24 to get from mph to meters per second
            // 0.02 as this is the tick rate
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x / N);
            double y_point = s(x_point);
            // std::cout << "N: " << N << std::endl;
            // std::cout << "x_point: " << x_point << std::endl;
            // std::cout << "y_point: " << y_point << std::endl;
            
            // std::cout << "step " << i << " " << x_point << ", " << y_point << std::endl;
            
            x_add_on = x_point;  // new start point
            
            // need to get back to global coordinates
            double x_ref = x_point;
            double y_ref = y_point;
            
            // rotate back to normal
            // std::cout << "ref_yaw: " << ref_yaw << std::endl;
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            // std::cout << "step " << i << " " << x_point << ", " << y_point << std::endl;
            
            // push point to trajectory
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // for (int i = 0; i < next_x_vals.size(); i++){
          //   std::cout << next_x_vals[i] << ", " << next_y_vals[i] << std::endl;
          // }
          
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