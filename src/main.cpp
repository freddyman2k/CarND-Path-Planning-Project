#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <vector>
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#define V_GOAL 49.5

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

  int lane = 1;

  double v_target = 0.0;

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
               &map_waypoints_dx,&map_waypoints_dy,&lane,&v_target]
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

          /**
           * PATH PLANNING : The path (made up by (x,y) points) 
           * defined below is sequentially visited every .02 seconds by the car.
           */

          double pos_x;
          double pos_y;
          double pos_s;
          double pos_d;
          vector<double> pos_frenet;
          double pos_angle;
          int path_size = previous_path_x.size();

          // The x and y vectors of points the path spline will be calculated from
          vector<double> spline_points_x;
          vector<double> spline_points_y;

          // Depending on whether there was a previously unprocessed path, we need 
          // to define the x, y and angle values we will work with in the next step
          if (path_size < 2) {
            // If there was no previous path, the values are initialized with
            // the car's localization data
            pos_x = car_x;
            pos_y = car_y;
            pos_s = car_s;
            pos_d = car_d;
            pos_angle = deg2rad(car_yaw);

            // Also, initalize the path with 2 "fake" points
            // creating a path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            spline_points_x.push_back(prev_car_x);
            spline_points_x.push_back(car_x);

            spline_points_y.push_back(prev_car_y);
            spline_points_y.push_back(car_y);
          } else {
            // If there was indeed a path that can be continued, initialize
            // the values with the last values on that path
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];

            double pos_x_prev = previous_path_x[path_size-2];
            double pos_y_prev = previous_path_y[path_size-2];
            pos_angle = atan2(pos_y-pos_y_prev,pos_x-pos_x_prev);

            // Make the path tangent to the previous path's end points
            spline_points_x.push_back(pos_x_prev);
            spline_points_x.push_back(pos_x);

            spline_points_y.push_back(pos_y_prev);
            spline_points_y.push_back(pos_y);

            pos_frenet = getFrenet(pos_x, pos_y, pos_angle, map_waypoints_x, map_waypoints_y);
            pos_s = pos_frenet[0];
            pos_d = pos_frenet[1];
          }

          // check if the car will collide with another car
          int blocked_lanes[] = {0, 0, 0};
          int current_lane = getLane(pos_d);
          double front_car_speed_mph;
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            auto other_car = sensor_fusion[i];
            double other_car_x = other_car[1];
            double other_car_y = other_car[2];
            double other_car_vx = other_car[3];
            double other_car_vy = other_car[4];
            double other_car_s = other_car[5];
            double other_car_d = other_car[6];

            double other_car_speed_in_m_per_s = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
            other_car_s += other_car_speed_in_m_per_s * 0.02 * (double)path_size;
            if ((other_car_s > pos_s - 10.0)  && (abs(other_car_s - pos_s) < 30.0)) {
              int blocked_lane = getLane(other_car_d);
              blocked_lanes[blocked_lane] = 1;
              if (blocked_lane == current_lane) {
                front_car_speed_mph = other_car_speed_in_m_per_s * 2.237; // convert to mph
              }
            }
          }

          int prev_lane = lane;
          bool lane_change = false;
          if (blocked_lanes[current_lane] == 1) {
            // Can only collide if the other car is driving in the same lane
            // Either change lanes or adapt speed
            vector<int> neighbor_lanes = getNextLanes(current_lane);
            for (int i = 0; i < neighbor_lanes.size(); ++i) {
              int next_lane = neighbor_lanes[i];
              if (blocked_lanes[next_lane] == 0) {
                lane_change = true;
                lane = next_lane;

                // Break loop, otherwise there could be multiple lane changes
                break;
              }
            }
          }

          // The spline should also include some future points.
          // Here, we use evenly spaced (30m apart) points along the road
          // Add an additional midpoint to the spline if the car is supposed 
          // to change lanes, this will create a smoother trajectory
          if (lane_change) {
            double d_mid_lane_change = 2 + 4 * (lane - 0.5 * (lane - prev_lane));
            vector<double> waypoint_pos_xy = getXY((pos_s + 30.0), d_mid_lane_change, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            spline_points_x.push_back(waypoint_pos_xy[0]);
            spline_points_y.push_back(waypoint_pos_xy[1]);
          } else {
            vector<double> waypoint_pos_xy = getXY((pos_s + 30.0), (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            spline_points_x.push_back(waypoint_pos_xy[0]);
            spline_points_y.push_back(waypoint_pos_xy[1]);
          }
          for (int i = 2; i < 4; ++i) {
            vector<double> waypoint_pos_xy = getXY((pos_s + 30.0*i), (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            spline_points_x.push_back(waypoint_pos_xy[0]);
            spline_points_y.push_back(waypoint_pos_xy[1]);
          }

          // Convert the x and y values of the waypoints into the car coordinates
          // (car at (0,0)). This makes the interpolation easier in a future step.
          for (int i = 0; i < spline_points_x.size(); ++i) {
            vector<double> xy_in_car_coord = toCarCoord(pos_x, pos_y, 
                                                        pos_angle, 
                                                        spline_points_x[i], 
                                                        spline_points_y[i]);

            spline_points_x[i] = xy_in_car_coord[0];
            spline_points_y[i] = xy_in_car_coord[1];
          }

          // Set up the spline function with the previously added points
          // (the old path points and the calculated future positions)
          tk::spline s;
          s.set_points(spline_points_x, spline_points_y);     

          // If there are unprocessed / old path points, we will use these 
          // to begin the next planned path. This will yield a smooth transition.

          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Use the spline to interpolate points: 
          // The points should be spaced in such a way that the car's velocity
          // matches the target velocity
          double target_x = 30.0; // Our path will "look ahead" in this distance
          double target_y = s(target_x);
          double target_dist = distance(target_x, target_y, 0.0, 0.0);
          double x_len = target_x;
          double N = target_dist / (0.02 * v_target/2.237);
          double x_point = 0.0;
          double y_point = 0.0;
          for (int i = 1; i <= 50-path_size; ++i) {
            if (blocked_lanes[current_lane] == 1 && !lane_change && v_target > front_car_speed_mph) {
              // Decelerate to the speed of the car in front if all lanes are blocked
              v_target -= .224;
              // Recalculate N for the new distance and velocity
              target_dist = distance(target_x, target_y, x_point, y_point);
              N = target_dist / (0.02 * v_target/2.237);
              x_len = target_x - x_point;
            } else if (v_target < V_GOAL) {
              // Accelerate to drive at the speed limit
              v_target += .224;
              // Recalculate N for the new distance and velocity
              target_dist = distance(target_x, target_y, x_point, y_point);
              N = target_dist / (0.02 * v_target/2.237);
              x_len = target_x - x_point;
            }

            x_point += x_len/N;
            y_point = s(x_point);

            // Convert / rotate the interpolated points back to global coordinates
            vector<double> xy_points_in_global_coord = carToGlobalCoord(pos_angle, x_point, y_point);
            next_x_vals.push_back(pos_x + xy_points_in_global_coord[0]);
            next_y_vals.push_back(pos_y + xy_points_in_global_coord[1]);
          }     

          // Pass the path's x and y points to the json object
          // so that they can be sent to the simulator
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