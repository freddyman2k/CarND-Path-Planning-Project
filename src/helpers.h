#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

#define NUM_LANES 3

// for convenience
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// Transform from global x,y coordinates to car x,y coordinates (car at (0,0) with angle 0)
// x_ref, y_ref, angle_ref: the car's position and heading in the global coord. system
// x,y: Point (in global coordinates) to transform
vector<double> toCarCoord(double x_ref, double y_ref, double angle_ref, 
                          double x, double y) {
  double x_shift = x - x_ref;
  double y_shift = y - y_ref;

  double x_in_car = (x_shift*cos(0-angle_ref) - y_shift*sin(0-angle_ref));
  double y_in_car = (x_shift*sin(0-angle_ref) + y_shift*cos(0-angle_ref));

  return {x_in_car, y_in_car};
}

// Transform from car x,y coordinates to global x,y coordinates (car at (0,0) with angle 0)
// angle_ref: Car's angle in global coordinate system
vector<double> carToGlobalCoord(double angle_ref, double x_car, double y_car) {
  double x_global = (x_car*cos(angle_ref) - y_car*sin(angle_ref));
  double y_global = (x_car*sin(angle_ref) + y_car*cos(angle_ref));

  return {x_global, y_global};
}

// get the lane id from the (Frenet coordinate) d value
int getLane(double d) {
  if (d >= 0.0 && d <= 4.0) {
    return 0;
  } else if (d > 4.0 && d <= 8.0) {
    return 1;
  } else if (d > 8.0 && d <= 12.0) {
    return 2;
  } else return -1;
}

// get the neighboring lane ids
vector<int> getNextLanes(int lane) {
  vector<int> neighbor_lanes;
  if (lane >= 1) {
    neighbor_lanes.push_back(lane-1);
  }
  if (lane <= NUM_LANES - 2) {
    neighbor_lanes.push_back(lane+1);
  }
  return neighbor_lanes;
}

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
   
   double si            = start[0];
   double si_dot        = start[1];
   double si_double_dot = start[2];
   double sf            = end[0];
   double sf_dot        = end[1];
   double sf_double_dot = end[2];
   
   MatrixXd A = MatrixXd(3,3);
   MatrixXd B = MatrixXd(3,1);
   // pre-compute terms
   double T2 = T * T;
   double T3 = T2 * T;
   double T4 = T3 * T;
   double T5 = T4 * T;
   
   // define matrices
   A <<     T3,    T4,    T5,
          3*T2,  4*T3,  5*T4,
           6*T, 12*T2, 20*T3;
           
   B << sf - (si + si_dot * T + 0.5 * si_double_dot * T2),
                    sf_dot - (si_dot + si_double_dot * T),
                            sf_double_dot - si_double_dot;
   
   // calculate coefficients (alphas) 3 to 5   
   VectorXd C = A.inverse() * B;

   return {si, si_dot, 0.5 * si_double_dot, C.data()[0], C.data()[1], C.data()[2]};
}

// calculate the values of a quintic polynomial
// (y(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5) 
// given by the 6 coefficients in the vector a
// usually used on the output of JMT()
double quintic_polynomial(vector<double> a, double t) {
  double result = a[0];
  for (int i = 1; i < 6; ++i) {
    result += a[i];
    t *= t;
  }
  return result;
}

#endif  // HELPERS_H