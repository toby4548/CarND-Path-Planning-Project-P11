//
// Created by toby4548 on 8/9/18.
//

#include "TrajectoryGeneration.h"
#include <math.h>
#include "spline.h"

vector<vector<double>> TrajectoryGeneration::GenerateTrajectory(double car_x, double car_y, double car_s, double car_yaw, int prev_size, json previous_path_x, json previous_path_y, int lane, double speed, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y){
  //create xy list for spline
  vector<double> ptsx;
  vector<double> ptsy;

  //reference x,y, yaw state
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  //check previous availability
  if (prev_size<2) {
    //almost not enough
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  } else {
    //use previous path's end point as starting reference
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    //use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  //Add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // transform to car coordinate
  for (int i = 0; i < ptsx.size(); i ++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0-ref_yaw));
  }

  //create spline
  tk::spline s;
  //set x,y points to spline
  s.set_points(ptsx,ptsy);

  //output points
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  //start from previous path points
  for (int i = 0; i < prev_size; i ++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  //Calculate spline points
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

  double x_add_on = 0;

  //Fill up the rest of our path planner, always output 50 points
  for (int i = 1; i<50 - prev_size; i ++) {
    double N = (target_dist/(0.02*speed/2.24));
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    //rotate back to global coordinate
    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;


    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  vector<vector<double>> next_vals;
  next_vals.push_back(next_x_vals);
  next_vals.push_back(next_y_vals);

  return next_vals;

}

