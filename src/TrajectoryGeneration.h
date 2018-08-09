//
// Created by toby4548 on 8/9/18.
//

#ifndef PATH_PLANNING_TRAJECTORYGENERATION_H
#define PATH_PLANNING_TRAJECTORYGENERATION_H

#include <iostream>
#include "helper.h"
#include "json.hpp"

using json = nlohmann::json;

class TrajectoryGeneration {
public:
  vector<vector<double>> GenerateTrajectory(double car_x, double car_y, double car_s, double car_yaw, int prev_size, json previous_path_x, json previous_path_y, int lane, double speed, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);
};


#endif //PATH_PLANNING_TRAJECTORYGENERATION_H
