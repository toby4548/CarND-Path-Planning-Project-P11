//
// Created by toby4548 on 8/5/18.
//

#ifndef PATH_PLANNING_SENSORFUSION_H
#define PATH_PLANNING_SENSORFUSION_H

#include <iostream>
#include <vector>
#include <map>
#include "Car.h"

// The sensor fusion class contains all the detected cars
class SensorFusion {
public:
  int my_lane_ = 1; //start at lane = 1

  bool too_close_ = false;

  //check car driving around
  bool other_car_front_ = false;
  bool other_car_left_ = false;
  bool other_car_right_ = false;

  std::map<int, Car *> car_map_; // the map is used to save the pointers to Car detected


  // making prediction
  void Prediction(double car_s,int prev_size, float d, double vx, double vy, double check_car_s);


};


#endif //PATH_PLANNING_SENSORFUSION_H
