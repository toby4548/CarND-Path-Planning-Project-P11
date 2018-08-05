//
// Created by toby4548 on 8/5/18.
//

#include "SensorFusion.h"
#include <iostream>
#include <math.h>

using namespace std;

void SensorFusion::Prediction(int lane, double car_s,int prev_size, float d, double vx, double vy, double check_car_s, bool& other_car_front, bool& other_car_left, bool& other_car_right)
{
  int other_car_lane = -1;

  if (d > 0 && d<=4) {
    other_car_lane = 0;
  } else if (d > 4 && d <=8) {
    other_car_lane = 1;
  } else if (d > 8 && d <= 12) {
    other_car_lane = 2;
  } else {

  }

  //predict other cars position using car speed
  double check_speed = sqrt(vx*vx+vy*vy);
  check_car_s += ((double)prev_size*0.02*check_speed);

  if (other_car_lane == lane) {
    //other car is in the same lane
    other_car_front |= check_car_s > car_s && check_car_s < 30 + car_s;
  } else if (other_car_lane == lane - 1) {
    //other car is in left lane
    other_car_left |= check_car_s > car_s - 30 && check_car_s < car_s + 30;
  } else if (other_car_lane == lane + 1) {
    //other car is in right lane
    other_car_right |= check_car_s > car_s - 30 && check_car_s < car_s + 30;
  } else {

  }
}