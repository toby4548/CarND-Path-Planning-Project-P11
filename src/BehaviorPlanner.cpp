//
// Created by toby4548 on 8/5/18.
//

#include "BehaviorPlanner.h"
#include <iostream>

using namespace std;

void BehaviorPlanner::plan(SensorFusion &fusion) {
  if (fusion.other_car_front_) {
    if (!fusion.other_car_left_ && fusion.my_lane_ > 0) {
      fusion.my_lane_--;
    } else if (!fusion.other_car_right_ && fusion.my_lane_ != 2) {
      fusion.my_lane_++;
    } else {
      my_speed_ -= 0.5;
    }
  } else {
    if (fusion.my_lane_ != 1) {
      if ((fusion.my_lane_ == 0 && !fusion.other_car_right_) || (fusion.my_lane_ == 2 && !fusion.other_car_left_)) {
        fusion.my_lane_ = 1;
      }
    }
    if (my_speed_ < 49.5) {
      my_speed_ += 0.224;
    }
  }
}


