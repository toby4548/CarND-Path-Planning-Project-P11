//
// Created by toby4548 on 8/5/18.
//

#include "BehaviorPlanner.h"
#include <iostream>

using namespace std;

void BehaviorPlanner::plan(bool other_car_front,bool other_car_left,bool other_car_right, int& lane, double& ref_vel) {
  if (other_car_front) {
    if (!other_car_left && lane > 0) {
      lane--;
    } else if (!other_car_right && lane != 2) {
      lane++;
    } else {
      ref_vel -= 0.5;
    }
  } else {
    if (lane != 1) {
      if ((lane == 0 && !other_car_right) || (lane == 2 && !other_car_left)) {
        lane = 1;
      }
    }
    if (ref_vel < 49.5) {
      ref_vel += 0.224;
    }
  }
}


