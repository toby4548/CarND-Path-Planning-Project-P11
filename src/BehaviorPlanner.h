//
// Created by toby4548 on 8/5/18.
//

#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H


#include "SensorFusion.h"

class BehaviorPlanner {
public:
  // planning vehicle behavior
  double my_speed_ = 0; //target speed

  void plan(SensorFusion &fusion);
};


#endif //PATH_PLANNING_BEHAVIORPLANNER_H
