//
// Created by toby4548 on 8/5/18.
//

#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H


class BehaviorPlanner {
public:
  // planning vehicle behavior
  void plan(bool other_car_front,bool other_car_left,bool other_car_right, int& lane, double& ref_vel);

};


#endif //PATH_PLANNING_BEHAVIORPLANNER_H
