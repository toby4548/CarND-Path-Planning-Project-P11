//
// Created by toby4548 on 8/7/18.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

// Use for save the information of the cars and can be used to generate predictions through
// the saved information
// Data type from the simulator : [ id, x, y, vx, vy, s, d]
class Car {
public:
  int id_;                        // car id
  double x_;                      // position x [m]
  double y_;                      // position y [m]
  double vx_;                     // x velocity [m/s]
  double vy_;                     // y velocity [m/s]
  double s_;                      // s in frenet
  double d_;                      // d in frenet

};


#endif //PATH_PLANNING_CAR_H
