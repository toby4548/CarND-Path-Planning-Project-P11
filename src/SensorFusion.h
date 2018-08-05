//
// Created by toby4548 on 8/5/18.
//

#ifndef PATH_PLANNING_SENSORFUSION_H
#define PATH_PLANNING_SENSORFUSION_H


class SensorFusion {
public:
  // making prediction
  void Prediction(int lane, double car_s,int prev_size, float d, double vx, double vy, double check_car_s, bool& other_car_front, bool& other_car_left, bool& other_car_right);
};


#endif //PATH_PLANNING_SENSORFUSION_H
