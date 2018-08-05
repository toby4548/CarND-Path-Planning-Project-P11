//
// Created by toby4548 on 8/5/18.
//

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s);

// return pi
constexpr double pi();

// convert deg to rad
double deg2rad(double x);

// convert rad to deg
double rad2deg(double x);

// return the distance of two points
double distance(double x1, double y1, double x2, double y2);

// Find the closest waypoint in the list
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

// Find the next waypoint to process
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);




#endif //PATH_PLANNING_HELPER_H
