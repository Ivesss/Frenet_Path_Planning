#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <algorithm>
#include <cfloat>
#include "polynomials.hpp"
#include "cubic_spline_planner.hpp"
	

// Parameter
double MAX_SPEED = 40.0/3.6; // maximum speed [m/s]
double MAX_ACCEL = 2.0;  // maximum acceleration [m/ss]
double MAX_CURVATURE = 1.0;  // maximum curvature [1/m]
double MAX_ROAD_WIDTH = 5.0;  // maximum road width [m]
double D_ROAD_W = 1.0;  // road width sampling length [m]
double DT = 0.25;  // time tick [s]
double MAXT = 6.0;  // max prediction time [m]
double MINT = 5.0;  // min prediction time [m]
double TARGET_SPEED = 30.0/3.6;  // target speed [m/s]
double D_T_S = 6.0/3.6;   // target speed sampling length [m/s]
double N_S_SAMPLE = 1;// sampling number of target speed
double ROBOT_RADIUS = 5.0;  // robot radius [m]
double minV = TARGET_SPEED - D_T_S*N_S_SAMPLE;
double maxV = TARGET_SPEED + D_T_S*N_S_SAMPLE;
double Tp = 4;  //Time segments per every path, unstable when > 15

// cost weights
double KJ = 0.1;
double KT = 0.1;
double KD = 1.0;
double KLAT = 1.0;
double KLON = 1.0;

// nav_msgs::Odometry odom;
// nav_msgs::OccupancyGrid cmap;
// geometry_msgs::PolygonStamped footprint;
vector<double> ob_x;
vector<double> ob_y;
#endif