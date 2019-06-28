#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include "common.hpp"
#define PI 3.1415926 // pi

using namespace std;


class ConverCartesionFrenet{


    public:
        
        static void XY2SL(double x_c, double y_c, vector<double> rx, vector<double> ry, 
        vector<double> ryaw, ReferenceLine &referenceLine);

        static void frenet_to_cartesian(ReferencePoint refer, double s,double d,double &x,double &y);

        static void frenet_to_cartesian(ReferencePoint refer, double s, double ds, double dds,
        double l, double dl, double ddl, double &x,double &y, double &theta, double &kappa,
        double &v, double &a);

        // static void frenet_to_cartesian(const double rs, const double rx,
        //                                     const double ry, const double rtheta,
        //                                     const double rkappa, const double rdkappa,
        //                                     const std::array<double, 3>& s_condition,
        //                                     const std::array<double, 3>& d_condition,
        //                                     double* const ptr_x, double* const ptr_y,
        //                                     double* const ptr_theta,
        //                                     double* const ptr_kappa, double* const ptr_v,
        //                                     double* const ptr_a);

        static void cartesian_to_frenet(
            const double rs, //curve length
            const double rx, const double ry, const double rtheta,//reference s,x,y,heading...
            const double rkappa, const double rdkappa, //参考点的k, dk
            const double x, const double y,//global points waitting to be converted
            const double v, const double a, const double theta, const double kappa,//other information of that global point
            //输出  s 三阶  d 三阶
            std::array<double, 3>* const ptr_s_condition, //Output
            std::array<double, 3>* const ptr_d_condition);

        static double Distance(int xg, int yg, int xc, int yc);


        static TrajectoryPoint initial_Point_trans(ReferencePoint nearRefer, double x, double y,double angle, double v,
        double a, double kappa, std::array<double, 3> &ptr_s_condition, std::array<double, 3> &ptr_d_condition);
        // put samplePatWaypoints to DP.cpp file


};
#endif