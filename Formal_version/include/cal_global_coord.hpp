#ifndef GLOBALCOORHPP
#define GLOBALCOORHPP

/*
Calculate global coordingate

Input
    path candidates containning information of: s, s_d, s_dd, d, d_d, d_ddd, costs, jerks........

Output
    coordinate in the global frame based on lateral and longitudinal information.

*/


#include "cal_frenet_candidates.hpp"

class global_cord{
    public:
    vector<frenet_optimal_path> origin_frenet(vector<frenet_optimal_path> &, Spline2D);

};

#endif