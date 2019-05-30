#ifndef GLOBALCOORHPP
#define GLOBALCOORHPP


#include <vector>
#include "cal_frenet_candidates.hpp"

class global_cord{
    public:
    vector<frenet_path_candidates> origin_frenet(vector<frenet_path_candidates> &, Spline2D);

};

#endif