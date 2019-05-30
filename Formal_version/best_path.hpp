#ifndef BESTPATHHPP
#define BESTPATHHPP
#include "cal_frenet_candidates.hpp"


class bestPath{

    public:
    frenet_path_candidates origin_frenet(vector<frenet_path_candidates>);
    frenet_path_candidates method2(vector<frenet_path_candidates>);
    frenet_path_candidates method3(vector<frenet_path_candidates>);
    frenet_path_candidates method4(vector<frenet_path_candidates>);

};

#endif