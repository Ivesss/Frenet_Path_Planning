#ifndef BESTPATHHPP
#define BESTPATHHPP
#include "cal_frenet_candidates.hpp"

/*
Chose best path according to different method.

Input
    path candidates of each algorithm

Output
    chose one best optimal path among all candidates according to different method

*/



class bestPath{

    public:
    frenet_optimal_path origin_frenet(vector<frenet_optimal_path>);
    frenet_optimal_path method2(vector<frenet_optimal_path>);
    frenet_optimal_path method3(vector<frenet_optimal_path>);
    frenet_optimal_path method4(vector<frenet_optimal_path>);

};

#endif