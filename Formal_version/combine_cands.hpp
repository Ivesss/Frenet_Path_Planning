#ifndef COMBINE_CANDS_HPP
#define	COMBINE_CANDS_HPP


#include "cal_frenet_candidates.hpp"

/*

Input:
      lateral and longitudinal candidates (two vectors)

Output:
      combined trajectory candidates (one vector)


*/
class combine{

    public:

        vector<frenet_optimal_path> origin_frenet(vector<frenet_optimal_path>, vector<frenet_optimal_path>);
        vector<frenet_optimal_path> method2(vector<frenet_optimal_path>, vector<frenet_optimal_path>);
        vector<frenet_optimal_path> method3(vector<frenet_optimal_path>, vector<frenet_optimal_path>);
        vector<frenet_optimal_path> method4(vector<frenet_optimal_path>, vector<frenet_optimal_path>);
};

#endif