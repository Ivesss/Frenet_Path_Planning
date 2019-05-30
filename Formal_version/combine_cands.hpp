#ifndef COMBINE_CANDS_HPP
#define	COMBINE_CANDS_HPP


#include "cal_frenet_candidates.hpp"
//#include "parameters.hpp"

class combine{

    public:

        vector<frenet_path_candidates> origin_frenet(vector<frenet_path_candidates>, vector<frenet_path_candidates>);
        vector<frenet_path_candidates> method2(vector<frenet_path_candidates>, vector<frenet_path_candidates>);
        vector<frenet_path_candidates> method3(vector<frenet_path_candidates>, vector<frenet_path_candidates>);
        vector<frenet_path_candidates> method4(vector<frenet_path_candidates>, vector<frenet_path_candidates>);
};

#endif