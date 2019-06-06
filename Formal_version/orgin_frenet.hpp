#include "cal_frenet_candidates.hpp"
// #include "polynomials.hpp"
// #include <vector>
//#include "parameter_read.hpp"

/*
Detailed description of calculating path candidates based on original frenet paper

Input:
    current speed c_speed
    current lateral distance c_d 
    current lateral velocity c_d_d
    current lateral acceleration c_d_dd
    current position s0

Output:
    vector of lateral candidates
    vector of longitudinal candidates

*/


class initial_frenet{
    public:


	vector<frenet_optimal_path> lateral_cands(double, double, double, double , double );
	vector<frenet_optimal_path> longi_cands(double, double, double, double, double);
};