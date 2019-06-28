#ifndef CAL_FRENET_CANDIDATES_HPP
#define	CAL_FRENET_CANDIDATES_HPP

#include "cubic_spline_planner.hpp"
#include <vector>
//@@@@@@@@@@
// #include "polynomials.hpp"
// #include "combine_cands.hpp"
// #include "cal_global_coord.hpp"
// #include "best_path.hpp"
// #include "collision_checking.hpp"
// #include "obstacles.hpp"
// #include "../matplotlibcpp.h"
// #include "parameter_read.hpp"


using namespace std;
typedef vector<double> vecD;

/*
Input:
       Any variables that every algorithms need.


Output:
       Single optimal path for each selected algorithms



*/

class frenet_optimal_path
{

	public:

		vecD t, d, d_d, d_dd, d_ddd, s, s_d, s_dd, s_ddd, x, y, yaw, ds, c;
		double cd, cv, cf, Ti;
		

		frenet_optimal_path origin_frenet(double, double, double, double, double, Spline2D); //Output 1 single optimal path.
		frenet_optimal_path BERTHA();
		frenet_optimal_path baidu_apollo();
		frenet_optimal_path someothermethods(); // function variable could be anything


}; // end of class


		











#endif