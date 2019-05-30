#ifndef CAL_FRENET_CANDIDATES_HPP
#define	CAL_FRENET_CANDIDATES_HPP

#include "cubic_spline_planner.hpp"
#include "polynomials.hpp"
#include "parameters.hpp"
#include <vector>

using namespace std;
typedef vector<double> vecD;

//#include "frenet_optimal_trajectory.hpp"

//Header file of path candidates
class frenet_path_candidates
{

	public:

		vecD t, d, d_d, d_dd, d_ddd, s, s_d, s_dd, s_ddd, x, y, yaw, ds, c;
		double cd, cv, cf, Ti;

		class lateral_cands
		{
			public:
			//Initial (Vel, lat_d, lat_vel, lat_acc, longi_position)
			vector<frenet_path_candidates> origin_frenet(double, double, double, double, double);
			vector<frenet_path_candidates> method2(double, double, double, double, double);
			vector<frenet_path_candidates> method3(double, double, double, double, double);
			vector<frenet_path_candidates> method4(double, double, double, double, double);
		};

		class longi_cands
		{
			public:
			//Initial (Vel, lat_d, lat_vel, lat_acc, longi_position)
			vector<frenet_path_candidates> origin_frenet(double, double, double, double, double);
			vector<frenet_path_candidates> method2(double, double, double, double, double);
			vector<frenet_path_candidates> method3(double, double, double, double, double);
			vector<frenet_path_candidates> method4(double, double, double, double, double);

		};
}; // end of class
#endif