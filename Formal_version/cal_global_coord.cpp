#include "cal_global_coord.hpp"

/*
Calculate global coordingate

Input
    path candidates containning information of: s, s_d, s_dd, d, d_d, d_ddd, costs, jerks........

Output
    coordinate in the global frame based on lateral and longitudinal information.

*/

std::vector<frenet_optimal_path> global_cord::origin_frenet(vector<frenet_optimal_path> &fplist, Spline2D csp)
{
	for(auto& fp : fplist)
	{

		for(int i = 0; i < fp.s.size(); i++)
		{
			double ix, iy;
			csp.calc_position(&ix, &iy, fp.s[i]);

			if(ix == NONE)
				break;
			double iyaw = csp.calc_yaw(fp.s[i]);
			double di = fp.d[i];

			double fx = ix - di*sin(iyaw);
			double fy = iy + di*cos(iyaw);

			fp.x.push_back(fx);
			fp.y.push_back(fy);
		}


		for(int i = 0; i < fp.x.size() - 1; i++)
		{
			double dx = fp.x[i + 1] - fp.x[i];
			double dy = fp.y[i + 1] - fp.y[i];

			fp.yaw.push_back(atan2(dy, dx));
			fp.ds.push_back(sqrt(dx*dx + dy*dy));
		}


		for(int i = 0; i < fp.yaw.size() - 1; i++)
			fp.c.push_back((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i]);
		
	}

	return fplist;
}	