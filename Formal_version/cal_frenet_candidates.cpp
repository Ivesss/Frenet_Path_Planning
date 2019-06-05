#include "cal_frenet_candidates.hpp"
#include "orgin_frenet.hpp"
#include "combine_cands.hpp"
#include "cal_global_coord.hpp"
#include "best_path.hpp"

/*
Body part of each frenet planning algorithms.

Detailed description of each function please refers to other files

Input:
    Any variables that the hpp file passed into

Output:
    One single optimal path selected

*/
frenet_optimal_path frenet_optimal_path::origin_frenet(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0, Spline2D csp){

        vector<frenet_optimal_path> lat_can, lon_can, combined, glo_cord;
        frenet_optimal_path path;

        initial_frenet frenet;
        combine com_obj;
        global_cord gl_obj;
        bestPath bp;

		lat_can = frenet.lateral_cands(c_speed, c_d, c_d_d, c_d_dd, s0);

		lon_can = frenet.longi_cands(c_speed, c_d, c_d_d, c_d_dd, s0);

		combined = com_obj.origin_frenet(lat_can,lon_can);

		glo_cord = gl_obj.origin_frenet(combined, csp);

		//********  Filer out obstacles                                *********
		
		// filered_fp = ck_colli.static_obs.method1(static_obs, glo_cord)

		path = bp.origin_frenet(glo_cord);
    return path;
    
}


frenet_optimal_path someothermethods(){


    //No matter how the body part looks like, eventually it will return an optimal path


    //return path;
}







