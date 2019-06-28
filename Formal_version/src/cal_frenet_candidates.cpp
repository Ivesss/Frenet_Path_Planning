#include "../include/cal_frenet_candidates.hpp"
#include "../include/origin_frenet/orgin_frenet.hpp"
#include "../include/combine_cands.hpp"
#include "../include/cal_global_coord.hpp"
#include "../include/best_path.hpp"
#include "../include/BERTHA/bertha.hpp"

/*
Body part of each path planning algorithms.

Input:
    Any variables that the hpp file passed into

Output:
    One single optimal path selected

Detailed description of each function please refers to other files

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
        cout<<"x and y: "<<endl;
        cout<<path.x[0]<<" "<<path.y[0]<<endl;
        cout<<path.x[1]<<" "<<path.y[1]<<endl<<endl;
        cout<<"s and d: "<<endl;
        cout<<path.s[0]<<" "<<path.d[0]<<endl;
        cout<<path.s[1]<<" "<<path.d[1]<<endl<<endl;
        cout<<"yaw is: "<<endl;
        cout<<path.yaw[0]<<" "<<path.yaw[0]<<endl;
        cout<<path.yaw[1]<<" "<<path.yaw[1]<<endl<<endl;
        

        
    return path;
    
}

frenet_optimal_path frenet_optimal_path::baidu_apollo(){

/*
E-step:
SL projection

projected_SL_map = 

M-step:

DP_path = 

QP_path =

E-step:

projected_ST_map = 

DP_speed = 
QP_speed =

COMBINE:

combined = ()



*/


}
























//Path Planning for BERTHA
//Using cartesian (x-y) coordinate system
/*
Process:
    1. Objective function
        argmin L,
        L(X) = Joff + Jvel + Jacc + Jjerk + Jyawr
        X = (x,y)
        R^2n -> R^n
        (x,y)-> cost

    2. Constraints
        1. Internal Constraints
            veichle dynamics (curvature, tire limit)

        2. External Constraints
            Driving corridor
            Obstacles
            Object
            (ALL PUT IT INTO Polygons)
    ................

*/

//Functions classes:

// frenet_optimal_path frenet_optimal_path::BERTHA(){

//     frenet_optimal_path path;

//     obj_fcn = cal_obj_fcn(map, weights, )

//     path = bertha_optimize(obj_fcn, constraints);

//     return path;
// }

frenet_optimal_path someothermethods(){


    //No matter how the body part looks like, eventually it will return an optimal path


    //return path;
}







