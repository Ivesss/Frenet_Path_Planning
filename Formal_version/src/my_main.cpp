#include "../include/cal_frenet_candidates.hpp"
#include "../include/cubic_spline_planner.hpp"
#include "../include/polynomials.hpp"
#include "../include/combine_cands.hpp"
#include "../include/cal_global_coord.hpp"
#include "../include/best_path.hpp"
#include "../include/collision_checking.hpp"
#include "../include/obstacles.hpp"
#include "../include/parameter_read.hpp"

//below headers for EM_Planner
#include "../include/EM_Planner/Clothoid.h"
#include "../include/EM_Planner/common.hpp"
#include "../include/EM_Planner/conversion.hpp"
#include "../include/EM_Planner/dp_path.hpp"

//END
#include "../../matplotlibcpp.h"

using namespace matplotlibcpp;





int main(){
	//read parameter from json
    extern parameter para;
    para.read_para();

    vecD tx, ty, tyaw, tc;
	//Some variables for testing only, will move into corresponding function later on
	//Move csp into origin_frenet
	//center line, upper/lower road boundary into parameter?
	
	Spline2D csp = calc_spline_course(para.wx, para.wy, tx, ty, tyaw, tc, 0.1);
	vector<double> centerline(tx.size(),4);

	//     end


//BELOW IS FOR TESTING @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

	double near_x, near_y, res_s, res_d;

	double x_c = 5, y_c = 5, init_heading = 70,velo;

	std::array<double, 3> ptr_s_condition;
    std::array<double, 3> ptr_d_condition;

	ReferenceLine reference_line_info;


	ConverCartesionFrenet::XY2SL(x_c, y_c, tx,ty,tyaw,reference_line_info);//get nearest x and y
	//printVecD(tyaw);

	// for (int i=0; i<reference_line_info.reference_points_.size(); i++){
	// 	cout<<reference_line_info.reference_points_[i].s<<endl;
	// }
	// cout<<"Following is from cubic spline test"<<endl;         SAME
	// vecD c_spline_S = csp.calc_s(tx,ty);
	// printVecD(c_spline_S);
	cout<<reference_line_info.reference_points_[0].x<<"   "<<reference_line_info.reference_points_[0].y<<endl;

	TrajectoryPoint init = ConverCartesionFrenet::initial_Point_trans(reference_line_info.reference_points_.front(),
	x_c,y_c,init_heading,velo,0,0,ptr_s_condition,ptr_d_condition);
	// initial x,y,theta,s,l

	vector<vector<SLPoint>>  all_sampled_sl_points = DPPATH::SamplePathWaypoints(init, reference_line_info);

	

	vecD global_x, global_y;

	for (int i=0; i<all_sampled_sl_points.size();i++){
		for(int j=0; j<all_sampled_sl_points[1].size();j++){
	 		//cout<<all_sampled_sl_points[i][j].s<<"  "<<all_sampled_sl_points[i][j].l<<endl;



		int finalindex = 0;
        for(auto referSLPoint:reference_line_info.reference_points_)
        {
            if(referSLPoint.s>all_sampled_sl_points[i][j].s) {
                break;
            }

            finalindex++;
        }
        if(finalindex>reference_line_info.reference_points_.size()-1)
        {
            finalindex = reference_line_info.reference_points_.size()-1;
            
            cout<<"#####################"<<endl;
            break;
        }
		//cout<<finalindex<<endl;
        auto referpoint = reference_line_info.reference_points_[finalindex];
        // if(lastindex == finalindex)
        //     double a =0;
        // lastindex = finalindex;
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double kappa = 0.0;//output parameters for that s-l point
        double v = 0.0;
        double a = 0.0;

		ConverCartesionFrenet::frenet_to_cartesian(referpoint,all_sampled_sl_points[i][j].s,
		all_sampled_sl_points[i][j].l, x, y);

        if(x==0.0&&y==0.0)
            cout<<"CONVERSION FAILED"<<endl;

		global_x.push_back(x);
		global_y.push_back(y);//@@@@@@@@@@@@@@@ some sampled lateral coordinate not perpendicular with S axis.
		}
	 }

// NOW , BEGIN TO GENERATE MINIMUM COST PATH.

	std::vector<DPRoadGraphNode> min_cost_path;


	    //将起点加到路点的最前面
	SLPoint initial;
	initial.s = init.path_point.s;
	initial.l = init.path_point.l;

    all_sampled_sl_points.insert(all_sampled_sl_points.begin(),
                          std::vector<SLPoint>{initial});

	DPPATH dpp(initial,init);
	dpp.GenerateMinCostPath(all_sampled_sl_points, &min_cost_path);
	vector<FrenetFramePoint> final_path = dpp.FindPathTunnel(min_cost_path);

    PathPointxy ep = dpp.Getfinalpath(final_path, reference_line_info); //last_path_ is empty now. now at the cloest point 




	vecD final_x, final_y;
	for (int i=0; i<ep.pps.size();i++){



		final_x.push_back(ep.pps[i].x);
		final_y.push_back(ep.pps[i].y);
	}

	printVecD(final_x);
	printVecD(final_y);

    clf();
	figure_size(1200, 1200);
	plot(tx, ty);
	plot(tx,centerline,"-.y");
	plot(global_x,global_y, "-og");
	plot(final_x,final_y,"-xr");









// ABOVE IS FOR TESTING@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


//Initial profile
	
	// double c_speed = 10.0 / 3.6, c_d = 4.0, c_d_d = 2.0, c_d_dd = 0.0, s0 = 10.0;

	// double area = 20.0;

	// //************* DEFINE CLASS OBJECTS ****************
	// frenet_optimal_path fp_optimal;
	// combine cbn_obj;
	// global_cord glo_obj;
	// check_collision ck_colli;
	// bestPath bestp;
	// //************* DEFINE CLASS OBJECTS ****************

	// vector<frenet_optimal_path> lat_can, lon_can, combined, glo_cord;
	// frenet_optimal_path path;

	// int cnt = 0;
	//figure_size(1200, 780);

	// for(int i = 0; i < 1; i++)
	// {

	// 	//Generate one single optimal path according to different methods 
	// 	path = fp_optimal.origin_frenet(c_speed, c_d, c_d_d, c_d_dd, s0, csp);

	// 	std::cout<<path.x[0];
	// 	s0 = path.s[1];
	// 	c_d = path.d[1];
	// 	c_d_d = path.d_d[1];
	// 	c_d_dd = path.d_dd[1];
	// 	c_speed = path.s_d[1];

	// 	if(sqrt(pow(path.x[1] - tx[-1], 2) + pow(path.y[1] - ty[-1], 2)) <= 1.0)
	// 	{
	// 		cout << "Goal!\n#machaya\n";
	// 		break;
	// 	}

	// 	cout << "Printing the path params :\n";
	// 	cout << s0 << " \n" << c_d << " \n" << c_d_d << " \n" << c_d_dd << " \n" << c_speed << "\n\n";
	// 	 printVecD(path.x);
	// 	// printVecD(path.y);
    //     cout<<"Best path cost is: "<<path.cf<<endl<<endl;
	// 	cnt ++;
    //     cout<<cnt<<endl;


	// 	// ************ PLOTS ***************

    //     clf();
    //     plot();
	// 	plot(tx,centerline,"-.y");
    //     plot(tx, ty);
	// // 	// for (int k=0; k<glo_cord.size(); k++){
	// // 	// 	plot(glo_cord[k].x, glo_cord[k].y, "--g");
	// // 	// }
		
    // //     plot(path.x, path.y, "-or");
    // //     grid(1);	
	// // 	pause(0.0001);
	// // }
	
	grid(1);
	//pause(0);
	matplotlibcpp::show();
	
	return 0;
 }
