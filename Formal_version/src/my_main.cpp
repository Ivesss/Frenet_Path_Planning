#include "../include/cal_frenet_candidates.hpp"
#include "../include/cubic_spline_planner.hpp"
#include "../include/polynomials.hpp"
#include "../include/combine_cands.hpp"
#include "../include/cal_global_coord.hpp"
#include "../include/best_path.hpp"
#include "../include/collision_checking.hpp"
#include "../include/obstacles.hpp"
#include "../include/parameter_read.hpp"

//#include "../matplotlibcpp.h"

//using namespace matplotlibcpp;


int main(){
	//read parameter from json
    extern parameter para;
    para.read_para();

    vecD tx, ty, tyaw, tc;
	Spline2D csp = calc_spline_course(para.wx, para.wy, tx, ty, tyaw, tc, 0.1);
	double c_speed = 10.0 / 3.6, c_d = 2.0, c_d_d = 2.0, c_d_dd = 0.0, s0 = 0.0;

	double area = 20.0;

	//************* DEFINE CLASS OBJECTS ****************
	frenet_optimal_path fp_optimal;
	combine cbn_obj;
	global_cord glo_obj;
	check_collision ck_colli;
	bestPath bestp;
	//************* DEFINE CLASS OBJECTS ****************

	vector<frenet_optimal_path> lat_can, lon_can, combined, glo_cord;
	frenet_optimal_path path;

	int cnt = 0;

	for(int i = 0; i < 100; i++)
	{

		//Generate one single optimal path according to different methods 
		path = fp_optimal.origin_frenet(c_speed, c_d, c_d_d, c_d_dd, s0, csp);

		std::cout<<path.x[0];
		s0 = path.s[1];
		c_d = path.d[1];
		c_d_d = path.d_d[1];
		c_d_dd = path.d_dd[1];
		c_speed = path.s_d[1];

		if(sqrt(pow(path.x[1] - tx[-1], 2) + pow(path.y[1] - ty[-1], 2)) <= 1.0)
		{
			cout << "Goal!\n#machaya\n";
			break;
		}

		cout << "Printing the path params :\n";
		cout << s0 << " \n" << c_d << " \n" << c_d_d << " \n" << c_d_dd << " \n" << c_speed << "\n\n";
		 printVecD(path.x);
		// printVecD(path.y);
        cout<<"Best path cost is: "<<path.cf<<endl<<endl;
		cnt ++;
        cout<<cnt<<endl;


		// ************ PLOTS ***************

        // clf();
        // plot();
        // plot(tx, ty);
		// for (int k=0; k<glo_cord.size(); k++){
		// 	plot(glo_cord[k].x, glo_cord[k].y, "--g");
		// }
        // plot(path.x, path.y, "-or");
        // grid(1);	
		// pause(0.0001);
	}
	//
	// grid(1);
	// pause(0.0001);
	// show();
	
	return 0;
 }
