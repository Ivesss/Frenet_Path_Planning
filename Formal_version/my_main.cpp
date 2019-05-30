#include "cal_frenet_candidates.cpp"
#include "cubic_spline_planner.cpp"
#include "polynomials.cpp"
#include "combine_cands.cpp"
#include "cal_global_coord.cpp"
#include "best_path.cpp"
#include "/home/uisee/Uisee_Project/Ives/matplotlibcpp.h"

//IF ONLY COMPILING THIS FILE, do: sudo g++ my_main.cpp -std=c++11 -I/usr/include/python2.7 -lpython2.7


using namespace matplotlibcpp;


int main(){

    vecD wx = {0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0};
    vecD wy = {0.0, -4.0, 1.0, 6.5, 8.0, 10.0, 6.0};


    vecD tx, ty, tyaw, tc;
	Spline2D csp = calc_spline_course(wx, wy, tx, ty, tyaw, tc, 0.1);
    
	double c_speed = 10.0 / 3.6, c_d = 2.0, c_d_d = 2.0, c_d_dd = 0.0, s0 = 0.0;

	double area = 20.0;

	//************* DEFINE CLASS OBJECTS ****************
	frenet_path_candidates::lateral_cands frenet_lat_obj;
	frenet_path_candidates::longi_cands frenet_lon_obj;
	combine cbn_obj;
	global_cord glo_obj;
	bestPath bestp;
	//************* DEFINE CLASS OBJECTS ****************

	vector<frenet_path_candidates> lat_can, lon_can, combined, glo_cord;
	frenet_path_candidates path;

	int cnt = 0;

	for(int i = 0; i < 100; i++)
	{
		//********  Cal lateral and Longitudinal candidates seperately *********

		lat_can = frenet_lat_obj.origin_frenet(c_speed, c_d, c_d_d, c_d_dd, s0);
		lon_can = frenet_lon_obj.origin_frenet(c_speed, c_d, c_d_d, c_d_dd, s0);

		//******** Combination of Lat and Longi candidates            *********

		combined = cbn_obj.origin_frenet(lat_can,lon_can);


		//********  Cal global coordinates                             *********

		glo_cord = glo_obj.origin_frenet(combined, csp);

		//********  Select best path                                   *********

		path = bestp.origin_frenet(glo_cord);


        //********                      END                            *********
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

        clf();
        plot();
        plot(tx, ty);
		// for (int k=0; k<glo_cord.size(); k++){
		// 	plot(glo_cord[k].x, glo_cord[k].y, "--g");
		// }
        plot(path.x, path.y, "-or");
        grid(1);	
		pause(0.0001);
	}

	show();
	
	return 0;
 }

// plt.cla()

// for j in range(len(fplist)):
//     plt.plot(fplist[j].x[1:], fplist[j].y[1:],".g")
//     plt.plot(fplist[j].x[-1], fplist[j].y[-1],'*g')
// plt.plot(tx, ty,"y")
// plt.plot(best_p.x[1:], best_p.y[1:],"-or")
// print("Best_path cost:",best_p.cf)
// plt.grid(True)
// plt.pause(0.0001)
