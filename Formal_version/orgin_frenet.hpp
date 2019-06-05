#include "cal_frenet_candidates.hpp"
#include "parameter_read.hpp"

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


    parameter para;
    std::cout<<para.MAX_ROAD_WIDTH<< std::endl;
    vector<frenet_optimal_path> lat_can, longi_can;


	vector<frenet_optimal_path> lateral_cands(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0){


        for(double di = -para.MAX_ROAD_WIDTH; di <= para.MAX_ROAD_WIDTH + para.D_ROAD_W; di += para.D_ROAD_W){
            for(double Ti = para.MINT; Ti <= para.MAXT + para.DT; Ti += para.DT){
                
                frenet_optimal_path fp;
                quintic lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
                fp.Ti = Ti;
                for(double t = 0.0; t <= para.Tp; t += para.DT){
                    fp.t.push_back(t);
                    fp.d.push_back(lat_qp.calc_point(t));
                    fp.d_d.push_back(lat_qp.calc_first_derivative(t));
                    fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
                    fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
                } 
                lat_can.push_back(fp);
            }
        }
    return lat_can;
}
	vector<frenet_optimal_path> longi_cands(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0){


        for(double tv = para.minV; tv <= para.maxV + para.D_T_S; tv += para.D_T_S){
            for(double Ti = para.MINT; Ti <= para.MAXT + para.DT; Ti += para.DT){

                frenet_optimal_path tfp;
                quartic lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);
                tfp.Ti = Ti;
                for(double t = 0.0; t <= para.Tp; t += para.DT){
                    tfp.t.push_back(t);
                    tfp.s.push_back(lon_qp.calc_point(t));
                    tfp.s_d.push_back(lon_qp.calc_first_derivative(t));
                    tfp.s_dd.push_back(lon_qp.calc_second_derivative(t));
                    tfp.s_ddd.push_back(lon_qp.calc_third_derivative(t));
                }
                longi_can.push_back(tfp);
            }
        }
    return longi_can;
    }
};