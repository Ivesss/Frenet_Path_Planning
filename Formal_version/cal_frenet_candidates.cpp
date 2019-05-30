#include "cal_frenet_candidates.hpp"

vector<frenet_path_candidates> frenet_path_candidates::lateral_cands::origin_frenet(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0){
	vector<frenet_path_candidates> frenet_paths;

        for(double di = -MAX_ROAD_WIDTH; di <= MAX_ROAD_WIDTH + D_ROAD_W; di += D_ROAD_W){
            for(double Ti = MINT; Ti <= MAXT + DT; Ti += DT){
                
                frenet_path_candidates fp;
                quintic lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
                fp.Ti = Ti;
                for(double t = 0.0; t <= Tp; t += DT){
                    fp.t.push_back(t);
                    fp.d.push_back(lat_qp.calc_point(t));
                    fp.d_d.push_back(lat_qp.calc_first_derivative(t));
                    fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
                    fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
                } 
                frenet_paths.push_back(fp);
            }
        }
    return frenet_paths;
}

vector<frenet_path_candidates> frenet_path_candidates::longi_cands::origin_frenet(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0){
	vector<frenet_path_candidates> frenet_paths;

        for(double tv = minV; tv <= maxV + D_T_S; tv += D_T_S){
            for(double Ti = MINT; Ti <= MAXT + DT; Ti += DT){

                frenet_path_candidates tfp;
                quartic lon_qp(s0, c_speed, 0.0, tv, 0.0, Ti);
                tfp.Ti = Ti;
                for(double t = 0.0; t <= Tp; t += DT){
                    tfp.t.push_back(t);
                    tfp.s.push_back(lon_qp.calc_point(t));
                    tfp.s_d.push_back(lon_qp.calc_first_derivative(t));
                    tfp.s_dd.push_back(lon_qp.calc_second_derivative(t));
                    tfp.s_ddd.push_back(lon_qp.calc_third_derivative(t));
                }
                frenet_paths.push_back(tfp);
            }
        }
    return frenet_paths;
}

vector<frenet_path_candidates> frenet_path_candidates::lateral_cands::method2(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0)
{}

// Add other lat/longi candidate methods
