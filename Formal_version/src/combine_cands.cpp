#include "../include/combine_cands.hpp"
#include "../include/parameter_read.hpp"

/*
Input
    lateral, longitudinal candidates

Output
    combined candidates
    
*/


extern parameter para;

vector<frenet_optimal_path> combine::origin_frenet(vector<frenet_optimal_path> lat_cands, vector<frenet_optimal_path> longi_cands){
    vector<frenet_optimal_path> fplist;

    for (int i=0; i<lat_cands.size(); i++){
        frenet_optimal_path dummy;

        dummy.d = lat_cands[i].d;
        dummy.d_d = lat_cands[i].d_d;
        dummy.d_dd = lat_cands[i].d_dd;
        dummy.d_ddd = lat_cands[i].d_ddd;

        for (int j=0; j<longi_cands.size(); j++){
            frenet_optimal_path tfp = dummy;
            tfp.s = longi_cands[j].s;
            tfp.s_d = longi_cands[j].s_d;
            tfp.s_dd = longi_cands[j].s_dd;
            tfp.s_ddd = longi_cands[j].s_ddd;
			double Jp = inner_product(tfp.d_ddd.begin(), tfp.d_ddd.end(), tfp.d_ddd.begin(), 0);
                    
            double Js = inner_product(tfp.s_ddd.begin(), tfp.s_ddd.end(), tfp.s_ddd.begin(), 0);

            double ds = pow((para.TARGET_SPEED - tfp.s_d.back()), 2);
            //Lateral Cost
            tfp.cd = para.KJ*Jp + para.KT * lat_cands[i].Ti + para.KD*tfp.d.back()*tfp.d.back();
            //Longi Cost
            tfp.cv = para.KJ*Js + para.KT*longi_cands[j].Ti + para.KD*ds;
            //Total Cost
            tfp.cf = para.KLAT*tfp.cd + para.KLON*tfp.cv;

            fplist.push_back(tfp);
        }
    }
    return fplist;

}




vector<frenet_optimal_path> combine::method2(vector<frenet_optimal_path> lat_cands, vector<frenet_optimal_path> longi_cands){
    
}


vector<frenet_optimal_path> combine::method3(vector<frenet_optimal_path> lat_cands, vector<frenet_optimal_path> longi_cands){
    
}



vector<frenet_optimal_path> combine::method4(vector<frenet_optimal_path> lat_cands, vector<frenet_optimal_path> longi_cands){
    
}