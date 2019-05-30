#include "combine_cands.hpp"

vector<frenet_path_candidates> combine::origin_frenet(vector<frenet_path_candidates> lat_cands, vector<frenet_path_candidates> longi_cands){
    vector<frenet_path_candidates> fplist;

    for (int i=0; i<lat_cands.size(); i++){
        frenet_path_candidates dummy;

        dummy.d = lat_cands[i].d;
        dummy.d_d = lat_cands[i].d_d;
        dummy.d_dd = lat_cands[i].d_dd;
        dummy.d_ddd = lat_cands[i].d_ddd;

        for (int j=0; j<longi_cands.size(); j++){
            frenet_path_candidates tfp = dummy;
            tfp.s = longi_cands[j].s;
            tfp.s_d = longi_cands[j].s_d;
            tfp.s_dd = longi_cands[j].s_dd;
            tfp.s_ddd = longi_cands[j].s_ddd;
			double Jp = inner_product(tfp.d_ddd.begin(), tfp.d_ddd.end(), tfp.d_ddd.begin(), 0);
                    
            double Js = inner_product(tfp.s_ddd.begin(), tfp.s_ddd.end(), tfp.s_ddd.begin(), 0);

            double ds = pow((TARGET_SPEED - tfp.s_d.back()), 2);
            //Lateral Cost
            tfp.cd = KJ*Jp + KT * lat_cands[i].Ti + KD*tfp.d.back()*tfp.d.back();
            //Longi Cost
            tfp.cv = KJ*Js + KT*longi_cands[j].Ti + KD*ds;
            //Total Cost
            tfp.cf = KLAT*tfp.cd + KLON*tfp.cv;

            fplist.push_back(tfp);
        }
    }
    return fplist;


}




vector<frenet_path_candidates> combine::method2(vector<frenet_path_candidates> lat_cands, vector<frenet_path_candidates> longi_cands){
    
}


vector<frenet_path_candidates> combine::method3(vector<frenet_path_candidates> lat_cands, vector<frenet_path_candidates> longi_cands){
    
}



vector<frenet_path_candidates> combine::method4(vector<frenet_path_candidates> lat_cands, vector<frenet_path_candidates> longi_cands){
    
}