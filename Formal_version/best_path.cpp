#include "best_path.hpp"




frenet_path_candidates bestPath::origin_frenet(vector<frenet_path_candidates> glo_fp){
	//frenet_path_candidates frenet_obj;

	double min_cost = FLT_MAX;
	frenet_path_candidates bestpath;
	for(int i=0; i<glo_fp.size(); i++)
	{
		if(min_cost >= glo_fp[i].cf)
		{
			min_cost = glo_fp[i].cf;
			bestpath = glo_fp[i];
		}
	}

	return bestpath;
}


frenet_path_candidates bestPath::method2(vector<frenet_path_candidates> fplists){
}

frenet_path_candidates bestPath::method3(vector<frenet_path_candidates> fplists){
}