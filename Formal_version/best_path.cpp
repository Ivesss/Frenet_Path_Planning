#include "best_path.hpp"

/*
Chose best path according to different method.

Input
    path candidates of each algorithm

Output
    chose one best optimal path among all candidates according to different method

*/



frenet_optimal_path bestPath::origin_frenet(vector<frenet_optimal_path> glo_fp){
	//frenet_path_candidates frenet_obj;

	double min_cost = 1000000000000;
	frenet_optimal_path bestpath;
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


frenet_optimal_path bestPath::method2(vector<frenet_optimal_path> fplists){
}

frenet_optimal_path bestPath::method3(vector<frenet_optimal_path> fplists){
}