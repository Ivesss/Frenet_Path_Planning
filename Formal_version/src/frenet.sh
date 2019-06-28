sudo g++ -std=c++11 -o output EM_Planner/Clothoid.cpp EM_Planner/common.cpp EM_Planner/conversion.cpp EM_Planner/dp_path.cpp polynomials.cpp cubic_spline_planner.cpp parameter_read.cpp origin_frenet/orgin_frenet.cpp combine_cands.cpp cal_frenet_candidates.cpp cal_global_coord.cpp best_path.cpp my_main.cpp -I/usr/include/python2.7 -lpython2.7

#Compile time is a bit slow
#Make a CMAKE file later

./output

