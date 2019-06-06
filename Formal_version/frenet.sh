#sudo g++ -std=c++11 -I/usr/include/python2.7 -lpython2.7 -o output polynomials.cpp #parameter_read.cpp orgin_frenet.cpp combine_cands.cpp
sudo g++ -std=c++11 -I/usr/include/python2.7 -lpython2.7 -o output polynomials.cpp cubic_spline_planner.cpp parameter_read.cpp orgin_frenet.cpp combine_cands.cpp cal_frenet_candidates.cpp cal_global_coord.cpp best_path.cpp my_main.cpp


./output
#./a.out
