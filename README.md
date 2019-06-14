# Frenet_Path_Planning

Frenet path planning according to "Optimal trajectory generation for dynamic street scenarios in a Frenét Frame" from Moritz Werling, C++ version.

This is a combination of path planning algorithms based on Frenet coordinate system from Moritz Werling, the basic idea is from  [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://ieeexplore.ieee.org/document/5509799).

Version 5/30/2019:

Modulated the basic overall structure and provided program interfaces for different functionality:  
1. Later/longitudinal path candidate generation    
2. Candidate combination   
3. Global coord generation   
4. Optimal path selection    

Visualization is done using matplotlib.h, a python package written in C++, for more detailed desciption and usage please refer to [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

Run frenet.sh executable.

Version 6/6/2019:

Changes:   
1. Modified path generation interface inside main function to one line: output a single optimal path.    
2. Added parameter.json file including all tunning parameters and global map coordinate.   
3. Changed header file inclusion, make the whole project more C++ formal.  

