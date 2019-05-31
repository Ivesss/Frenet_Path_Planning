# Local_traj_planning_exp
This is a combination of path planning algorithms based on Frenet coordinate system from Moritz Werling, the basic idea is from  [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://ieeexplore.ieee.org/document/5509799).

Version 5/30/2019:

Modulated the basic overall structure and provided program interfaces for different functionality:  
1. Later/longitudinal path candidate generation    
2. Candidate combination   
3. Global coord generation   
4. Optimal path selection    

Visualization is done using matplotlib.h, a python package written in C++, for more detailed desciption and usage please refer to [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

Run frenet.sh executable.

