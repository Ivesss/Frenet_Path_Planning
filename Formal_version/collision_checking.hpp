#ifndef COLLISION_CHECKING
#define COLLISION_CHECKING

#include "cal_frenet_candidates.hpp"
/*Input: obstacle list(static so far)
         
  Output: List of filtered path candidates without collision
*/
class check_collision{

    
    public:
    class moving_obs{
        
        public:
        
    };

    class static_obs{

        public:
        vector<frenet_path_candidates> method1(std::vector<std::pair<int,int>>, vector<frenet_path_candidates>);
    };
};
#endif