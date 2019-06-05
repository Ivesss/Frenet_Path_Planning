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
<<<<<<< HEAD
        vector<frenet_optimal_path> method1(std::vector<std::pair<int,int>>, vector<frenet_optimal_path>);
=======
        vector<frenet_path_candidates> method1(std::vector<std::pair<int,int>>, vector<frenet_path_candidates>);
>>>>>>> 9e8a7fd237ad8fa467503d48a513ea51c0b0a33f
    };
};
#endif