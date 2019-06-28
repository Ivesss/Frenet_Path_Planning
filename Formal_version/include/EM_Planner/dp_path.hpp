#ifndef DP_PATH_HPP
#define DP_PATH_HPP

#include "common.hpp"
#include <list>
//#include "TrajectoryCost.hpp"


#include "../polynomials.hpp"

using namespace std;




class DPPATH{

public:

    DPPATH(SLPoint initSL, TrajectoryPoint cur_XY):init_point_(cur_XY){
        start.s = initSL.s;
        start.l = initSL.l;
    }


    static vector<vector<SLPoint>> SamplePathWaypoints(const TrajectoryPoint init_point, ReferenceLine reference_line_);

    bool GenerateMinCostPath(vector<vector<SLPoint>>, std::vector<DPRoadGraphNode>  *min_cost_path);

    void UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,
                                const int level, const int total_level,
                                /*TrajectoryCost *trajectory_cost,*/
                                DPRoadGraphNode *front,
                                DPRoadGraphNode *cur_node);



    vector<FrenetFramePoint> FindPathTunnel(std::vector<DPRoadGraphNode> _path);//最后生成路径之后进行评价

    PathPointxy Getfinalpath(vector<FrenetFramePoint>, ReferenceLine reference_line_); //last_path_ is empty now. now at the cloest point 





    double cal_costs( quintic &curve, const float start_s, const float end_s,
                    const uint32_t curr_level,const uint32_t total_level);

    double CalculatePathCost(
         quintic &curve, const float start_s,
        const float end_s, const uint32_t curr_level, const uint32_t total_level);

private:

    TrajectoryPoint init_point_;//路径点 CURRENT POSITION, INITIAL POINT
                                // x,y,theta,v,a...
    SLPoint start;
    FrenetFramePoint init_frenet_frame_point_;//起点
    bool IsValidCurve(const quintic &curve) const;



    

};

#endif