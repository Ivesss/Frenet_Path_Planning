#include "../../include/EM_Planner/dp_path.hpp"
#include "../../include/EM_Planner/conversion.hpp"




vector<vector<SLPoint>> DPPATH::SamplePathWaypoints(const TrajectoryPoint init_point, ReferenceLine reference_line_) {
    // if(reference_line_.reference_points_.empty())
    //     return false;
    //采样间隔最小为8米，最大15米，根据初速度来确定
    

    //const double kMinSampleDistance = 40.0;
    // 总长度 = min(初始点 + max(初始速度 × 8, 最小采样距离), 参考线长度）
//    const float total_length = std::fmin(
//            init_sl_point_.s + std::fmax(init_point.v * 8.0, kMinSampleDistance)
    const float total_length= reference_line_.Length; // total length of the curve

    //constexpr float kSamplePointLookForwardTime = 4.0;
    // 采样步长 = 初始速度 × 采样前视时长，要求：
    // step_length_min(默认值：8) <= 采样步长 <= step_length_max(默认值：15)//正常情况 至少10米
    // float step_length //采样步长其实应该根据车速来选择 效果较好。
    //        =std::fmin( std::fmax(init_point.v * kSamplePointLookForwardTime, 8), 15);
    float step_length = 10;  // at least 10m
    // 累计轨迹弧长
    // float accumulated_s = init_sl_point_.s;
	float accumulated_s = init_point.path_point.s;
    //float prev_s = accumulated_s;
    vector<vector<SLPoint>> as;
    while(accumulated_s<total_length)//纵向
    {
        vector<SLPoint> levelPoints;
        accumulated_s +=step_length;
        if(accumulated_s>(total_length+step_length/2.0))//超出总长度过多
            break;
        const float kDefaultUnitL =1;//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@2
        const float offset = 4;//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        for (float i = -offset; i <= offset; ) {
            SLPoint samplepoint;
            samplepoint.s=accumulated_s;
            samplepoint.l =i;
            levelPoints.push_back(samplepoint);
            i+=kDefaultUnitL;
        }
 
        as.push_back(levelPoints);
        // if(accumulated_s>=30)
        //     step_length =15.0;
		
    }
	// ofstream log;
    // log<<as.size()*as.front().size()<<"\t";
    vector<vector<SLPoint>> m_AllSamplePoints; //(s,l) coordinates along the curve AT THE START POINT NOW

    m_AllSamplePoints=as;
    if(m_AllSamplePoints.empty())
		cout<<"sampling failed"<<endl;
        
    else
        return m_AllSamplePoints;

}

bool DPPATH::GenerateMinCostPath(vector<vector<SLPoint>> allsampledpts, 
std::vector<DPRoadGraphNode>  *min_cost_path){

    // std::list<std::list<int>> graph_nodes{{42,3,2},{2,2,1},{444,67,3},{7,3,2}};
    // graph_nodes.emplace_back();//re-add another list<int> at the end
    // graph_nodes.back().emplace_back(12354);

    // list<list<int>>::iterator it = graph_nodes.begin();
    // for(it =graph_nodes.begin();it!=graph_nodes.end();it++){
    //     for(list<int>::iterator ot=it->begin(); ot!=it->end();ot++){
    //         cout<<*ot<<" ";
    //     }
    //     cout<<endl;
    // }





//    TrajectoryCost trajectory_cost(config_, reference_line_,m_obstacles,  init_sl_point_ ,m_lastFrenetPath);//initialize through constructor 
  

    std::list<std::list<DPRoadGraphNode>> graph_nodes; //information of every sampled pts
    graph_nodes.emplace_back();//reserve place

    graph_nodes.back().emplace_back(start,nullptr);

    auto &front = graph_nodes.front().front();//return reference of the first element 
    // front = 0, 12.3143, same as allsampledpts[0][0].s,l ==> good.

    size_t total_level = allsampledpts.size();
    //total level = 26
    //graph_node size final ==> 26, each has 9 element in it
    for (size_t level = 1; level < total_level; ++level) {

        const auto &prev_dp_nodes = graph_nodes.back();
        const auto &level_points = allsampledpts[level];

        graph_nodes.emplace_back(); //reserve position, because THIS IS A 2D LISTS
        //从后一个点向前一个点生成曲线
        for (int num = 0; num < level_points.size(); ++num) // generate quntic curve from all 
        //notes in one level. Then go to next level. Find min path
        {
            const auto &cur_point = level_points[num];

            graph_nodes.back().emplace_back(cur_point, nullptr);//stored as DPRoadGra type
            auto &cur_node = graph_nodes.back().back();
           // cout<<cur_node.sl_point.s<< "  x  "<<cur_node.sl_point.l<<endl;

            //生成到之前cost最小的路径 最后存在graph_nodes.back().back();  from current 
            //node in level, to ALL previous nodes in previous level
            UpdateNode(prev_dp_nodes, level, total_level, &front,
                       &cur_node);
            //output cur_node.min_cost_prev_node->sl_point.s does work.(have sl points)@@@@@@@@@@@@
            //cout<<cur_node.min_cost_prev_node->sl_point.s<<" "<<cur_node.min_cost_prev_node->sl_point.l<<endl;
        }
    }

   
    DPRoadGraphNode fake_head;
    double min_c = 1000000000000;
    //find the minium cost from ALL THE NODES from the LAST LEVEL，===> fake_head
    for (const auto &cur_dp_node : graph_nodes.back()) {
        if (cur_dp_node.min_cost < min_c){
            min_c = cur_dp_node.min_cost;
            fake_head = cur_dp_node;
        }
        // fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,
        //                       cur_dp_node.min_cost);
        //cout<<cur_dp_node.min_cost_prev_node->sl_point.s<<"~~~~~~"<<cur_dp_node.min_cost_prev_node->sl_point.l<<endl;
        //above works
        
    //cout<<cur_dp_node.min_cost_prev_node->sl_point.s<<" "<<cur_dp_node.min_cost_prev_node->sl_point.l<<endl;

    }
    //@@@@@@@@@@@@@@@@@@@@ problem is, SL, cost, mini, not updating. Pointer problem@@@@@@@@@@
    cout<<"minimum cost node(SL) from the final level is: "<<fake_head.sl_point.s<<" "<<fake_head.sl_point.l<<endl;


    const auto *fatherNode =&fake_head;
    std::vector<RoadPoint> repath;
    while(fatherNode->min_cost_prev_node)
    {
        fatherNode =fatherNode->min_cost_prev_node;
        min_cost_path->push_back(*fatherNode);
        repath.emplace_back(fatherNode->sl_point.l,fatherNode->sl_point.s,0.0,0.0);
    }
    //找到起点

    if (fatherNode != &graph_nodes.front().front()) {
        cout<<" 最优路径没有导向起点。。。。。。。"<<endl;
        return false;
    }
    //cout<<"   xxxxxxx"<<endl;
    std::reverse(min_cost_path->begin(),min_cost_path->end());




}


void DPPATH::UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,
                             const int level, const int total_level,
                             /*TrajectoryCost *trajectory_cost,*/
                             DPRoadGraphNode *front,
                             DPRoadGraphNode *cur_node){
    // 生成当前点到前一level所有航点的的曲线 5次曲线, find the minimum pre_node to cur_node
    //and store the cost, its curve and pre_node

    // prev_nodes == graph_node.back()
    for (auto &prev_dp_level:prev_nodes){
        const auto &prev_sl_point = prev_dp_level.sl_point;
        const auto &cur_point = cur_node->sl_point;
        // cout<<"pre_sl_pt:   "<<prev_sl_point.s<<" "<<prev_sl_point.l<<endl;
        // cout<<"cur_pt:   "<<cur_point.s<<" "<<cur_point.l<<endl;
        // SL INDEED PASSED HERE @@@@@@@@@@@@@@@@

        float init_dl = 0.0;
        float init_ddl = 0.0;
        if (level == 1) {
            init_dl = init_frenet_frame_point_.dl;
            init_ddl = init_frenet_frame_point_.ddl;
        }
        quintic poly_curve(prev_sl_point.l, init_dl, init_ddl,
                                       cur_point.l, 0.0, 0.0,/*path following, so dl, ddl = 0*/
                                       cur_point.s - prev_sl_point.s);
        //@@@@@@@@@@@@ above quntic curve diff @@@@@@@ 
        // maybe not so different @@@@@@@@@@@@@@@22
        // NOW, determine if the curve is successfully generated!!!!@@
        // if(!IsValid):return....
        //Below, calculate costs

        double path_cost = cal_costs(poly_curve,prev_sl_point.s, 
                            cur_point.s,level, total_level);//@@@@@ add prev_dp_level.min_cost
       // cout<<"path cost: "<< path_cost<<endl; // cost weired @@@@@probably ok?
        // cout<<"Current node is: "<<cur_node->sl_point.s<<" "<<cur_node->sl_point.l<<endl;
        // cout<<"path cost from prev.s to cur.s is: "<<path_cost<<endl;  


        cur_node->UpdateCost(&prev_dp_level,poly_curve,path_cost);

        //cout<<"Pre min node is:   ";
        //cout<<cur_node->min_cost_prev_node->sl_point.s<<" x "<<cur_node->min_cost_prev_node->sl_point.l<<endl;
        
        //Above test works now, but commented last part in COST func @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        //#TODO IF THIS PATH OR NODE IS COLLIDED, MAKE IT A NULL PTR
    }
    //是否有更优父节点
    // 尝试将当前点直接连接到初始点，看其代价是否比当前点到前一level航点的最小代价还小，
    // 若小于则将最小代价航点更新。这种情况一般只会存在于改变车道的情形。 @@@@@@@@@@@@@@@@@@
    // try to connect the current point with the first point directly
    // if (level >= 2) {
    //     const float init_dl = init_frenet_frame_point_.dl;
    //     const float init_ddl = init_frenet_frame_point_.ddl;
    //     quintic curve(start.l, init_dl, init_ddl,
    //                                    cur_node->sl_point.l, 0.0, 0.0,
    //                                    cur_node->sl_point.s - start.s);
    //     // if (!IsValidCurve(curve)) {//判断曲线是否生成
    //     //     return;
    //     // }
    //     const auto cost = cal_costs(
    //             curve, start.s, cur_node->sl_point.s, level, total_level);
    //           //  cout<<"cost from start pt: "<<cost<<endl;
    //     cur_node->UpdateCost(front, curve, cost);
    //     // if(cur_node->min_cost.cost_items[ComparableCost::HAS_COLLISION])
    //     //     cur_node->min_cost_prev_node = nullptr; //@@@@ if node is collided
    // }






}


double DPPATH::cal_costs( quintic &curve, const float start_s, const float end_s,
                    const uint32_t curr_level,const uint32_t total_level)
{
    // setCarsize();
    // ComparableCost total_cost;
    // path cost
    double total_cost = 0;


    total_cost += CalculatePathCost(curve, start_s, end_s, curr_level, total_level);
    //cout<<"PathCost:"<<total_cost.smoothness_cost<<" ";
    // static obstacle cost
   // struct timeval t1, t2;
   // gettimeofday(&t1, NULL);
    //total_cost += CalculateStaticObstacleCost(curve, start_s, end_s);
    //gettimeofday(&t2,NULL);
    //level可能需要调整。
    //historical cost can be ignored for now?? dynamic obstacle
   // total_cost += CalculateHistoricalCost(curve, start_s, end_s, curr_level, total_level);
    //auto deltaT = (t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec;//微秒
    //cout<<"Obstacle check time:"<<deltaT<<"us"<<endl;
    //cout<<"ObstacleCost:"<<total_cost.safety_cost<<endl;
    // dynamic obstacle cost
    //total_cost += CalculateDynamicObstacleCost(curve, start_s, end_s);
    return total_cost;
}

double DPPATH::CalculatePathCost(
         quintic &curve, const float start_s,
        const float end_s, const uint32_t curr_level, const uint32_t total_level){

    // ComparableCost cost;
    double cost;
    float path_cost = 0.0;
    std::function<float(const float)> quasi_softmax = [this](const float x) {
        // const float l0 = this->config_.path_l_cost_param_l0;//1.5
        // const float b = this->config_.path_l_cost_param_b;//0.4
        // const float k = this->config_.path_l_cost_param_k;//1.5
        const float l0 = 1.5;
        const float b = 0.4;
        const float k = 1.5;
        return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
    };

    //获取车辆大小信息 用car代替即可
//    const auto &vehicle_config =
//            common::VehicleConfigHelper::instance()->GetConfig();
    Car vehiclemodel;
    const float width = 2.1;//vehicle_config.vehicle_param().width();
    const double path_l_cost = 6.5;
    const double path_dl_cost = 8000;
    const double path_ddl_cost = 5;
    const int path_resolution = 1;
    const double path_end_l_cost = 5000;

    for (float curve_s = 0.0; curve_s < (end_s - start_s);
         curve_s += 1) {
        const float l = curve.calc_point(curve_s);//@@@@@@@ that 6 coefficients of quintic curve

        path_cost += l * l * path_l_cost* quasi_softmax(std::fabs(l));
        //* quasi_softmax(std::fabs(l));
                        // purpose of softmax??????????????
        const float dl = std::fabs(curve.calc_first_derivative(curve_s));
        path_cost += dl * dl * path_dl_cost;

        const float ddl = std::fabs(curve.calc_second_derivative(curve_s));
        path_cost += ddl * ddl * path_ddl_cost;
    }// NO TIME, NO derivative of curvature. time in ST?
    path_cost *= path_resolution;//分辨率  步长@@@@@@@@@@@@@@@@@@@

    // if (curr_level == total_level-1) {//TODO :never step into,find the reason
    //     const float end_l = curve.calc_point(end_s - start_s);
    //     //path_cost += end_l*end_l * path_end_l_cost;
    //     path_cost += std::sqrt(end_l - start.l / 2.0) * path_end_l_cost; // path_end_l_cost：10000 
    //     //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ PURPOSE OF THIS???????????
    // }
    //cost.smoothness_cost = path_cost;
    return path_cost;

    }


vector<FrenetFramePoint> DPPATH::FindPathTunnel(vector<DPRoadGraphNode> min_cost_path)//最后生成路径之后进行评价
{
// from the minimum cost path, get the COMPLETE information including SL, waypoints....
    std::vector<FrenetFramePoint> frenet_path;//最终的路径, single optimal path?
    float accumulated_s = start.s;//累计的s
    const float path_resolution = 1;//config_.path_resolution == 1

    for (std::size_t i = 1; i < min_cost_path.size(); ++i) {
        const auto &prev_node = min_cost_path[i - 1];
        const auto &cur_node = min_cost_path[i];

        const float path_length = cur_node.sl_point.s - prev_node.sl_point.s;
        float current_s = 0.0;
        const auto &curve = cur_node.min_cost_curve;
        while (current_s + path_resolution /*2.0*/ < path_length) {
            ////l dl ddl 三阶
            const float l = curve.calc_point(current_s);
            const float dl = curve.calc_first_derivative(current_s);
            const float ddl = curve.calc_second_derivative(current_s);
            FrenetFramePoint frenet_frame_point;

            frenet_frame_point.s= (accumulated_s + current_s);
            frenet_frame_point.l=(l);
            frenet_frame_point.dl=(dl);
            frenet_frame_point.ddl=(ddl);
            frenet_path.push_back(std::move(frenet_frame_point));
            current_s += path_resolution;
        }
        //长度可能不一致
//        if (i == min_cost_path.size() - 1) {
//            accumulated_s += current_s;
//        } else
            
            accumulated_s += path_length;
        
    }
    if(frenet_path.size()<2)
        cout<<"empty frenet_path"<<endl;
    return  frenet_path;
    //FrenetFramePath tunnel(frenet_path);
    //path_data->SetReferenceLine(&reference_line_);
    //path_data->SetFrenetPath(tunnel); 


}

PathPointxy DPPATH::Getfinalpath(vector<FrenetFramePoint> last_p, ReferenceLine reference_line_){


// struct PathPointxy{
//     PathPointxy():length(0),dis(0),obssize(0){};
//     std::vector<RoadPoint> pps;
//     double length;
//     double dis;
//     double obssize;
// };
    vector<FrenetFramePoint> lastFrenetPath = last_p;




    PathPointxy path;
    int index_last=0;
    double lasts=0.0;

    bool outrange=0;
    int lastindex =-1;
    for (auto frenetp:last_p)
    {
        int finalindex =0;
        for(auto referSLPoint:reference_line_.reference_points_)
        {
            if(referSLPoint.s>frenetp.s) {
                break;
            }

            finalindex++;
        }
        if(finalindex>reference_line_.reference_points_.size()-1)
        {
            finalindex = reference_line_.reference_points_.size()-1;
            outrange = true;
            //cout<<"#####################"<<endl;
            break;
        }
        auto referpoint = reference_line_.reference_points_[finalindex];
        if(lastindex == finalindex)
            double a =0;
        lastindex = finalindex;


        // SL to XY
        
        double ds = 5, dds = 0.5; // random ds and dds
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double kappa = 0.0;
        double v = 0.0;
        double a = 0.0;

        ConverCartesionFrenet::frenet_to_cartesian(referpoint, frenetp.s,ds,dds,
        frenetp.l, frenetp.dl, frenetp.ddl, x, y,theta,kappa, v,a);

        RoadPoint ptemp(x,y,theta,kappa);
        if(x==0.0&&y==0.0)
            cout<<"aa"<<endl;
        path.pps.push_back(ptemp);
    }
    path.length = last_p.back().s;


    return path;

}


// bool DPPATH::IsValidCurve(const quintic &curve) const {
//     constexpr float kMaxLateralDistance = 20.0;
//     for (float s = 0.0; s < curve.ParamLength(); s += 2.0) {
//         const float l = curve.Evaluate(0, s);
//         if (std::fabs(l) > kMaxLateralDistance) {
//             return false;
//         }
//     }
//     return true;
// }




