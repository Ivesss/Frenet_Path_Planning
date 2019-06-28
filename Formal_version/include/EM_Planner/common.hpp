#ifndef COMMON_HPP
#define COMMON_HPP

#include <vector>
#include <iostream>
#include <array>
#include "Clothoid.h"
#include "../../include/polynomials.hpp"




struct RoadPoint{
    double x;
    double y;
    double angle;
    double k;
    double cost;
    RoadPoint(){
        x=0;
        y=0;
        angle=0;
        k=0;
        cost=0;
    }
    RoadPoint(double _x,double _y,double _angle,double _k): x(_x), y(_y),angle(_angle), k(_k) ,cost(0){}
};


struct PathPointxy{
    PathPointxy():length(0),dis(0),obssize(0){};
    std::vector<RoadPoint> pps;
    double length;
    double dis;
    double obssize;
};

struct SLPoint {
	double s=0.0;
	double l=0.0;
};

struct FrenetFramePoint {
	double s=0.0;
	double l=0.0;
	double dl=0.0;
	double ddl=0.0;
};


struct PathPoint {
	// coordinates
	double x;
	double y;
	double z;

	// direction on the x-y plane
	double theta;
	// curvature on the x-y planning
	double kappa;
	// accumulated distance from beginning of the path
	double s;

	double l;

	// derivative of kappa w.r.t s.
	double dkappa;
	// derivative of derivative of kappa w.r.t s.
	// double ddkappa = 8;
	// The lane ID where the path point is on
	//string lane_id;
};

struct TrajectoryPoint {
	TrajectoryPoint(){};
	TrajectoryPoint(double v_,double a_):v(v_),a(a_),relative_time(0){}
	// path point
	PathPoint path_point;

	// linear velocity
	double v=0.0;  // in [m/s]
	// linear acceleration
	double a=0.0;
	// relative time from beginning of the trajectory
	double relative_time;
};

struct ReferencePoint{
	double x=0.0;
	double y=0.0;
	double s=0.0;
	double l=0.0;
	double heading =0.0;
	double kappa_ = 0.0;
	double dkappa_ = 0.0;
};

struct  ReferenceLine{
	ReferenceLine() = default;
	ReferenceLine(const ReferenceLine& reference_line) = default;
	struct SpeedLimit {
		double start_s = 0.0;
		double end_s = 0.0;
		double speed_limit = 0.0;  // unit m/s
		SpeedLimit() = default;
		SpeedLimit(double _start_s, double _end_s, double _speed_limit)
				: start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
	};
	std::vector<SpeedLimit> speed_limit_;
	std::vector<ReferencePoint> reference_points_;
	double Length;//Length of reference line
};



struct DPRoadGraphNode {

public:
    DPRoadGraphNode(){};
	double min_cost = 10000000000;
    DPRoadGraphNode(const SLPoint point_sl,
                    const DPRoadGraphNode *node_prev)
            : sl_point(point_sl), min_cost_prev_node(node_prev) {}

    // DPRoadGraphNode(const SLPoint point_sl,
    //                 const DPRoadGraphNode *node_prev,
    //                 const ComparableCost &cost)
    //         : sl_point(point_sl), min_cost_prev_node(node_prev), min_cost(cost) {}

    void UpdateCost(const DPRoadGraphNode *node_prev,
                    const quintic &curve,//@@@@@@@@@@@
                    const double &cost) {
        if (cost <= min_cost) {
            min_cost = cost;
            min_cost_prev_node = node_prev;
            min_cost_curve = curve;
        }
    }

    SLPoint sl_point;

    const DPRoadGraphNode *min_cost_prev_node = nullptr;
    // ComparableCost min_cost = {true, true, true,
    //                             std::numeric_limits<float>::infinity(),
    //                             std::numeric_limits<float>::infinity(),
    //                             std::numeric_limits<float>::infinity()};
    quintic min_cost_curve;//@@@@@
};


struct Car//车结构体
{
    RoadPoint Position;//位置为车后轮轴中心的坐标

    double rearx;//后轮的x和y
    double reary;
    double phi;  //车辆航向角

    double frontx;//前轮的x和y
    double fronty;

    double theta;   //前轮转角
    double length;//车长
    double width;//车宽

    double L;//前后轮轴距
    double RtoT;//后轮距离车尾的距离

};























#endif