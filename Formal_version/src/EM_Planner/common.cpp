
#include "../../include/EM_Planner/common.hpp"








// struct SLPoint {
// 	double s=0.0;
// 	double l=0.0;
// };



// struct PathPoint {
// 	// coordinates
// 	double x;
// 	double y;
// 	double z;

// 	// direction on the x-y plane
// 	double theta;
// 	// curvature on the x-y planning
// 	double kappa;
// 	// accumulated distance from beginning of the path
// 	double s;

// 	double l;

// 	// derivative of kappa w.r.t s.
// 	double dkappa;
// 	// derivative of derivative of kappa w.r.t s.
// 	// double ddkappa = 8;
// 	// The lane ID where the path point is on
// 	//string lane_id;
// };

// struct TrajectoryPoint {
// 	TrajectoryPoint(){};
// 	TrajectoryPoint(double v_,double a_):v(v_),a(a_),relative_time(0){}
// 	// path point
// 	PathPoint path_point;

// 	// linear velocity
// 	double v=0.0;  // in [m/s]
// 	// linear acceleration
// 	double a=0.0;
// 	// relative time from beginning of the trajectory
// 	double relative_time;
// };

// struct ReferencePoint{
// 	double x=0.0;
// 	double y=0.0;
// 	double s=0.0;
// 	double l=0.0;
// 	double heading =0.0;
// 	double kappa_ = 0.0;
// 	double dkappa_ = 0.0;
// };

// struct  ReferenceLine{
// 	ReferenceLine() = default;
// 	ReferenceLine(const ReferenceLine& reference_line) = default;
// 	struct SpeedLimit {
// 		double start_s = 0.0;
// 		double end_s = 0.0;
// 		double speed_limit = 0.0;  // unit m/s
// 		SpeedLimit() = default;
// 		SpeedLimit(double _start_s, double _end_s, double _speed_limit)
// 				: start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
// 	};
// 	std::vector<SpeedLimit> speed_limit_;
// 	std::vector<ReferencePoint> reference_points_;
// 	double Length;//参考路径长度

// };