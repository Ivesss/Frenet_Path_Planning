#include "../../include/EM_Planner/conversion.hpp"
#include <cmath>


void ConverCartesionFrenet::frenet_to_cartesian(ReferencePoint refer, double s,double d,double &x,double &y){


   if(std::abs(refer.s - s) < 1.0e-6){
           std::cout<< "The reference point s and s_condition[0] don't match";
   }
    const double cos_theta_r = std::cos(refer.heading);
    const double sin_theta_r = std::sin(refer.heading);

    x = refer.x - sin_theta_r * d;
    y = refer.y + cos_theta_r * d;


}


void ConverCartesionFrenet::frenet_to_cartesian(ReferencePoint refer, double s, double ds, double dds,
        double l, double dl, double ddl, double &x,double &y, double &theta, double &kappa,
        double &v, double &a){

   if(std::abs(refer.s - s) < 1.0e-6){
           std::cout<< "The reference point s and s_condition[0] don't match";
   }

    const double cos_theta_r = std::cos(refer.heading);
    const double sin_theta_r = std::sin(refer.heading);

    x = refer.x - sin_theta_r * l;
    y = refer.y + cos_theta_r * l;

    const double one_minus_kappa_r_d = 1 - refer.kappa_ * l;

    const double tan_delta_theta = dl / one_minus_kappa_r_d;
    const double delta_theta = std::atan2(dl, one_minus_kappa_r_d);
    const double cos_delta_theta = std::cos(delta_theta);

    theta = (delta_theta + refer.heading);

    while(theta>2.0*PI){
        theta=theta-2.0*PI;
    } 
    while(theta<0){
        theta=theta+2.0*PI;
    }



    const double kappa_r_d_prime =
            refer.dkappa_ * l + refer.kappa_ * dl;
    kappa = (((ddl + kappa_r_d_prime * tan_delta_theta) *
                   cos_delta_theta * cos_delta_theta) /
                  (one_minus_kappa_r_d) +
                  refer.kappa_) *
                 cos_delta_theta / (one_minus_kappa_r_d);

    const double d_dot = dl * ds;
    v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
                       ds * ds +
                       d_dot * d_dot);

    const double delta_theta_prime =
            one_minus_kappa_r_d / cos_delta_theta * (kappa) - refer.kappa_;

    a = dds * one_minus_kappa_r_d / cos_delta_theta +
             ds * ds / cos_delta_theta *
             (d_dot * delta_theta_prime - kappa_r_d_prime);  



}




void ConverCartesionFrenet::cartesian_to_frenet(
        const double rs, //弧长
        const double rx, const double ry, const double rtheta,//参考点笛卡尔坐标下的(x,y,heading)
        const double rkappa, const double rdkappa, //参考点的k, dk
        const double x, const double y,//待转换点的(x,y)
        const double v, const double a, const double theta, const double kappa,//待转换点的其他信息
        //输出  s 三阶  d 三阶
        std::array<double, 3>* const ptr_s_condition,
        std::array<double, 3>* const ptr_d_condition)
{
    const double dx = x - rx;
    const double dy = y - ry;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);
//判断角度差 判断距离的正负 正为左，负为右
    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;//法向（模）
    ptr_d_condition->at(0) =
            std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);//前面一个参数的值，后面一个参数的正负
            //sqrt(dx * dx + dy * dy), honestly, its just c=sqrt(a^2 + b^2) for the origin lateral point.
    const double delta_theta = theta - rtheta;//角度差
    const double tan_delta_theta = std::tan(delta_theta);
    const double cos_delta_theta = std::cos(delta_theta);

    const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
    ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;

    const double kappa_r_d_prime =
            rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);

    ptr_d_condition->at(2) =
            -kappa_r_d_prime * tan_delta_theta +
            one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
            (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

    ptr_s_condition->at(0) = rs;

    ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

    const double delta_theta_prime =
            one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
    ptr_s_condition->at(2) =
            (a * cos_delta_theta -
             ptr_s_condition->at(1) * ptr_s_condition->at(1) *
             (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
            one_minus_kappa_r_d;
    return;
}





double ConverCartesionFrenet::Distance(int xg, int yg, int xc, int yc)
{
return sqrt((xg-xc)*(xg-xc)+(yg-yc)*(yg-yc));
}

//get cloest point, output (nearRefer.s,nearRefer.x,nearRefer.y,nearRefer.heading,nearRefer.kappa_,nearRefer.dkappa_,
            //pointxy.x,pointxy.y,v,a,pointxy.angle,kappa,&ptr_s_condition,&ptr_d_condition); // those are addition initial conditions
void ConverCartesionFrenet::XY2SL(double x_c, double y_c, vector<double> rx, vector<double> ry, vector<double> ryaw, 
ReferenceLine &referenceLine){
    // if(referxy.pps.empty()) {
    //     cout<<("Reference lane is empty!")<<endl;
    //     return false;
    // }
    double sum_s =0.0;
    double dis =10000;
    int index = -1;
    for (int i = 0; i < rx.size(); ++i) // type pps: x y angle k cost
    {
        double d =Distance(x_c, y_c, rx[i], ry[i]);

        double angle3 = atan2(ry[i]-y_c, rx[i]-x_c);
        double anglediff = cos(abs(angle3-70)); //init_heading set as 70 degree.
        if(d<dis&&anglediff>0){
            dis = d;
            index = i;
        }
        // if(i>20)
        //     break;   //@@@@@@@@@@@@@@@@@@@@@@2 why added this? if only looking for closest point
    }

    ReferencePoint lastp;// =referxy.pps[0];
    lastp.x=rx[index];
    lastp.y=ry[index];
    lastp.heading =ryaw[index];
    lastp.s =0.0;//origin of SL coordinate must be (0,0)
    lastp.l=0.0;//origin of SL coordinate must be (0,0)
    lastp.kappa_=0.0; 
    lastp.dkappa_=0.0;
    referenceLine.reference_points_.push_back(lastp);
    double k,dk,l;
    for(int rexy=index+1;rexy<rx.size();++rexy)
    {
        ReferencePoint reSL;
        //double dis =BasicStruct::Distance(referxy.pps[rexy],lastp);

		

        Clothoid::buildClothoid(lastp.x,lastp.y,lastp.heading,rx[rexy],ry[rexy],ryaw[rexy],k,dk,l);
		



        sum_s+=l; // curve length? in SL? need to check!@@@@@@@@@@@@      //弧长, same as my spline2D. Try to change it later on
		
//        if(sum_s>10)
//            break;
        reSL.s =sum_s; 
        reSL.l =0.0;
        reSL.x =rx[rexy];
        reSL.y =ry[rexy];
        reSL.heading = ryaw[rexy];
        reSL.kappa_ =k;//referxy.pps[rexy].k;//(referxy.pps[rexy].angle-lastp.heading)/dis;
        reSL.dkappa_ =dk;//(reSL.kappa_-lastp.kappa_)/dis;
        lastp =reSL;//ferxy.pps[rexy];
        referenceLine.reference_points_.push_back(reSL);
    }
    referenceLine.Length =sum_s;
    // if(referenceLine.reference_points_.empty())
    //     return false;
    // return true;
}


TrajectoryPoint ConverCartesionFrenet::initial_Point_trans(ReferencePoint nearRefer, double x, double y,double angle, double v,
                                                   double a, double kappa, std::array<double, 3> &ptr_s_condition, std::array<double, 3> &ptr_d_condition){
    //输出  s 三阶  d 三阶



    ConverCartesionFrenet::cartesian_to_frenet(nearRefer.s,nearRefer.x,nearRefer.y,nearRefer.heading,nearRefer.kappa_,nearRefer.dkappa_,
            x,y,v,a,angle,kappa,&ptr_s_condition,&ptr_d_condition);
    TrajectoryPoint start;
    start.path_point.x=x; //CURRENT POSITION
    start.path_point.y=y;//CURRENT POSITION
    start.path_point.theta=angle;//CURRENT POSITION
    start.path_point.s=ptr_s_condition.front(); // s = 0 for the initial point
    start.path_point.l=ptr_d_condition.front();
    start.v = v;
    start.a = a;
    return start;
}


