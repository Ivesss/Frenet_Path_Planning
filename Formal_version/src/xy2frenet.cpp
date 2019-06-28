// #include <cmath>
// #include <iostream>
// #include <array>
// #include <vector>
// #include "../include/parameter_read.hpp"

// using namespace std;

XY2SL(){


/*
1. Find closest point of g_current to reference line

2. From that point on reference line, iterate till the end of reference line

3. Every iteration, store information of EVERY way point of the reference line

referenceLine.reference_points_.push_back(reSL);

4. information includes: (x,y,s,l=0,heading,kappa,dkappa)
reSL.s, reSL.y......

5. Purpose: construct initial SL frame according to reference line.......



*/

}
bool beginPlan(){

referLane = wholeReferLane[targetIndex]; //true reference line, put it into reference_line_info
    common::ReferenceLine reference_line_info;//dummy reference line, to store information



    if (XY2SL(referLane,reference_line_info)){
        auto init = CartesianFrenetConverter::initial_Point_trans(reference_line_info.reference_points_.front(), g_currentLocation, velocity, 0, 0);
        //init: s,s',s'', d,d',d'' of initial point on the reference line
















    }










}

bool replan(){
    /*
    bool flag_pathempty = 0;		// 规划路径为空触发重规划标志
    bool flag_pathcollision = 0;	// 规划路径有障碍物触发重规划标志
    bool flag_traveldis = 0;		// 行驶距离触发的重规划标志（主动触发策略）
    bool flag_cartopath = 0;		// 车偏离规划路径触发的重规划标志
    ....
    */
}

void process(){
    g_currentLocation
    TravelingDis = BasicStruct::Distance(lastLocation,g_currentLocation); //dis between last pt and current 
    // planning is an trigger event, triger by replan()

    if (replan()){
        flag_PlanSucceed = beginPlan();
        lastLocation = g_currentLocation;
    }

}


int main(){

process()



}








































// //get distance to the point x,y

// double Distance(int xg, int yg, int xc, int yc)
// {
// return sqrt((xg-xc)*(xg-xc)+(yg-yc)*(yg-yc));
// }


// //get cloest point
// bool XY2SL(int x, int y,vector<double> xr, vector<double> yr){
//     if(xr.empty()) {
//         cout<<("Reference lane is empty!")<<endl;
//         return false;
//     }
//     double sum_s =0.0;
//     double dis =10000;
//     int index = -1;
//     for (int i = 0; i < xr.size(); ++i) // type pps: x y angle k cost
//     {
//         double d =Distance(x,y,xr[i],yr[i]);
//         double angle3 = atan2(yr[i]-y,xr[i]-x);
//         //double anglediff = cos(abs(angle3-g_currentLocation.angle));
//         // if(d<dis&&anglediff>0){
//         //     dis = d;
//         //     index = i;
//         // }
//         // if(i>20)
//         //     break;
//         if(d<dis){
//             dis = d;
//             index = i;
//         }
//         if(i>20)
//             break;
//     }
// //     common::ReferencePoint lastp;// =referxy.pps[0];
// //     lastp.x=referxy.pps[index].x;
// //     lastp.y=referxy.pps[index].y;
// //     lastp.heading =referxy.pps[index].angle;
// //     lastp.s =0.0;
// //     lastp.l=0.0;
// //     lastp.kappa_=0.0;
// //     lastp.dkappa_=0.0;
// //     referenceLine.reference_points_.push_back(lastp);
//     double near_x = xr[index], near_y = yr[index];
//     cout<<"nearest x y: "<<near_x<<"  "<<near_y<<endl;

//     double k,dk,l;
//     for(int rexy=index+1;rexy<xr.size();++rexy)
//     {
//         common::ReferencePoint reSL;
//         //double dis =BasicStruct::Distance(referxy.pps[rexy],lastp);

//         Clothoid::buildClothoid(lastp.x,lastp.y,lastp.heading,referxy.pps[rexy].x,referxy.pps[rexy].y,referxy.pps[rexy].angle,k,dk,l);
//         //above curve edit k, dk, l
//         sum_s+=l;
// //        if(sum_s>10)
// //            break;
//         reSL.s =sum_s;
//         reSL.l =0.0;
//         reSL.x =referxy.pps[rexy].x;
//         reSL.y =referxy.pps[rexy].y;
//         reSL.heading = referxy.pps[rexy].angle;
//         reSL.kappa_ =k;//referxy.pps[rexy].k;//(referxy.pps[rexy].angle-lastp.heading)/dis;
//         reSL.dkappa_ =dk;//(reSL.kappa_-lastp.kappa_)/dis;
//         lastp =reSL;//ferxy.pps[rexy];
//         referenceLine.reference_points_.push_back(reSL);
//     }
//     referenceLine.Length =sum_s;
//     if(referenceLine.reference_points_.empty())
//         return false;
//     return true;
// }











// int main(){

// vector<double> wx(0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0);
// vector<double> wy(0.0, -4.0, 1.0, 6.5, 8.0, 10.0, 6.0);

// int x= 9.559, y = 0.00878754
// //corresponding s,l = 10, 4
// //int x= 9.88706, y = 0.490637
// //corresponding s,l = 10.6978 4.49184






// }












