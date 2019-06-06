#ifndef PARA_HPP
#define PARA_HPP


#include <cstdio>
#include <iostream>
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <vector>

class parameter{
    public:
     static double MAX_SPEED;
    static double MAX_ACCEL;
    static double MAX_CURVATURE;
    static double MAX_ROAD_WIDTH;
    static double D_ROAD_W;
    static double DT;
    static double MAXT;
    static double MINT;
    static double TARGET_SPEED;
    static double D_T_S;
    static double N_S_SAMPLE;
    static double ROBOT_RADIUS;
    static double minV;
    static double maxV;
    static double KJ;
    static double KT;
    static double KLAT;
    static double KD;
    static double KLON;
    static double Tp;

    std::vector<double> wx, wy;
    //#TODO: Make variables as const type.
    
    void read_para();
    
};


#endif