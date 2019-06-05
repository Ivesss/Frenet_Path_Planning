// #ifndef PARAMETER_HPP
// #define	PARAMETER_HPP

#include <cstdio>
#include <iostream>
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <vector>

class parameter{
    public:
    double MAX_SPEED;
    double MAX_ACCEL;
    double MAX_CURVATURE;
    double MAX_ROAD_WIDTH;
    double D_ROAD_W;
    double DT;
    double MAXT;
    double MINT;
    double TARGET_SPEED;
    double D_T_S;
    double N_S_SAMPLE;
    double ROBOT_RADIUS;
    double minV;
    double maxV;
    double KJ;
    double KT;
    double KLAT;
    double KD;
    double KLON;
    double Tp;
    std::vector<double> wx, wy;

    
    void read_para(){
        FILE *fp = fopen("parameter.json", "r");
        char buf[0XFFFF];

        //FileReadStream(FILE *fp, char *buffer, std::size_t bufferSize)
        rapidjson::FileReadStream input(fp, buf, sizeof(buf));
        rapidjson::Document document;
        document.ParseStream(input);
        fclose(fp);


        MAX_SPEED = document["max_speed"].GetDouble()/3.6; // maximum speed [m/s]
        MAX_ACCEL = document["max_acc"].GetDouble();// maximum acceleration [m/ss]
        MAX_CURVATURE = document["max_curvature"].GetDouble(); // maximum curvature [1/m]
        MAX_ROAD_WIDTH = document["road_width"].GetDouble();// maximum road width [m]
        D_ROAD_W = document["offset_gap"].GetDouble();// road width sampling length [m]
        MAXT = document["max_t"].GetDouble();// max prediction time [m]
        MINT = document["min_t"].GetDouble();// min prediction time [m]
        TARGET_SPEED = document["target_speed"].GetDouble()/3.6;// target speed [m/s]
        DT = document["delta_t"].GetDouble(); // time tick [s]
        D_T_S = document["sampling_t"].GetDouble()/3.6;// target speed sampling length [m/s]
        N_S_SAMPLE = document["sampling_t_num"].GetDouble();// sampling number of target speed
        ROBOT_RADIUS = document["veh_radius"].GetDouble();// robot radius [m]
        minV = document["stable_dist"].GetDouble();
        maxV = document["stable_time"].GetDouble();
        Tp = document["Tp"].GetDouble();//Time segments per every path, unstable when > 15
        // cost weights
        KJ = document["KJ"].GetDouble();
        KT = document["KT"].GetDouble();
        KD = document["KD"].GetDouble();
        KLAT = document["KLAT"].GetDouble();
        KLON = document["KLON"].GetDouble();

        if(document.HasMember("glo_x") && document["glo_x"].IsArray()){
            const rapidjson::Value& array = document["glo_x"];
            const rapidjson::Value& array2 = document["glo_y"];

            size_t len = array.Size();
            for(size_t i = 0; i < len; i++){
                //std::cout << "wx is [" << i << "] = " << array[i].GetDouble() << std::endl;
                wx.push_back(array[i].GetDouble());
                wy.push_back(array2[i].GetDouble());
                }
        }
    }
};

