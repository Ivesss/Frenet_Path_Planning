
#include "parameter_read.hpp"

parameter para;

double parameter::MAX_SPEED = 0;
double parameter::MAX_ACCEL = 0;
double parameter::MAX_CURVATURE = 0;
double parameter::MAX_ROAD_WIDTH = 0;
double parameter::D_ROAD_W = 0;
double parameter::DT = 0;
double parameter::MAXT = 0;
double parameter::MINT = 0;
double parameter::TARGET_SPEED = 0;
double parameter::D_T_S = 0;
double parameter::N_S_SAMPLE = 0;
double parameter::ROBOT_RADIUS = 0;
double parameter::minV = 0;
double parameter::maxV = 0;
double parameter::KJ = 0;
double parameter::KT = 0;
double parameter::KLAT = 0;
double parameter::KD = 0;
double parameter::KLON = 0;
double parameter::Tp = 0;

void parameter::read_para(){
        FILE *fp = fopen("parameter.json", "r");
        char buf[0XFFFF];
        
        //FileReadStream(FILE *fp, char *buffer, std::size_t bufferSize)
        rapidjson::FileReadStream input(fp, buf, sizeof(buf));
        rapidjson::Document document;
        document.ParseStream(input);
        fclose(fp);

        parameter pa;

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
        minV = TARGET_SPEED - D_T_S*N_S_SAMPLE;
        maxV = TARGET_SPEED + D_T_S*N_S_SAMPLE;
        
        Tp = document["Tp"].GetDouble();//Time segments per every path, unstable when > 15

        // cost weights
        parameter::KJ = document["KJ"].GetDouble();
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
