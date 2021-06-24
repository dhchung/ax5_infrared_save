#pragma once

#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <string>
#include <math.h>

struct ThermalParameters{
    float min_temp = 0.0;
    float max_temp = 0.0;

    void PrintStuff(){
        std::cout<<"Thermal Parmeters"<<std::endl;
        std::cout<<"min_temp: "<<min_temp<<std::endl;
        std::cout<<"max_temp: "<<max_temp<<std::endl;
    }

};

class Params{
public:
    Params();
    ~Params();
    void read_data(std::string & json_dir);
    float value2float(Json::ValueIterator & it);
    int value2int(Json::ValueIterator & it);

    ThermalParameters thermalParam;
};