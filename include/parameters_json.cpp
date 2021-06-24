#include "parameters_json.h"

Params::Params(){

}

Params::~Params(){

}

void Params::read_data(std::string & json_dir) {
    std::ifstream stream;
    stream.open(json_dir);

    Json::Value root;
    stream >> root;

    Json::Value thermal_params = root["thermal_parms"];
    Json::ValueIterator it = thermal_params.begin();
    it = thermal_params.begin();

    while(it != thermal_params.end()) {
        if(it->isObject()) {
            std::string name = (*it)["name"].asString();
            if(name == "min_temp") {
                thermalParam.min_temp = value2float(it);
            }else if(name=="max_temp") {
                thermalParam.max_temp = value2float(it);
            }
        }
        ++it;
    }
    thermalParam.PrintStuff();
}

float Params::value2float(Json::ValueIterator & it) {
    return std::stof((*it)["value"].asString());
}
int Params::value2int(Json::ValueIterator & it) {
    return std::stoi((*it)["value"].asString());
}