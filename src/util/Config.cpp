#include <util/Config.hpp>

using namespace util;

Config* Config::instance = nullptr;

Config* Config::getInstance(){

    if(instance == nullptr){
        instance = new Config();
    }

    return instance;
}

float Config::getFloatByKey(std::string key){
    return key2float_map[key];
}

int64_t Config::getIntegerByKey(std::string key){
     return key2Integer_map[key];
}
