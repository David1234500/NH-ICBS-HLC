#ifndef UTIL_CONFIG_HPP
#define UTIL_CONFIG_HPP

#include <cstdint>
#include <map>
#include <string>

namespace util{

class Config{

public:
Config(){};

static Config* instance;

std::map<std::string, float> key2float_map = {{"map_size", 400.f}}; 
std::map<std::string, int64_t> key2Integer_map = {{"timestep", 100}, 
                                                  {"angle_count", 10}, 
                                                  {"velocity_count", 5},
                                                  {"vehicle_count", 2},
                                                  {"threads", 12},
                                                  {"branching_count", 3}}; 

public:
static Config* getInstance();
float getFloatByKey(std::string key);
int64_t getIntegerByKey(std::string key);

};

}
#endif //UTIL_CONFIG_HPP
  