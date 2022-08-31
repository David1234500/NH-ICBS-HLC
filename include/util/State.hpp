#ifndef UTIL_STATE_HPP
#define UTIL_STATE_HPP
 
#include <util/Pose.hpp>
#include <util/Config.hpp>
#include <iostream>
#include <random> 

namespace dynamics{

using namespace data;

class VehicleState{
public:
    Pose2D pose;
    
    Pose2D target;
    
    Pose2D start;    

    VehicleState(){};

    void print(){
        std::cout << std::to_string(pose.pos[0]) << ":" << std::to_string(pose.pos[1]) << std::endl;
        std::cout << std::to_string(pose.vel) << std::endl;
        std::cout << std::to_string(pose.h) << std::endl;
    }

    void init_random(){
        
        util::Config* config = util::Config::getInstance();
        
        std::random_device rd;     // only used once to initialise (seed) engine
        std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<float> uni(0,config->getFloatByKey("map_size")); // guaranteed unbiased

        auto random_integer = uni(rng);

        float rx = (float)  uni(rng);;
        float ry = (float)  uni(rng);;
        pose.vel = 0;
        std::cout << "start " << rx << ":" << ry << std::endl;

        pose.h = (float) (rand()) / ((float) (RAND_MAX/6.3));
        pose.pos = {rx,ry};
        pose.vel = 0;

        start.pos = {rx,ry};
        start.h = (float) (rand()) / ((float) (RAND_MAX/6.3));
        pose.vel = 0;

        rx = (float)  uni(rng);;
        ry = (float)  uni(rng);;

        std::cout << "target " << rx << ":" << ry << std::endl;

        target.pos = {rx,ry};
        target.h = (float) (rand()) / ((float) (RAND_MAX/6.3));
        target.vel = 0;
    }
};




}
#endif //UTIL_STATE_HPP