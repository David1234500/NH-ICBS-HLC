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
        
        // util::Config* config = util::Config::getInstance();
        
        // std::random_device rd;     // only used once to initialise (seed) engine
        // std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
        // std::uniform_int_distribution<double> uni(0,config->getdoubleByKey("map_size")); // guaranteed unbiased

        // auto random_integer = uni(rng);

        // double rx = (double)  uni(rng);;
        // double ry = (double)  uni(rng);;
        // pose.vel = 0;
        // std::cout << "start " << rx << ":" << ry << std::endl;

        // pose.h = (double) (rand()) / ((double) (RAND_MAX/6.3));
        // pose.pos = {rx,ry};
        // pose.vel = 0;

        // start.pos = {rx,ry};
        // start.h = (double) (rand()) / ((double) (RAND_MAX/6.3));
        // pose.vel = 0;

        // rx = (double)  uni(rng);;
        // ry = (double)  uni(rng);;

        // std::cout << "target " << rx << ":" << ry << std::endl;

        // target.pos = {rx,ry};
        // target.h = (double) (rand()) / ((double) (RAND_MAX/6.3));
        // target.vel = 0;
    }
};




}
#endif //UTIL_STATE_HPP