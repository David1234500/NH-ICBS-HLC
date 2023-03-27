#ifndef COLLISION_BASE_HPP
#define COLLISION_BASE_HPP

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <nlopt.hpp>
#include <util/Pose.hpp>


struct CollisionInfo {
    bool collision_occurred = false;
    size_t index1 = 0;
    size_t index2 = 0;
    dynamics::data::Pose2WithTime pose1;
    dynamics::data::Pose2WithTime pose2;
};

class CollisionDetectBase{
public:
    CollisionDetectBase(){}
    virtual CollisionInfo checkForCollision(const std::vector<dynamics::data::Pose2WithTime>& track1,
                                   const std::vector<dynamics::data::Pose2WithTime>& track2) = 0;

};

#endif