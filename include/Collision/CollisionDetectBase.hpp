#ifndef COLLISION_BASE_HPP
#define COLLISION_BASE_HPP

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <util/Pose.hpp>
#include <MPCompute/MPCompute.hpp>


struct CollisionInfo {
    bool collision_occurred = false;
    size_t index1 = 0;
    size_t index2 = 0;
    uint32_t car_index_1 = 0;
    uint32_t car_index_2 = 0;
    int32_t collision_with_veh_at_track_end = 0;
    dynamics::data::Pose2WithTime pose1;
    dynamics::data::Pose2WithTime pose2;
    dynamics::data::PoseByIndex pose1bi;
    dynamics::data::PoseByIndex pose2bi;
    int32_t car_rem_1 = 0;
    int32_t car_rem_2 = 0;
    bool one_feasible = false;
    bool two_feasible = false;
};

class CollisionDetectBase{
public:
    CollisionDetectBase(){}
    virtual CollisionInfo checkForCollision(const std::vector<dynamics::data::Pose2WithTime>& track1,
                                   const std::vector<dynamics::data::Pose2WithTime>& track2) = 0;
    // static CollisionInfo checkForCollision(const MotionPrimitive p1, const dynamics::data::Pose2D pbi1,
    //                                         const MotionPrimitive p2, const dynamics::data::Pose2D pbi2) {};

    static void print_collision_info(const CollisionInfo& ci) {
        std::cout << "Collision occurred: " << std::boolalpha << ci.collision_occurred << std::endl;
        std::cout << "Index 1: " << ci.index1 << std::endl;
        std::cout << "Index 2: " << ci.index2 << std::endl;
        std::cout << "Car index 1: " << ci.car_index_1 << std::endl;
        std::cout << "Car index 2: " << ci.car_index_2 << std::endl;
        
        const dynamics::data::Pose2WithTime& p1 = ci.pose1;
        const dynamics::data::Pose2WithTime& p2 = ci.pose2;
        
        std::cout << "Pose 1:" << std::endl;
        std::cout << "  Position: (" << p1.pos[0] << ", " << p1.pos[1] << ")" << std::endl;
        std::cout << "  Heading: " << p1.h << std::endl;
        std::cout << "  Velocity: " << p1.vel << std::endl;
        std::cout << "  Time (ms): " << p1.time_ms << std::endl;
        
        const dynamics::data::PoseByIndex& p1bi = ci.pose1bi;
        std::cout << "  Pose by index 1:" << std::endl;
        std::cout << "    x: " << p1bi.x << std::endl;
        std::cout << "    y: " << p1bi.y << std::endl;
        std::cout << "    a: " << p1bi.a << std::endl;
        std::cout << "    s: " << p1bi.s << std::endl;
        
        std::cout << "Pose 2:" << std::endl;
        std::cout << "  Position: (" << p2.pos[0] << ", " << p2.pos[1] << ")" << std::endl;
        std::cout << "  Heading: " << p2.h << std::endl;
        std::cout << "  Velocity: " << p2.vel << std::endl;
        std::cout << "  Time (ms): " << p2.time_ms << std::endl;
        
        const dynamics::data::PoseByIndex& p2bi = ci.pose2bi;
        std::cout << "  Pose by index 2:" << std::endl;
        std::cout << "    x: " << p2bi.x << std::endl;
        std::cout << "    y: " << p2bi.y << std::endl;
        std::cout << "    a: " << p2bi.a << std::endl;
        std::cout << "    s: " << p2bi.s << std::endl;

        std::cout << " collision_with_veh_at_track_end: " << ci.collision_with_veh_at_track_end << std::endl;
    }


};

#endif