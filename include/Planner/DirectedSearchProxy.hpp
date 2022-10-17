#ifndef DirectedSearchProxy_HPP
#define DirectedSearchProxy_HPP

#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <Config.hpp>

#include <vector>
#include <map>
#include <mutex>


struct MotionPrimitiv{
    dynamics::data::Pose2DWithMotionData link;
    dynamics::data::PoseByIndex target;

    
};

struct LatticeNode{
    //Stores the relativ position of this node to the center
    dynamics::data::Pose2D rel_pose;
};

class DirectedSearchProxy{
public:

DirectedSearchProxy();


// Proxy Edges/Node

void computeProxyEdges();

float m_config_baseVelocityFactor = 0.25;

float m_config_speedsFactor[3] = {0.f, 1.f, 2.f}; 
int m_config_map_size_speed = 3;

int m_config_map_size_x_cm = 450;
int m_config_map_size_y_cm = 400;
int m_config_map_size_angle = 8;

float m_config_ts_min = 200.f;
float m_config_ts_max = 500.f;

std::map<uint32_t, std::map<uint32_t, std::vector<MotionPrimitiv>>> motion_primitive_map;

// debug functions
void writeGraphToDisk();
void loadGraphFromDisk();
void loadGraphFromDisk(std::string path);

};


#endif