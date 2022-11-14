#ifndef DirectedSearchProxy_HPP
#define DirectedSearchProxy_HPP

#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <Config.hpp>

#include <vector>
#include <map>
#include <mutex>
#include <memory>

struct MotionPrimitiv{
    dynamics::data::Pose2DWithMotionData link;
    dynamics::data::PoseByIndex target;
    bool is_zero_connection = false;    
};



struct searchJob{
    int m_config_map_size_angle;
    int32_t x; 
    int32_t y; 
    int32_t a; 
    double api; 
    double base_node_distance; 
    dynamics::data::Pose2D target_pose;                     
    std::map<uint32_t, std::map<uint32_t, std::shared_ptr<std::vector<dynamics::data::Pose2DWithMotionData>>>> reachable_set_map;
};

class DirectedSearchProxy{

public:

DirectedSearchProxy();


// Proxy Edges/Node
void worker(searchJob job);
void computeProxyEdges();
void dispatch();

std::vector<searchJob> job_queue;
std::mutex job_mutex;

double m_config_baseVelocityFactor = 0.5;
double m_base_node_distance = 8.f;

std::vector<double> m_config_speedsFactor = {1.f, 0.f, -1.f}; 
int m_config_map_size_zero_velocity_level = 1;
int m_config_map_size_speed = 3;


int m_config_base_node_distance = 10.f;

int m_config_map_size_x_cm = 600;
int m_config_map_size_y_cm = 600;
int m_config_map_size_angle = 8;

double m_config_ts_base = 450.f;
double m_config_ts_min = 400.f;
double m_config_ts_max = 700.f;

double  m_config_state_change_fit_quality_angle = 0.3f;
double  m_config_state_change_fit_quality_position =  0.025f;

int m_comp_map_extent = 0;
int m_comp_map_size_x = 0;
int m_comp_map_size_y = 0;

std::mutex motion_primitive_map_mutex;
std::map<int32_t, std::map<int32_t, std::vector<MotionPrimitiv>>> motion_primitive_map;

// debug functions
void writeGraphToDisk(std::string name = "proxy_state_graph");
void loadGraphFromDisk();
void loadGraphFromDisk(std::string path);

};


#endif