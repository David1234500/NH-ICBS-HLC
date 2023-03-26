#ifndef MPCompute_HPP
#define MPCompute_HPP

#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <Config.hpp>
#include <Planner/Logger.hpp>

#include <vector>
#include <map>
#include <set>
#include <mutex>
#include <cmath>
#include <iostream>
#include <thread>
#include <fstream>



struct MotionPrimitive{
    dynamics::data::Pose2DWithError link;
    dynamics::data::PoseByIndex target;
};

struct MPNode{
    //Stores the relativ position of this node to the center
    dynamics::data::Pose2D rel_pose;
};

struct MPTask{
    
    int32_t txi = 0;
    int32_t tyi = 0;
    int32_t tsi = 0;
    int32_t cai = 0;
    int32_t csi = 0;
    
    float tstep = 0.0f;
};


class MPCompute{
public:

MPCompute();
~MPCompute();

void computeMPEdges();

std::vector<Eigen::Vector2i> computeConicIntegerHull(float heading);
bool  isInsideParallelogram(const Eigen::Vector2i& point, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);
std::vector<Eigen::Vector2i> integerPointsInParallelogram(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);

int32_t m_mpMapReachableNodeCount = 0;
std::map<uint32_t, std::map<uint32_t, std::vector<MotionPrimitive>>> m_mpmap;

std::mutex m_mpTaskMutex;
std::vector<MPTask> m_mpTaskQueue;
bool m_terminateMPThreads = false;
virtual void workerThreadMPEdges(uint32_t index) = 0;
void addSymetricMPs(double st1, double st2, double vel1, double vel2, dynamics::data::PoseByIndex tp_bi, int32_t h_sw_beg, int32_t tht_cai, int32_t tht_csi, int32_t tht_tsi); 

void writeGraphToDisk(std::string name = "mp_state_graph.json");
void loadGraphFromDisk();
void loadGraphFromDisk(std::string path);
void printPositions();
void writeIntegerHullToDisc(std::vector<Eigen::Vector2i> hull, std::string name = "mp_integer_hull.json");

};


#endif