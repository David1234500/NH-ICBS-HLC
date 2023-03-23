#ifndef MPNLOpt_HPP
#define MPNLOpt_HPP

#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <Planner/ProxyGraph.hpp>
#include <Config.hpp>

#include <vector>
#include <map>
#include <mutex>


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

    bool isIntermediate = false;
};


class MPNLOpt{
public:

MPNLOpt();

void computeMPEdges();

uint32_t m_mpMapReachableSpan = 0;
uint32_t m_mpMapCarOffset = 0;

std::map<uint32_t, double> m_solvetimes;

std::map<uint32_t, std::map<uint32_t, std::vector<MotionPrimitive>>> m_mpmap;

std::mutex m_mpTaskMutex;
std::vector<MPTask> m_mpTaskQueue;
bool m_terminateMPThreads = false;
void workerThreadMPEdges(uint32_t index);

bool  isInsideParallelogram(const Eigen::Vector2i& point, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);
std::vector<Eigen::Vector2i> integerPointsInParallelogram(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);

void writeGraphToDisk(std::string name = "mp_state_graph.json");
void loadGraphFromDisk();
void loadGraphFromDisk(std::string path);
void printPositions();

};


#endif