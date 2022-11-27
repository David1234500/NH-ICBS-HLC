#ifndef ProxyGraph_HPP
#define ProxyGraph_HPP

#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <Config.hpp>

#include <vector>
#include <map>
#include <mutex>


struct TraversableEdge{
    dynamics::data::Pose2DWithError link;
    dynamics::data::PoseByIndex target;
};

struct ProxyNode{
    //Stores the relativ position of this node to the center
    dynamics::data::Pose2D rel_pose;
};

struct ProxyTask{
    
    int32_t txi = 0;
    int32_t tyi = 0;
    int32_t tsi = 0;
    
    int32_t cai = 0;
    int32_t csi = 0;
    
    float tstep = 0.0f;

    bool isIntermediate = false;
};


class ProxyGraph{
public:

ProxyGraph();

void computeProxyEdges();


uint32_t m_proxyMapReachableSpan = 0;
uint32_t m_proxyMapCarOffset = 0;
std::map<uint32_t, std::map<uint32_t, std::vector<TraversableEdge>>> m_proxyEdgeList;

std::mutex m_proxyTaskMutex;
std::vector<ProxyTask> m_proxyTaskQueue;
bool m_terminateProxyThreads = false;
void workerThreadProxyEdges(uint32_t index);

// debug functions
void writeGraphToDisk(std::string name = "proxy_state_graph.json");
void loadGraphFromDisk();
void loadGraphFromDisk(std::string path);
void printPositions();

};


#endif