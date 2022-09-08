#ifndef STATEGRAPH_HPP
#define STATEGRAPH_HPP

#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>

#include <vector>
#include <map>
#include <mutex>

typedef int IHeading; //Heading Index


struct StateNode{

};

struct TraversableEdge{
    dynamics::data::Pose2DWithError link;
    dynamics::data::PoseByIndex target;
};

struct ProxyNode{
    //Stores the relativ position of this node to the center
    dynamics::data::Pose2D rel_pose;
};

struct IndexNode{
    int32_t x = 0;
    int32_t y = 0;
    int32_t a = 0;
    int32_t s = 0;

    inline bool operator==(IndexNode e) {
    if(e.x==x && e.y == y && e.a == a && e.s== s){
       return true;
      }else{
       return false;
       }
    }

    inline bool operator&=(IndexNode e) {
    if(e.x==x && e.y == y){
       return true;
      }else{
       return false;
       }
    }

    inline IndexNode operator+(IndexNode e) {
        IndexNode n = {e.x + x, e.y + y, a, s};
        return n;
    }

};

struct ProxyTask{
    
    int32_t txi = 0;
    int32_t tyi = 0;

    int32_t cai = 0;
    int32_t csi = 0;

    float tstep = 0.0f;
};

constexpr int map_size_x = 50;
constexpr int map_size_y = 50;
constexpr int map_size_angle = 16;
constexpr int map_size_speed = 5;

constexpr int map_size_x_cm = 400;
constexpr int map_size_y_cm = 400;

constexpr float xpc = (float)map_size_x_cm / (float)map_size_x;
constexpr float ypc = (float)map_size_y_cm / (float)map_size_y;
constexpr float api = (2.f * PI) / (float)map_size_angle;

constexpr float m_speedsFactor[5] = {-0.5f, -0.25f, 0.25f, 0.5f, 1.0f};

class StateGraph{
public:

StateGraph();


// Proxy Edges/Node

void computeProxyEdges();

ProxyNode m_proxyMap[map_size_x][map_size_y][map_size_angle][map_size_speed];
uint32_t m_proxyMapReachableSpan = 0;
std::map<IHeading,std::vector<TraversableEdge>> m_proxyEdgeList;

std::mutex m_proxyTaskMutex;
std::vector<ProxyTask> m_proxyTaskQueue;
bool m_terminateProxyThreads = false;
void workerThreadProxyEdges(uint32_t index);

// debug functions
void printEdges();
void printPositions();

// Convenience Operator Overrides
// inline ProxyNode operator[](IndexNode a) {
//     if(a.x==x && a.y== y)
//        return true;
//       else
//        return false;
// }


};


#endif