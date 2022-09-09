#ifndef ProxyGraph_HPP
#define ProxyGraph_HPP

#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <Config.hpp>

#include <vector>
#include <map>
#include <mutex>

typedef int IHeading; //Heading Index




struct TraversableEdge{
    dynamics::data::Pose2DWithError link;
    dynamics::data::PoseByIndex target;
};

struct ProxyNode{
    //Stores the relativ position of this node to the center
    dynamics::data::Pose2D rel_pose;
};

// struct IndexNode{
//     int32_t x = 0;
//     int32_t y = 0;
//     int32_t a = 0;
//     int32_t s = 0;

//     inline bool operator==(IndexNode e) {
//     if(e.x==x && e.y == y && e.a == a && e.s== s){
//        return true;
//       }else{
//        return false;
//        }
//     }

//     inline bool operator&=(IndexNode e) {
//     if(e.x==x && e.y == y){
//        return true;
//       }else{
//        return false;
//        }
//     }

//     inline IndexNode operator+(IndexNode e) {
//         IndexNode n = {e.x + x, e.y + y, a,s};
//         return n;
//     }

//     inline IndexNode operator+(int32_t offset) {
//         IndexNode n = {x + offset, y + offset, a,s};
//         return n;
//     }

//     inline IndexNode operator-(IndexNode e) {
//         IndexNode n = {e.x - x, e.y - y,a,s};
//         return n;
//     }

//      inline IndexNode operator-(int32_t offset) {
//         IndexNode n = {x - offset, y - offset, a,s};
//         return n;
//     }

// };

// struct GINodeWS{
//     int32_t x = 0;
//     int32_t y = 0;
//     int32_t a = 0;
//     int32_t s = 0;
//     float fScore = 0.f;
//     float gScore = 100000000.f;

//     inline bool operator=(IndexNode e) {
//         x = e.x;
//         y = e.y;
//         a = e.a;
//         s = e.s;
//     }

//     inline IndexNode operator*() {
//         IndexNode n = {x,y,a,s};
//         return n;
//     }

//     inline bool operator==(IndexNode e) {
//         if(e.x==x && e.y == y && e.a == a && e.s== s){
//             return true;
//         }else{
//             return false;
//         }
//     }


// };

struct ProxyTask{
    
    int32_t txi = 0;
    int32_t tyi = 0;

    int32_t cai = 0;
    int32_t csi = 0;

    float tstep = 0.0f;
};


class ProxyGraph{
public:

ProxyGraph();


// Proxy Edges/Node

void computeProxyEdges();

ProxyNode m_proxyMap[map_size_x][map_size_y][map_size_angle][map_size_speed];
uint32_t m_proxyMapReachableSpan = 0;
uint32_t m_proxyMapCarOffset = 0;
std::map<IHeading,std::vector<TraversableEdge>> m_proxyEdgeList;

std::mutex m_proxyTaskMutex;
std::vector<ProxyTask> m_proxyTaskQueue;
bool m_terminateProxyThreads = false;
void workerThreadProxyEdges(uint32_t index);

// debug functions
void writeGraphToDisk();
void readGraphFromDisk();
void printPositions();

};


#endif