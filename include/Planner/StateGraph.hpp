#ifndef STATEGRAPH_HPP
#define STATEGRAPH_HPP

#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>

#include <vector>



struct StateNode{
   

};

struct Edge{



};

constexpr int map_size_x = 40;
constexpr int map_size_y = 40;
constexpr int map_size_angle = 20;

constexpr int map_size_x_cm = 400;
constexpr int map_size_y_cm = 400;

constexpr float xpc = (float)map_size_x_cm / (float)map_size_x;
constexpr float ypc = (float)map_size_y_cm / (float)map_size_y;
constexpr float api = (2.f * PI) / (float)map_size_angle;

class StateGraph{
public:

StateGraph();

StateNode m_nodes[20][20][10];
//                      from x      from  y     from a          to
Edge m_adjacency_matrix[map_size_x][map_size_y][map_size_angle][map_size_x][map_size_y][map_size_angle];

void computeSpacing();
void computeEdges();
void writeToDisk();
float indexToAngle();
dynamics::data::Pose2D gridToPosition(uint32_t x, uint32_t y, uint32_t angle);

};


#endif