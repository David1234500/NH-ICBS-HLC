#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <util/Pose.hpp>
#include <util/Config.hpp>
#include <util/State.hpp>
#include <util/SearchTree.hpp>

#include <DynamicsModel/SingleTrackModel.hpp>
#include <Planner/StateGraph.hpp>

#include <queue>
#include <map>

class CBSPlanner 
{
public:
    CBSPlanner(){

    }

    void astar(IndexNode start, IndexNode target);
    void insertByOrder(IndexNode node, std::vector<IndexNode>& openSet, std::map<IndexNode,float>& fScore);


};


#endif