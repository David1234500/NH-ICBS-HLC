#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <util/Pose.hpp>

#include <DynamicsModel/SingleTrackModel.hpp>
#include <Planner/ProxyGraph.hpp>
#include <Planner/StateGraph.hpp>
#include <Config.hpp>
#include <queue>
#include <map>



class CBSPlanner 
{
public:
    CBSPlanner(){

    }


    dynamics::data::PoseByIndex toGlobalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex relative);
    dynamics::data::Pose2D indexToPose(dynamics::data::PoseByIndex);
    dynamics::data::PoseByIndex toLocalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex global);

    void astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target);
    int32_t binarySearch(dynamics::data::PoseByIndex node, std::vector<dynamics::data::PoseByIndex>& openSet,  std::map<dynamics::data::PoseByIndex, float>& fScoreMap);

private:
    ProxyGraph m_proxGraph;
    StateGraph m_stateGraph;

};


#endif