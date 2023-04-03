#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <util/Pose.hpp>

#include <DynamicsModel/SingleTrackModel.hpp>
#include <MPCompute/MPBruteforce.hpp>
#include <Planner/PlannerBase.hpp>
#include <unordered_set>
#include <unordered_map>

#include <Config.hpp>
#include <queue>
#include <map>
#include <thread>
#include <memory>
#include <tuple>



class CBSPlanner : public PlannerBase{
public:
    CBSPlanner(){}
    ~CBSPlanner(){
    }
    constraint_node cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions, bool relax = false, bool disable_collisions = false);

    ReachabilityResult checkForReachability(); //TODO Remove
    
    void writeMultiplePathsToDisk(constraint_node cnode, std::string name);
    void writeConstraintNodeToDisk(constraint_node cnode, std::string name);  
};


#endif