#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <util/Pose.hpp>

#include <DynamicsModel/SingleTrackModel.hpp>
#include <MPCompute/MPBruteforce.hpp>
#include <Planner/PlannerBase.hpp>
#include <util/Profiler.hpp>

#include <unordered_set>
#include <unordered_map>

#include <Config.hpp>
#include <queue>
#include <map>
#include <thread>
#include <memory>
#include <tuple>
#include <thread>
#include <vector>
#include <condition_variable>


class CBSPlanner : public PlannerBase{
public:
    CBSPlanner(){}
    ~CBSPlanner(){
    }
    constraint_node cbs_single(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions, bool relax, bool disable_collisions);
    constraint_node cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions, bool relax = false, bool disable_collisions = false);
    void cbsWorker(int32_t id, std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions, bool disable_collisions);

    std::mutex m_openSetMutex;
    std::priority_queue<constraint_node> m_openSet;
    

    std::vector<std::thread> m_cbsWorkers;
    bool m_resultHasBeenFound = true;
    int32_t currently_working = 0;
    std::mutex m_resultMutex;
    std::condition_variable m_result_var;
    constraint_node m_result;
    uint64_t m_node_id = 0;



    ReachabilityResult checkForReachability(); 
    
    void writeMultiplePathsToDisk(constraint_node cnode, std::string name);
    void writeConstraintNodeToDisk(constraint_node cnode, std::string name);  
};


#endif