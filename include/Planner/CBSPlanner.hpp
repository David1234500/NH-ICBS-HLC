#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <util/Pose.hpp>

#include <DynamicsModel/SingleTrackModel.hpp>
#include <Planner/ProxyGraph.hpp>
#include <Planner/StateGraph.hpp>
#include <unordered_set>
#include <unordered_map>
#include <Config.hpp>
#include <queue>
#include <map>
#include <thread>
#include <memory>

struct LLJob{
    dynamics::data::PoseByIndex start_positions; 
    dynamics::data::PoseByIndex target_positions;
    std::vector<dynamics::data::PBIConstraint> avoid;
    uint32_t job_id = 0;
    uint16_t car_id = 0;  
};


struct LLResult{
    bool found_path = false;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> path;
    std::vector<dynamics::data::Pose2WithTime> spline;
    uint32_t job_id = 0;
    uint16_t car_id = 0;
};

struct constraint_node{
    uint64_t sic = 0;
    std::vector<dynamics::data::PBIConstraint> avoid;
    std::map<int32_t, LLResult> result;
   
    bool operator < (const constraint_node r) const {
        if(sic < r.sic){
            return false;
        }else{
            return true;
        }
    }
};

class CBSPlanner 
{
public:
    CBSPlanner(){}
    ~CBSPlanner(){
        m_keepThreadsAlive = false;
        for(auto& s: m_lowLevelWorkers){
            s.join();
        }
    }

    bool validatePosition(dynamics::data::PoseByIndex base);
    dynamics::data::PoseByIndex toGlobalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex relative);
    dynamics::data::Pose2D indexToPose(dynamics::data::PoseByIndex);
    dynamics::data::PoseByIndex toLocalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex global);
    dynamics::data::Pose2D indexToPose(dynamics::data::PBIConstraint global);

    static dynamics::data::PoseByIndex findNearestPoseByIndex(dynamics::data::Pose2D pose);

    constraint_node cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions);
    
    std::mutex m_lowLevelSearchJobLock;
    std::vector<LLJob> m_lowLevelJobs;
    std::vector<std::thread> m_lowLevelWorkers;

    void low_level_astar_worker(uint32_t i);
    std::mutex m_lowLevelSearchResultsLock;
    std::vector<LLResult> m_lowLevelResults;
    bool m_keepThreadsAlive = true;

    LLResult astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target, std::vector<dynamics::data::PBIConstraint> obstacles);

    ProxyGraph m_proxGraph;

    void writePathToDisk( std::vector<dynamics::data::PoseByIndex> path, std::string name);
    void writeCurveToDisk(std::vector<dynamics::data::Pose2WithTime> path, std::string name);
    void writeMultiplePathsToDisk(constraint_node cnode, std::string name);
    
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> getPath(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, dynamics::data::PoseByIndex& target);
    std::vector<dynamics::data::Pose2WithTime> getSplines(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, std::unordered_map<dynamics::data::PoseByIndex,TraversableEdge>& edge_map, dynamics::data::PoseByIndex target);
    

};


#endif