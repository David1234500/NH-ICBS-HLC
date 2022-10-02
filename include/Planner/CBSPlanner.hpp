#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <util/Pose.hpp>

#include <DynamicsModel/SingleTrackModel.hpp>
#include <Planner/ProxyGraph.hpp>
#include <Planner/StateGraph.hpp>
#include <Config.hpp>
#include <queue>
#include <map>
#include <thread>

struct low_level_job{
    dynamics::data::PoseByIndex start_positions; 
    dynamics::data::PoseByIndex target_positions;
    std::vector<dynamics::data::PBIConstraint> avoid;
    uint32_t job_id = 0;
    uint16_t car_id = 0;
    
};


struct low_level_result{
    bool found_path = false;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> path;
    uint32_t job_id = 0;
    uint16_t car_id = 0;
};

struct constraint_node{
    uint64_t sic = 0;
    std::vector<dynamics::data::PBIConstraint> avoid;
    std::shared_ptr<constraint_node> r_child = nullptr;
    std::shared_ptr<constraint_node> l_child = nullptr;
    std::map<int32_t, low_level_result> result;
    bool operator < (const constraint_node r) const {
        if(sic < r.sic){
            return true;
        }else{
            return false;
        }
    }
};


class CBSPlanner 
{
public:
    CBSPlanner(){

    }

    bool validatePosition(dynamics::data::PoseByIndex base);
    dynamics::data::PoseByIndex toGlobalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex relative);
    dynamics::data::Pose2D indexToPose(dynamics::data::PoseByIndex);
    dynamics::data::PoseByIndex toLocalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex global);

    constraint_node cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions);
    
    std::mutex m_lowLevelSearchJobLock;
    std::vector<low_level_job> m_lowLevelJobs;
    std::vector<std::thread> m_lowLevelWorkers;

    void low_level_astar_worker(uint32_t i);
    std::mutex m_lowLevelSearchResultsLock;
    std::vector<low_level_result> m_lowLevelResults;

    
    bool m_keepThreadsAlive = true;


    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target, std::vector<dynamics::data::PBIConstraint> obstacles );
    bool binarySearch(dynamics::data::PoseByIndex node, std::vector<dynamics::data::PoseByIndex>& openSet,  std::map<dynamics::data::PoseByIndex, float>& fScoreMap);
    void insert(dynamics::data::PoseByIndex node, std::vector<dynamics::data::PoseByIndex>& openSet,  std::map<dynamics::data::PoseByIndex, float>& fScoreMap);


    ProxyGraph m_proxGraph;

    void writePathToDisk( std::vector<dynamics::data::PoseByIndex> path, std::string name);
    void writeCurveToDisk(std::vector<dynamics::data::Pose2D> path, std::string name);
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> getPath(std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, dynamics::data::PoseByIndex& target);
    std::vector<dynamics::data::Pose2D> getCurves(std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, std::map<dynamics::data::PoseByIndex,TraversableEdge>& edge_map, dynamics::data::PoseByIndex target);

    

};


#endif