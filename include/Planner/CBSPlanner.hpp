#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <util/Pose.hpp>

#include <DynamicsModel/SingleTrackModel.hpp>
#include <Planner/ProxyGraph.hpp>
#include <Planner/StateGraph.hpp>
#include <util/ThreadPool.hpp>
#include <Config.hpp>
#include <queue>
#include <map>
#include <thread>



struct low_level_job{
    dynamics::data::PoseByIndex start_positions; 
    dynamics::data::PoseByIndex target_positions;
    std::shared_ptr<ProxyGraph> proxyGraph;
};


struct low_level_result{
    bool found_path = false;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> path;
};

struct collision_job{
    uint32_t job_id = 0;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> path1;
    uint16_t car_id_1 = 0;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> path2;
    uint16_t car_id_2 = 0;
};

struct collision_result{
    uint32_t job_id = 0;
    bool found_conflict = false;
    dynamics::data::PoseByIndex conflicting_pose;
    uint32_t conflict_step_count = 0;
};

struct constraint_node{
    uint64_t sic = 0;
    std::vector<dynamics::data::PoseByIndex> avoid;
    std::weak_ptr<constraint_node> parent; 
    std::shared_ptr<constraint_node> r_child = nullptr;
    std::shared_ptr<constraint_node> l_child = nullptr;
    std::map<uint32_t, low_level_result> result;

    bool operator < (const constraint_node r) const {
        if(sic < r.sic){
            return true;
        }else{
            return false;
        }
    }
    
};

low_level_result low_level_search_processor_func(low_level_job);
collision_result collision_processor_func(collision_job);

class CBSPlanner 
{
public:
    CBSPlanner(): m_lowLevelSearchPool(&low_level_search_processor_func), m_collisionCheckPool(&collision_processor_func){

    }

    ThreadPool<low_level_job, low_level_result> m_lowLevelSearchPool;
    ThreadPool<collision_job, collision_result> m_collisionCheckPool;

    static bool validatePosition(dynamics::data::PoseByIndex base);
    static dynamics::data::PoseByIndex toGlobalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex relative, std::shared_ptr<ProxyGraph> proxGraph);
    static dynamics::data::Pose2D indexToPose(dynamics::data::PoseByIndex);
    static dynamics::data::PoseByIndex toLocalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex global, std::shared_ptr<ProxyGraph> proxGraph);

    static std::vector<std::vector<dynamics::data::PoseByIndex>> cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions);
    
    static std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target, std::shared_ptr<ProxyGraph> proxGraph);
    static bool binarySearch(dynamics::data::PoseByIndex node, std::vector<dynamics::data::PoseByIndex>& openSet,  std::map<dynamics::data::PoseByIndex, float>& fScoreMap);
    static void insert(dynamics::data::PoseByIndex node, std::vector<dynamics::data::PoseByIndex>& openSet,  std::map<dynamics::data::PoseByIndex, float>& fScoreMap);


    std::shared_ptr<ProxyGraph> m_proxGraph;

    void writePathToDisk( std::vector<dynamics::data::PoseByIndex> path, std::string name);
    void writeCurveToDisk(std::vector<dynamics::data::Pose2D> path, std::string name);
    static std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> getPath(std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, dynamics::data::PoseByIndex& target);
    std::vector<dynamics::data::Pose2D> getCurves(std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, std::map<dynamics::data::PoseByIndex,TraversableEdge>& edge_map, dynamics::data::PoseByIndex target);

    

};


#endif