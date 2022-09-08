#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <util/Pose.hpp>
#include <util/Config.hpp>
#include <util/State.hpp>
#include <util/SearchTree.hpp>

#include <DynamicsModel/SingleTrackModel.hpp>

#include <queue>

namespace plan{

using namespace dynamics;

struct possible_vehicle_plans{
int test;

};

class SimpleRoutePlanner 
{
public:

struct lessThanByError
{
  bool operator()(const std::shared_ptr<CombinedSearchNode>& lhs, const std::shared_ptr<CombinedSearchNode>& rhs) const
  {
    return lhs->cumulative_error > rhs->cumulative_error;
  }
};

std::map<uint32_t, dynamics::VehicleState> vehicle_initial_states;

// std::map<uint32_t, std::shared_ptr<SearchTreeNode>> vehicle_tree_roots;

//std::shared_ptr<SearchTreeNode> single_step_vehicle(std::shared_ptr<SearchTreeNode> current_node);

std::vector<Pose2DWithError> single_step_vehicle_list(Pose2D pose, Pose2D target, bool complete = false);

float compute_error(dynamics::data::Pose2D planned_pose, dynamics::data::Pose2D comparison_point);

bool compute_intersection(dynamics::data::Pose2D planned_pose_a, dynamics::data::Pose2D planned_pose_b);

bool has_arrived(dynamics::data::Pose2D planned_pose, dynamics::VehicleState current_state);

std::vector<std::shared_ptr<CombinedSearchNode>> build_options_recursive(uint32_t index, std::shared_ptr<CombinedSearchNode> node, std::map<uint32_t, std::vector<dynamics::Pose2DWithError>>& map);

public:
    std::priority_queue<std::shared_ptr<CombinedSearchNode>,std::vector<std::shared_ptr<CombinedSearchNode>>, lessThanByError> queue;
    std::mutex queue_mutex;  
    std::mutex out_mutex;  
    bool terminate_solver_threads = false;
    std::shared_ptr<CombinedSearchNode> solution;

    void print(std::string string, int level);

    void solver_thread();

    SimpleRoutePlanner(){};
    
    void intitialize_randomly();

    // void plan();

    void plan2();

    void drawState(std::shared_ptr<SearchTreeNode> root);


    std::shared_ptr<CombinedSearchNode> get_results();
};

}
#endif