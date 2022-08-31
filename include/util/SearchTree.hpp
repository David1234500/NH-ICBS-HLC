#ifndef SEARCH_TREE_NODE_HPP
#define SEARCH_TREE_NODE_HPP

#include <util/State.hpp>
#include <map>

class SearchTreeNode{
public:
    SearchTreeNode(){};

    std::map<float, std::map<float, float>> options_error_map; 

    std::weak_ptr<SearchTreeNode> predecessor;

    float gscore = 10000000.f; 
    float fscore = 0.f;

    dynamics::Pose2D pose;
};

class SearchNode{
public:
    SearchNode(){};

    std::map<float, std::map<float, float>> options_error_map; 

    std::weak_ptr<SearchTreeNode> predecessor;

    float error = 0.f;

    dynamics::Pose2D pose;
};

class CombinedSearchNode{
public:
    CombinedSearchNode(){};

    float cumulative_error = 0.f;

    std::map<uint32_t, dynamics::Pose2D> vehicle_poses;
 
    std::shared_ptr<CombinedSearchNode> predecessor;
};




#endif // SEARCH_TREE_NODE_HPP