#include <Planner/CBSPlanner.hpp>

#include <iostream>




int32_t CBSPlanner::binarySearch(dynamics::data::PoseByIndex node, std::vector<dynamics::data::PoseByIndex>& openSet,  std::map<dynamics::data::PoseByIndex, float>& fScoreMap){
    uint32_t index_left = 0;
    uint32_t index_right = openSet.size() - 1;
    
    while(index_left < index_right){
        uint32_t m = std::floor((index_left + index_right) / 2);
        if(fScoreMap[openSet.at(m)] < fScoreMap[node]){
            index_left = m + 1;
        }else if (fScoreMap[openSet.at(m)] > fScoreMap[node]){
            index_right = m - 1;
        }else{
           return m;
        }
    }
    return -1;
}

dynamics::data::PoseByIndex CBSPlanner::toGlobalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex relative){
    return base + (relative - m_proxGraph.m_proxyMapCarOffset);
}

dynamics::data::Pose2D CBSPlanner::indexToPose(dynamics::data::PoseByIndex global){
    dynamics::data::Pose2D global_pose;
    
    global_pose.pos[0] = (xpc*global.x);
    global_pose.pos[1] = (ypc*global.y);
    global_pose.h = api * static_cast<float>(global.a);
    global_pose.vel = m_speedsFactor[global.s] * dynamics::SimpleDynamicsModel::velocity_limit();
    
    return global_pose;
}



dynamics::data::PoseByIndex CBSPlanner::toLocalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex global){
    return (global - base) +  m_proxGraph.m_proxyMapCarOffset;
}

void CBSPlanner::astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target){
    std::vector<dynamics::data::PoseByIndex> openSet;

    std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex> cameFrom;
    std::map<dynamics::data::PoseByIndex, float> fScore;
    std::map<dynamics::data::PoseByIndex, float> gScore;

    auto target_pose = indexToPose(target);

    while(!openSet.empty()){
        auto current = openSet.front();
       
        if(current == target){
            std::cout << "a star finished" << std::endl;
            return;
        }

        openSet.erase(openSet.begin());
        for(auto rel_neighbor: m_proxGraph.m_proxyEdgeList[current.a]){

            dynamics::data::PoseByIndex gl_neighbor = toGlobalIndex(current, rel_neighbor.target);

            auto neigh_pose = indexToPose(gl_neighbor);
            auto current_pose = indexToPose(current);
            
            float dist = (neigh_pose.pos - current_pose.pos).norm();
            auto current_state_node = m_stateGraph[current];
            float tentative_score = gScore[gl_neighbor] + dist;
            
            if(tentative_score < 0.f){
                cameFrom[gl_neighbor] = current;
                gScore[gl_neighbor] = tentative_score;
                
                float h = (neigh_pose.pos - target_pose.pos).norm();
                fScore[gl_neighbor] = tentative_score + h;
                
                if(binarySearch(gl_neighbor, openSet, fScore) != -1){
                              
                }
            }
        }
    }
}
