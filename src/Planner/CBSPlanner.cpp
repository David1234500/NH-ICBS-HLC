#include <Planner/CBSPlanner.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

void CBSPlanner::insert(dynamics::data::PoseByIndex node, std::vector<dynamics::data::PoseByIndex>& openSet,  std::map<dynamics::data::PoseByIndex, float>& fScoreMap){
    uint32_t index_left = 0;
    uint32_t index_right = openSet.size() - 1;
    
    while(index_left < index_right){
        uint32_t m = std::floor((index_left + index_right) / 2);
        if(fScoreMap[openSet.at(m)] < fScoreMap[node]){
            index_left = m + 1;
        }else if (fScoreMap[openSet.at(m)] > fScoreMap[node]){
            index_right = m - 1;
        }else{
            openSet.insert(openSet.begin() + m, node);
        }
    }
}


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



std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex> CBSPlanner::astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target){
    std::vector<dynamics::data::PoseByIndex> openSet;
    openSet.push_back(start);

    std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex> cameFrom;
    
    auto current_pose = indexToPose(start);
    auto target_pose = indexToPose(target);

    std::map<dynamics::data::PoseByIndex, float> fScore;
    fScore[start] = (target_pose.pos - current_pose.pos).norm();
    std::map<dynamics::data::PoseByIndex, float> gScore;
    gScore[start] = 0.f;

    while(!openSet.empty()){
        
        if(openSet.size() % 5 == 0){
            std::cout << "Size os:" << openSet.size() << std::endl;
        }

        auto current = openSet.front();
       
        if(current == target){
            std::cout << "a star finished" << std::endl;
            return cameFrom;
        }

        openSet.erase(openSet.begin());
        for(auto rel_neighbor: m_proxGraph.m_proxyEdgeList[current.a]){
            
            std::cout << "neigh" << rel_neighbor.target.x << " " << rel_neighbor.target.y << std::endl; 

            dynamics::data::PoseByIndex gl_neighbor = toGlobalIndex(current, rel_neighbor.target);

            auto neigh_pose = indexToPose(gl_neighbor);
            auto current_pose = indexToPose(current);
            
            float dist = (neigh_pose.pos - current_pose.pos).norm();
            
            if(gScore.count(gl_neighbor) == 0){
                gScore[gl_neighbor] = 1000000000.f;
                fScore[gl_neighbor] = 1000000000.f;
            }
                        
            float tentative_score = gScore[gl_neighbor] + dist; // HAS NO GSCORE JET
            
            if(tentative_score < gScore[gl_neighbor]){
                cameFrom[gl_neighbor] = current;
                
                gScore[gl_neighbor] = tentative_score;
                
                float h = (neigh_pose.pos - target_pose.pos).norm();
                fScore[gl_neighbor] = tentative_score + h;
                
                if(binarySearch(gl_neighbor, openSet, fScore) == -1){
                    insert(gl_neighbor, openSet, fScore);
                }
            }
        }
    }
    return cameFrom;
}

void CBSPlanner::writePathToDisk( std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex> predecessor, dynamics::data::PoseByIndex target){
    json astar_path;
    auto current = target;

    while(predecessor.find(current) != predecessor.end()){
        json node;
        node["x"] = current.x;
        node["y"] = current.y;
        node["s"] = current.s;
        node["a"] = current.a;
        astar_path["path"].push_back(node);

        current = predecessor[current];
    }

    //dump file to disc
    std::ofstream o("astar_path.json");
    o << astar_path << std::endl;
    o.close();
}