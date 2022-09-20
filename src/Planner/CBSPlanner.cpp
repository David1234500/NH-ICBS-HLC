#include <Planner/CBSPlanner.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <queue>

using json = nlohmann::json;

void CBSPlanner::insert(dynamics::data::PoseByIndex node, std::vector<dynamics::data::PoseByIndex>& openSet,  std::map<dynamics::data::PoseByIndex, float>& fScoreMap){
    uint32_t index = 0;
    while(index < openSet.size() && (fScoreMap[openSet.at(index)] < fScoreMap[node])){
        index ++;
    }
    openSet.insert(openSet.begin() + index, node);
}


bool CBSPlanner::binarySearch(dynamics::data::PoseByIndex node, std::vector<dynamics::data::PoseByIndex>& openSet,  std::map<dynamics::data::PoseByIndex, float>& fScoreMap){
    
    if(openSet.size() == 0){
        return false;
    }
    
    for(auto snode: openSet){
        if(snode == node){
            return true;
        }
    }
    return false;
    // int32_t index_left = 0;
    // int32_t index_right = openSet.size() - 1;
    
    // while(index_left <= index_right){
    //     int32_t m = std::floor((index_left + index_right) / 2);
        
    //     if(fScoreMap.count(openSet.at(m)) == 0){
    //         fScoreMap[openSet.at(m)] = 1000000000.f;
    //     }

    //     if(fScoreMap.count(node) == 0){
    //         fScoreMap[node] = 1000000000.f;
    //     }

    //     if(fScoreMap[openSet.at(m)] < fScoreMap[node]){
    //         index_left = m + 1;
    //     }else if (fScoreMap[openSet.at(m)] > fScoreMap[node]){
    //         index_right = m - 1;
    //     }else{
    //        return m;
    //     }
    // }
    // return -1;
}

dynamics::data::PoseByIndex CBSPlanner::toGlobalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex relative){
    return  (relative + base) - m_proxGraph.m_proxyMapCarOffset;
}

dynamics::data::Pose2D CBSPlanner::indexToPose(dynamics::data::PoseByIndex global){
    dynamics::data::Pose2D global_pose;
    
    global_pose.pos[0] = (xpc * global.x);
    global_pose.pos[1] = (ypc * global.y);
    global_pose.h = api * static_cast<float>(global.a);
    global_pose.vel = m_speedsFactor[global.s] * dynamics::SimpleDynamicsModel::velocity_limit();
    
    return global_pose;
}



dynamics::data::PoseByIndex CBSPlanner::toLocalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex global){
    return (global - base) +  m_proxGraph.m_proxyMapCarOffset;
}

bool CBSPlanner::validatePosition(dynamics::data::PoseByIndex base){
    if(base.a < 0 || base.a > map_size_angle){
        return false;
    }
    if(base.s < 0 || base.s > map_size_speed){
        return false;
    }
    if(base.x < 0 || base.x > map_size_x){
        return false;
    }
     if(base.y < 0 || base.y > map_size_y){
        return false;
    }
    return true;
}

std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> CBSPlanner::astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target){
    std::vector<dynamics::data::PoseByIndex> openSet;
    openSet.push_back(start);

    std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex> cameFrom;
    std::map<dynamics::data::PoseByIndex,TraversableEdge> usedEdge;
    
    auto current_pose = indexToPose(start);
    auto target_pose = indexToPose(target);

    std::map<dynamics::data::PoseByIndex, float> fScore;
    fScore[start] = (target_pose.pos - current_pose.pos).norm();
    
    std::map<dynamics::data::PoseByIndex, float> gScore;
    gScore[start] = 0.f;

    while(!openSet.empty()){
        
        
        // std::cout << "Size os:" << openSet.size() << std::endl;
        
        auto current = openSet.front();

        // std::cout << "current index :" << current.x << "_" << current.y << "_" << current.a << "_" << current.s << std::endl;
        // std::cout << "current scores g" << gScore[current] << " f" << fScore[current] <<  std::endl;

        if(current == target){
            std::cout << "a star finished" << std::endl;
            return getPath(cameFrom, current);
        }

        openSet.erase(openSet.begin());
        for(auto rel_neighbor: m_proxGraph.m_proxyEdgeList[current.a]){

            dynamics::data::PoseByIndex gl_neighbor = toGlobalIndex(current, rel_neighbor.target);

            // std::cout << "neigh " << rel_neighbor.target.x << " " << rel_neighbor.target.y << " " << rel_neighbor.target.a << " " << rel_neighbor.target.s << std::endl; 
            // std::cout << "gl neigh " << gl_neighbor.x << " " << gl_neighbor.y << " " << gl_neighbor.a << " " << gl_neighbor.s << std::endl; 

            auto neigh_pose = indexToPose(gl_neighbor);

            // Invalid position, so we can skip this
            if(!validatePosition(gl_neighbor)){
                continue;
            }

            auto current_pose = indexToPose(current);
            
            // std::cout << "npose" << neigh_pose.pos[0] << " "<< neigh_pose.pos[1]  << std::endl; 

            float dist = (neigh_pose.pos - current_pose.pos).norm();
            
            if(gScore.count(gl_neighbor) == 0){
                gScore[gl_neighbor] = 1000000000.f;
                fScore[gl_neighbor] = 1000000000.f;
            }
                        
            float tentative_score = gScore[current] + dist; // HAS NO GSCORE JET
            
            // std::cout << "t: " <<  tentative_score << " gs: " << gScore[gl_neighbor] << std::endl;

            if(tentative_score < gScore[gl_neighbor]){
                cameFrom[gl_neighbor] = current;
                usedEdge[gl_neighbor] = rel_neighbor;
                
                gScore[gl_neighbor] = tentative_score;
                
                float h = (neigh_pose.pos - target_pose.pos).norm();
                fScore[gl_neighbor] = tentative_score + h;
                
               
                if(!binarySearch(gl_neighbor, openSet, fScore)){
                    insert(gl_neighbor, openSet, fScore);
                }
            }
        }
    }
    return std::make_shared<std::vector<dynamics::data::PoseByIndex>>();
}

std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> CBSPlanner::getPath(std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, dynamics::data::PoseByIndex& target){
    auto current = target;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> result;

    while(predecessor.find(current) != predecessor.end()){
        result->push_back(current);
        current = predecessor[current];
    }
    return result;
}

std::vector<dynamics::data::Pose2D> CBSPlanner::getCurves(std::map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, std::map<dynamics::data::PoseByIndex,TraversableEdge>& edge_map, dynamics::data::PoseByIndex target){
    auto current = target;
    std::vector<dynamics::data::PoseByIndex> nodes;
    std::vector<TraversableEdge> edges;

    while(predecessor.find(current) != predecessor.end()){
        nodes.push_back(current);
        edges.push_back(edge_map[current]);
        std::cout << " " << current.x << " " << current.y << " " << current.a << " " << current.s << std::endl;
        current = predecessor[current];
    }

    std::vector<dynamics::data::Pose2D> result;
    for(int64_t i = nodes.size() - 1; i >= 1; i --){
       
        dynamics::data::Pose2D next_pose;
        dynamics::data::Pose2D veh_pose = indexToPose(nodes.at(i));
        for(float ts = 0; ts <= timestep_ms; ts += 25.f){    
            next_pose = dynamics::SimpleDynamicsModel::computeNextPose(veh_pose, edges.at(i - 1).link.s_a, edges.at(i - 1).link.s_v, ts);
            result.push_back(next_pose);
        }
        
        for(float ts = 0; ts <= timestep_ms; ts += 25.f){
            auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPose(next_pose, edges.at(i - 1).link.s_a_2, edges.at(i - 1).link.s_v_2, ts);
            result.push_back(next_pose2);
        }
    }

    return result;
}

void CBSPlanner::low_level_astar_worker(){
    while(m_keepThreadsAlive){
        
        bool hasJob = false;
        low_level_job job;

        m_lowLevelSearchJobLock.lock();
        if(!m_lowLevelJobs.empty()){
            hasJob = true;

            job = m_lowLevelJobs.back();
            m_lowLevelJobs.pop_back();
        
        }else{
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        m_lowLevelSearchJobLock.unlock();

        low_level_result res;

        res.path = astar(job.start_positions, job.target_positions);
        res.job_id = job.job_id;
        res.car_id = job.car_id;

        m_lowLevelSearchResultsLock.lock();
        m_lowLevelResults.push_back(res);
        m_lowLevelSearchResultsLock.unlock();
    }
}

void CBSPlanner::conflict_check_worker(){
    
    while(m_keepThreadsAlive){
        
        bool hasJob = false;
        collision_job job;

        m_conflictJobLock.lock();
        if(!m_conflictJobs.empty()){
            hasJob = true;

            job = m_conflictJobs.back();
            m_conflictJobs.pop_back();
        
        }else{
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        m_conflictJobLock.unlock();

        collision_result res;

         for(uint32_t i = 0; i < job.path1->size() && i < job.path2->size(); i ++){
            auto p = job.path1->at(i) - job.path2->at(i);
            if( p.x == 0 && p.y == 0){
                res.found_conflict = true;
                res.conflicting_pose = job.path1->at(i);
                res.conflict_step_count = i;
            }
        }

        res.job_id = job.job_id;

        m_conflictResultsLock.lock();
        m_conflictResults.push_back(res);
        m_conflictResultsLock.unlock();
    }    
}


std::vector<std::vector<dynamics::data::PoseByIndex>> CBSPlanner::cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions){
    std::vector<std::vector<dynamics::data::PoseByIndex>> paths_by_vehicle;
    
    for(uint32_t i = 0; i < worker_counter; i ++){
        m_lowLevelWorkers.push_back(std::thread(&CBSPlanner::low_level_astar_worker, this));
        m_conflictWorkers.push_back(std::thread(&CBSPlanner::conflict_check_worker, this));
    }

    // Enqueu all jobs to astar workers
    std::priority_queue<constraint_node> openSet;
    for(uint32_t i = 0; i < start_positions.size(); i ++){
        low_level_job job;
        job.job_id = i;
        job.start_positions = start_positions.at(i);
        job.target_positions = target_positions.at(i);
        job.car_id = i;
        
        m_lowLevelSearchJobLock.lock();
        m_lowLevelJobs.push_back(job);
        m_lowLevelSearchJobLock.unlock();
    }

    //Wait for all threads to termiante
    while(true){
        m_lowLevelSearchResultsLock.lock();
        if(m_lowLevelResults.size() == start_positions.size()){
            break;
        }
        m_lowLevelSearchResultsLock.unlock();
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Compute SIC by hop count
    uint64_t sic = 0;
    for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
        sic += m_lowLevelResults.at(i).path->size();
    }

    constraint_node node;
    node.sic = sic;
    node.avoid.clear();
    for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
        node.result = m_lowLevelResults.at(i); 
        node.result[m_lowLevelResults.at(i).car_id]= m_lowLevelResults.at(i);
    }
    
    openSet.push(node);
    while(!openSet.empty()){
        constraint_node node = openSet.top();

        for(uint32_t i = 0; i < start_positions.size(); i ++){
            


        }

        //Wait for all threads to termiante
        while(true){
            m_lowLevelSearchResultsLock.lock();
            if(m_lowLevelResults.size() == start_positions.size()){
                break;
            }
            m_lowLevelSearchResultsLock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } 

    }

    m_keepThreadsAlive = false;
    for(uint32_t i = 0; i < worker_counter; i ++){
        m_lowLevelWorkers.at(i).join();
        m_conflictWorkers.at(i).join();
    }
}


void CBSPlanner::writeCurveToDisk(std::vector<dynamics::data::Pose2D> path, std::string name){
    json astar_path;
    
    for(auto current: path){
        json node;
        node["x"] = current.pos[0];
        node["y"] = current.pos[1];

        astar_path["path"].push_back(node);
    }

    //dump file to disc
    std::ofstream o(name);
    o << astar_path << std::endl;
    o.close();
}

void CBSPlanner::writePathToDisk( std::vector<dynamics::data::PoseByIndex> path, std::string name){
    json astar_path;
    
    for(auto current: path){
        json node;
        node["x"] = current.x;
        node["y"] = current.y;
        node["s"] = current.s;
        node["a"] = current.a;

        astar_path["path"].push_back(node);
    }

    //dump file to disc
    std::ofstream o(name);
    o << astar_path << std::endl;
    o.close();
}