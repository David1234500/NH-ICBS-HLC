#include <Planner/CBSPlanner.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

#include <queue>

using json = nlohmann::json;

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

static dynamics::data::PoseByIndex CBSPlanner::findNearestPoseByIndex(dynamics::data::Pose2D pose){
    float near_x = pose.pos[0] / xpc;
    float near_y = pose.pos[1] / ypc;
    float near_y = pose.h / api;

    dynamics::data::PoseByIndex result = {round(x),round(y),round(a),0}

    return result;
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



LLResult CBSPlanner::astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target, std::unordered_set<dynamics::data::PBIConstraint> obstacles){
    
    std::priority_queue<dynamics::data::LLNode> openQueue;
    std::unordered_set<dynamics::data::PoseByIndex> openSet;
    

    std::unordered_map<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex> cameFrom;
    std::unordered_map<dynamics::data::PoseByIndex, TraversableEdge> usedEdge;

    auto current_pose = indexToPose(start);
    auto target_pose = indexToPose(target);

    std::unordered_map<dynamics::data::PoseByIndex, float> fScore;
    fScore[start] = (target_pose.pos - current_pose.pos).norm(); //+ 1 * (start.a - target.a);
    
    std::unordered_map<dynamics::data::PoseByIndex, float> gScore;
    gScore[start] = 0.f;


    dynamics::data::LLNode initial = {start, fScore[start], 0};
    openQueue.push(initial);
    openSet.insert(start);

    uint32_t explored_nodes = 0;

    while(!openQueue.empty()){
        
        auto current = openQueue.top(); 

        if(explored_nodes % 500 == 0){
            std::cout << "explored nodes:" << explored_nodes << " open nodes " << openQueue.size() << std::endl;
            std::cout << "current scores g" << gScore[current.pose] << " f" << fScore[current.pose] <<  std::endl;
            std::cout << "pose " << current.pose.x <<":"<< current.pose.y << std::endl;
        }
        
        explored_nodes += 1;

        if(current.pose == target){
            std::cout << "a star finished" << std::endl;
            
            LLResult res;
            res.path = getPath(cameFrom, current.pose);
            res.spline = getSplines(cameFrom, usedEdge, current.pose);
            res.found_path = true;

            return res;
        }

        openQueue.pop();
        for(auto rel_neighbor: m_proxGraph.m_proxyEdgeList[current.pose.a]){

            dynamics::data::PoseByIndex gl_neighbor = toGlobalIndex(current.pose, rel_neighbor.target);
            auto neigh_pose = indexToPose(gl_neighbor);

            // Invalid position, so we can skip this
            if(!validatePosition(gl_neighbor)){
                continue;
            }
            
            dynamics::data::PBIConstraint possible_constraint;
            possible_constraint = gl_neighbor;
            possible_constraint.t = current.timestep + 1;
            if(obstacles.count(possible_constraint) == 1){
                continue;
            }

            auto current_pose = indexToPose(current.pose); 
            float dist = (neigh_pose.pos - current_pose.pos).norm();
            
                        
            float tentative_score = gScore[current.pose] + dist; //+ 1 * std::abs(gl_neighbor.a - target.a);
            if(gScore.count(gl_neighbor) == 0 || tentative_score < gScore[gl_neighbor]){
                cameFrom[gl_neighbor] = current.pose;
                usedEdge[gl_neighbor] = rel_neighbor;
                gScore[gl_neighbor] = tentative_score;
                
                float h = (neigh_pose.pos - target_pose.pos).norm();
                fScore[gl_neighbor] = tentative_score + h;
                
                if(openSet.count(gl_neighbor) == 0){
                    dynamics::data::LLNode node = {gl_neighbor, fScore[gl_neighbor], current.timestep + 1};
                    openQueue.push(node);
                    openSet.insert(gl_neighbor);
                }
            }
        }
    }
    std::cout << "a star infeasible" << std::endl;
    LLResult res;
    res.found_path = false;
    return res;
}

std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> CBSPlanner::getPath(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, dynamics::data::PoseByIndex& target){
    auto current = target;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> result = std::make_shared<std::vector<dynamics::data::PoseByIndex>>();

    while(predecessor.find(current) != predecessor.end()){
        result->push_back(current);
        current = predecessor[current];
    }
    return result;
}

std::vector<dynamics::data::Pose2D> CBSPlanner::getSplines(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, std::unordered_map<dynamics::data::PoseByIndex,TraversableEdge>& edge_map, dynamics::data::PoseByIndex target){
    auto current = target;
    std::vector<dynamics::data::PoseByIndex> nodes;
    std::vector<TraversableEdge> edges;

    while(predecessor.find(current) != predecessor.end()){
        nodes.push_back(current);
        edges.push_back(edge_map[current]);
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

void CBSPlanner::low_level_astar_worker(uint32_t threadid){
    while(m_keepThreadsAlive){
        
        bool hasJob = false;
        LLJob job;

        m_lowLevelSearchJobLock.lock();
        if(!m_lowLevelJobs.empty()){
            hasJob = true;
            job = m_lowLevelJobs.back();
            m_lowLevelJobs.pop_back();
            m_lowLevelSearchJobLock.unlock();
        }else{
            m_lowLevelSearchJobLock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        

        
        auto res = astar(job.start_positions, job.target_positions, job.avoid);
        res.job_id = job.job_id;
        res.car_id = job.car_id;
        m_lowLevelSearchResultsLock.lock();
        m_lowLevelResults.push_back(res);
        m_lowLevelSearchResultsLock.unlock();
    }
}



constraint_node CBSPlanner::cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions){
    std::vector<std::vector<dynamics::data::PoseByIndex>> paths_by_vehicle;
    
    for(uint32_t i = 0; i < worker_counter; i ++){
        m_lowLevelWorkers.push_back(std::thread(&CBSPlanner::low_level_astar_worker, this, i));
    }

    // Enqueu all jobs to astar workers
    std::priority_queue<constraint_node> openSet;
    for(uint32_t i = 0; i < start_positions.size(); i ++){
        LLJob job;

        job.job_id = i;
        job.avoid.clear();
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
            m_lowLevelSearchResultsLock.unlock();
            break;
        }
        m_lowLevelSearchResultsLock.unlock();
       std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[INFO CBS] All threads returned" << std::endl;
    // Compute SIC by hop count
    
    uint64_t sic = 0;
    for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
        sic += m_lowLevelResults.at(i).path->size();
    }

    // Create initial constraint node
    constraint_node node;
    node.sic = sic;
    node.avoid.clear();
    for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
        node.result[m_lowLevelResults.at(i).car_id] = m_lowLevelResults.at(i);
    }
    m_lowLevelResults.clear();
    
    std::cout << "[INFO CBS] Finished intial low level phase for all vehicles" << std::endl;


    // Insert initial constraint node into list
    openSet.push(node);
    while(!openSet.empty()){
        constraint_node node = openSet.top();

        std::cout << "[INFO CBS] Iteration, openSet: " << openSet.size() << std::endl;
        std::cout << "[INFO CBS] SIC: " << node.sic << std::endl;

        //Validate Solution in first node to find conflicts
        dynamics::data::PoseByIndex conflict_pose;
        int32_t conflict_step = -1;
        std::array<int32_t,2> conflicting_vehicles = {-1,-1};

        for(int32_t car_index = 0; car_index < start_positions.size(); car_index ++){
            for (int32_t car_index2 = 0; car_index2 < start_positions.size(); car_index2 ++){
                
                if(car_index == car_index2){
                    continue;
                }

                for(uint32_t i = 0; i < node.result[car_index].path->size() && i < node.result[car_index2].path->size(); i ++){
                
                    // std::cout << "Conflict check " << std::endl;
                    // std::cout << "V1: " << node.result[car_index].path->at(i).x << ":" << node.result[car_index].path->at(i).y << std::endl;
                    // std::cout << "V2: " << node.result[car_index2].path->at(i).x << ":" << node.result[car_index2].path->at(i).y << std::endl;

                    auto p = node.result[car_index].path->at(i) - node.result[car_index2].path->at(i);
                    if(p.x == 0 && p.y == 0){
                        
                        std::cout << "Found a conflict" << std::endl;
                        conflict_pose = node.result[car_index].path->at(i);
                        conflict_step = i;
                        conflicting_vehicles = {car_index, car_index2};
                        break;

                    }
                }   
            }
        }


        std::cout << "[INFO CBS] Conflict at: " << conflict_step << " with " << conflicting_vehicles[0] << ":" << conflicting_vehicles[1] << std::endl;

        // Check if we have just obtained a valid solution -> terminate if yes
        if(conflict_step == -1){
            std::cout << "CBS TERMINATION!" << std::endl;
            return node;
        }

        // Add new constraints and replan
        for(auto vehicle_in_conflict: conflicting_vehicles){
            // Create new constraint node
            constraint_node constraint = node;

            dynamics::data::PBIConstraint constr;
            constr.id = vehicle_in_conflict;
            constr = conflict_pose;
            constr.t = conflict_step;

            std::cout << "[INFO CBS] Adding new constraint to situation " << conflict_pose.x << ":" << conflict_pose.y << std::endl;

            constraint.avoid.push_back(constr);

            // Recompute Solutions for this path TODO: maybe restore astar compute from here

            // Enqueu all jobs to astar workers
            for(uint32_t i = 0; i < start_positions.size(); i ++){
                LLJob job;
                job.job_id = i;
                job.avoid.clear();

                for(auto c: constraint.avoid){
                    if(c.id == i){
                        job.avoid.insert(c);
                    }
                }

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
                    m_lowLevelSearchResultsLock.unlock();
                    break;
                }
                m_lowLevelSearchResultsLock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // Compute SIC by hop count
            uint64_t sic = 0;
            bool found_paths_for_all_vehicles = true;
            for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
                if(m_lowLevelResults.at(i).found_path){
                    sic += m_lowLevelResults.at(i).path->size();
                }else{
                    found_paths_for_all_vehicles = false;
                }
            }

            // Update constraint node with new information
            constraint.sic = sic;
            for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
                constraint.result[m_lowLevelResults.at(i).car_id]= m_lowLevelResults.at(i);
            }
            m_lowLevelResults.clear();

            // Add new constraint node if it is feasible
            if(found_paths_for_all_vehicles){
                openSet.push(constraint);
            }else{
                std::cout << "CBS Found infeasible node" << std::endl;
            }
        }
    }

    m_keepThreadsAlive = false;
    for(uint32_t i = 0; i < worker_counter; i ++){
        m_lowLevelWorkers.at(i).join();
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

void CBSPlanner::writeMultiplePathsToDisk(constraint_node cnode, std::string name){
    json cbs_paths;

    cbs_paths["sizex"] = map_size_x;
    cbs_paths["sizey"] = map_size_y;

    for(auto node: cnode.result){
        json path_vehicle;
        for(auto current: *node.second.path){
            json node;
            node["x"] = current.x;
            node["y"] = current.y;
            node["s"] = current.s;
            node["a"] = current.a;
            path_vehicle["path"].push_back(node);
        }
        path_vehicle["id"] = node.first;
        cbs_paths["multipath"].push_back(path_vehicle);
    }

    //dump file to disc
    std::ofstream o(name);
    o << cbs_paths << std::endl;
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