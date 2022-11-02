#include <Planner/CBSPlanner.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

#include <queue>

using json = nlohmann::json;

dynamics::data::PoseByIndex CBSPlanner::toGlobalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex relative){
    return  (relative + base);
}

dynamics::data::Pose2D CBSPlanner::indexToPose(dynamics::data::PoseByIndex global){
    dynamics::data::Pose2D global_pose;
    
    double xpc = m_proxGraph.m_base_node_distance;
    double ypc = m_proxGraph.m_base_node_distance;
    double api = 2.f * PI / m_proxGraph.m_config_map_size_angle;

    global_pose.pos[0] = (xpc * global.x);
    global_pose.pos[1] = (ypc * global.y);
    global_pose.h = api * static_cast<double>(global.a);
    global_pose.vel = m_proxGraph.m_config_speedsFactor.at(global.s);
    
    return global_pose;
}


dynamics::data::PoseByIndex CBSPlanner::findNearestPoseByIndex(dynamics::data::Pose2D pose){

    double xpc = m_proxGraph.m_base_node_distance;
    double ypc = m_proxGraph.m_base_node_distance;
    double api = 2.f * PI / m_proxGraph.m_config_map_size_angle;

    double near_x = pose.pos[0] / xpc;
    double near_y = pose.pos[1] / ypc;
    
    pose.h = fmod(pose.h + 2*PI , 2*PI);

    double near_a = pose.h / api;

    dynamics::data::PoseByIndex result = {(int32_t)round(near_x),(int32_t)round(near_y),(int32_t)round(near_a),zero_velocity_level};

    return result;
}

dynamics::data::PoseByIndex CBSPlanner::toLocalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex global){
    return (global - base);
}

bool CBSPlanner::validatePosition(dynamics::data::PoseByIndex base){
    if(base.x < 0 || base.x > m_proxGraph.m_comp_map_size_x){
        return false;
    }
     if(base.y < 0 || base.y > m_proxGraph.m_comp_map_size_y){
        return false;
    }
    return true;
}


void CBSPlanner::writeVisitedNodesToDisk(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target,  std::unordered_map<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex> cameFrom){
    json visited_nodes;
    
    visited_nodes["target"]["x"] = target.x;
    visited_nodes["target"]["y"] = target.y;
    visited_nodes["target"]["a"] = target.a;
    visited_nodes["target"]["s"] = target.s;

    visited_nodes["start"]["x"] = start.x;
    visited_nodes["start"]["y"] = start.y;
    visited_nodes["start"]["a"] = start.a;
    visited_nodes["start"]["s"] = start.s;

    for(auto it: cameFrom){

        json node;
        node["x"] = it.second.x;
        node["y"] = it.second.y;
        node["a"] = it.second.a;
        node["s"] = it.second.s;

        node["px"] = it.first.x;
        node["py"] = it.first.y;
        node["pa"] = it.first.a;
        node["ps"] = it.first.s;

        visited_nodes["nodes"].push_back(node);
    }

    std::ofstream o("visited_nodes.json");
    o << visited_nodes << std::endl;
    o.close();

}

LLResult CBSPlanner::astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target, std::vector<dynamics::data::PBIConstraint> obstacles){
    
    std::priority_queue<dynamics::data::LLNode> openQueue;
    std::unordered_set<dynamics::data::PoseByIndex> openSet;
    std::unordered_map<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex> cameFrom;
    std::unordered_map<dynamics::data::PoseByIndex, MotionPrimitiv> usedEdge;

    auto current_pose = indexToPose(start);
    auto target_pose = indexToPose(target);

    std::unordered_map<dynamics::data::PoseByIndex, double> fScore;
    fScore[start] = (target_pose.pos - current_pose.pos).norm(); 
    
    std::unordered_map<dynamics::data::PoseByIndex, double> gScore;
    gScore[start] = 0.f;

    dynamics::data::LLNode initial = {start, fScore[start], 0};
    openQueue.push(initial);
    openSet.insert(start);
    uint32_t explored_nodes = 0;

    while(!openQueue.empty()){
        
        auto current = openQueue.top(); 
        explored_nodes += 1;

        if(current.pose.x == target.x && current.pose.y == target.y && current.pose.a == target.a ){
            std::cout << "a star finished" << std::endl;
            
            LLResult res;
            res.path = getPath(cameFrom, current.pose);
            res.spline = getSplines(cameFrom, usedEdge, current.pose);
            //writeVisitedNodesToDisk(start,target,cameFrom);
            res.found_path = true;

            return res;
        }

        std::cout << "Visited node: " << current.pose.x << ":" << current.pose.y << ":" << current.pose.a  << ":" << current.pose.s << std::endl;

        openQueue.pop();
        for(auto rel_neighbor: m_proxGraph.motion_primitive_map[current.pose.a][current.pose.s]){

            dynamics::data::PoseByIndex gl_neighbor = toGlobalIndex(current.pose, rel_neighbor.target);
            auto neigh_pose = indexToPose(gl_neighbor);

            // Invalid position, so we can skip this
            if(!validatePosition(gl_neighbor)){
                continue;
            }
            
            // TODO POSSIBLY DO A LOOKUP USING UNORDERED MAP or KD Tree to avoid this computation every time
            bool discard_due_to_obstacle = false;
            for(auto obstacle : obstacles){
                auto obstPose = indexToPose(obstacle);
                if(obstacle.t == current.timestep + 1 && (obstPose.pos - neigh_pose.pos).norm() < safe_radius){
                    discard_due_to_obstacle = true;
                    break;
                }
            }

            if(discard_due_to_obstacle){
                continue;
            }

            auto current_pose = indexToPose(current.pose); 
            double dist = (neigh_pose.pos - current_pose.pos).norm() + rel_neighbor.link.s_a + rel_neighbor.link.s_a_2;
                        
            double tentative_score = gScore[current.pose] + dist; 
            if(gScore.count(gl_neighbor) == 0 || tentative_score < gScore[gl_neighbor]){

                cameFrom[gl_neighbor] = current.pose;
                usedEdge[gl_neighbor] = rel_neighbor;
                gScore[gl_neighbor] = tentative_score;
                
                double h = (neigh_pose.pos - target_pose.pos).norm();
                fScore[gl_neighbor] = tentative_score + h;
                
                if(openSet.count(gl_neighbor) == 0){
                    dynamics::data::LLNode node = {gl_neighbor, fScore[gl_neighbor], current.timestep + 1};
                    openQueue.push(node);
                    openSet.insert(gl_neighbor);
                }
            }
        }
    }
    std::cout << "a star infeasible with explored nodes: " << explored_nodes << std::endl;
    std::cout << "START: " << start.x << ":" << start.y << ":" << start.a << ":" << start.s << std::endl;
    std::cout << "TARGET: " << target.x << ":" << target.y << ":" << target.a << ":" << target.s << std::endl;
    
    LLResult res;
    res.found_path = false;
    res.spline.clear();

    writeVisitedNodesToDisk(start,target,cameFrom);
    
    return res;
}

std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> CBSPlanner::getPath(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, dynamics::data::PoseByIndex& target){
    auto current = target;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> result = std::make_shared<std::vector<dynamics::data::PoseByIndex>>();

    do{
        result->push_back(current);
        current = predecessor[current];
    } while(predecessor.find(current) != predecessor.end());
    
    result->push_back(current);
    return result;
}

std::vector<dynamics::data::Pose2WithTime> CBSPlanner::getSplines(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, std::unordered_map<dynamics::data::PoseByIndex,MotionPrimitiv>& edge_map, dynamics::data::PoseByIndex target){
    auto current = target;
    std::vector<dynamics::data::PoseByIndex> nodes;
    std::vector<MotionPrimitiv> edges;

    do{
        nodes.push_back(current);
        edges.push_back(edge_map[current]);
        current = predecessor[current];
    }while(predecessor.find(current) != predecessor.end());

    nodes.push_back(current);
    edges.push_back(edge_map[current]);

    dynamics::data::Pose2D veh_pose;
    veh_pose = indexToPose(nodes.at(nodes.size() - 1));
    std::vector<dynamics::data::Pose2WithTime> result;
    double time = 0.f;
    for(int64_t i = nodes.size() - 1; i > 0; i --){
        
        uint32_t intp = 8;
        double timestep = static_cast<double>(edges.at(i - 1).link.ts_ms);
        auto pose_series = dynamics::SimpleDynamicsModel::computePoseSeries(veh_pose, edges.at(i - 1).link.s_a, edges.at(i - 1).link.s_a_2, edges.at(i - 1).link.start_vel, edges.at(i - 1).link.target_vel, intp, timestep, time);
        
        result.insert(result.end(), pose_series.begin(), pose_series.end());
        time = pose_series.back().time_ms;
        
        veh_pose.pos = pose_series.back().pos;
        veh_pose.h = pose_series.back().h;
        veh_pose.vel = pose_series.back().vel;
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

std::pair<bool, dynamics::data::Constraint> CBSPlanner::computeCollisions(constraint_node node, std::vector<dynamics::data::PoseByIndex> start_positions){
        
        std::pair<bool, dynamics::data::Constraint> result;
        result.first = false;

        for(int32_t car_index = 0; car_index < start_positions.size(); car_index ++){
            for (int32_t car_index2 = 0; car_index2 < start_positions.size(); car_index2 ++){
                
                if(car_index == car_index2){
                    continue;
                }

                for(uint32_t i = 0; i < node.result[car_index].spline.size() && i < node.result[car_index2].spline.size(); i ++){
                    
                    if((node.result[car_index].spline.at(i).pos - node.result[car_index2].spline.at(i).pos).norm() < safe_radius
                    && node.result[car_index].spline.at(i).time_ms - node.result[car_index2].spline.at(i).time_ms < safe_time){
                        
                        dynamics::data::Constraint con;
                        con.vehicle_a = car_index;
                        con.vehicle_b = car_index2;
                        con.time_ms = node.result[car_index].spline.at(i).time_ms;
                        con.pos = node.result[car_index].spline.at(i).pos;

                        std::cout << "FOUND CONFLICT!" << std::endl;
                    
                        result.first = true;
                        result.second =  con;            
                        return result;

                    }
                }   
            }
        }
    return result;
}

constraint_node CBSPlanner::cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions){
    std::cout << "CBS start" << std::endl;

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
        if(!m_lowLevelResults.at(i).found_path){
            std::cout << "CBS INFEASIBLE due to not finding initial path" << std::endl;
            return constraint_node();
        }
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
        openSet.pop();

        std::cout << "[INFO CBS] Iteration, openSet: " << openSet.size() << std::endl;
        std::cout << "[INFO CBS] SIC: " << node.sic << std::endl;

        for(auto r: node.result){
            if(!r.second.found_path){
                std::cout << "[INFO CBS] found infeasible node, no viable path was found" << std::endl;
                continue;
            }
        }

        //Validate Solution in first node to find conflicts
       auto collision = computeCollisions(node, start_positions);
        
        // Check if we have just obtained a valid solution -> terminate if yes
        if(!collision.first){
            std::cout << "CBS TERMINATION!" << std::endl;
            return node;
        }else{
           
           

            

            // Enqueu all jobs to astar workers
            

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
    constraint_node dnode;
    return dnode;
}


void CBSPlanner::writeCurveToDisk(std::vector<dynamics::data::Pose2WithTime> path, std::string name){
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

    cbs_paths["sizex"] = m_proxGraph.m_comp_map_size_x;
    cbs_paths["sizey"] = m_proxGraph.m_comp_map_size_y;

    for(auto node: cnode.result){
        std::cout << "Printing path for vehicle: " << node.first << std::endl; 
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