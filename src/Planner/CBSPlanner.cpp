#include <Planner/CBSPlanner.hpp>
#include <Planner/Logger.hpp>

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

dynamics::data::Pose2D CBSPlanner::indexToPose(dynamics::data::PBIConstraint global){
    dynamics::data::Pose2D global_pose;
    
    global_pose.pos[0] = (xpc * global.x);
    global_pose.pos[1] = (ypc * global.y);
    global_pose.vel = 0.f;
    global_pose.h = 0;
    
    return global_pose;
}



dynamics::data::PoseByIndex CBSPlanner::findNearestPoseByIndex(dynamics::data::Pose2D pose){
    float near_x = pose.pos[0] / xpc;
    float near_y = pose.pos[1] / ypc;
    
    pose.h = fmod(pose.h + 2*PI , 2*PI);

    float near_a = pose.h / api;

    dynamics::data::PoseByIndex result = {(int32_t)round(near_x),(int32_t)round(near_y),(int32_t)round(near_a),zero_velocity_level};

    return result;
}


dynamics::data::PoseByIndex CBSPlanner::toLocalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex global){
    return (global - base) +  m_proxGraph.m_proxyMapCarOffset;
}

bool CBSPlanner::validatePosition(dynamics::data::PoseByIndex base){
    if(base.x < 0 || base.x > map_size_x){
        return false;
    }
     if(base.y < 0 || base.y > map_size_y){
        return false;
    }
    return true;
}

ReachabilityResult CBSPlanner::checkForReachability(){
    ReachabilityResult result;
    result.reachable = true;

    float reachable_distance = dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<float>(timestep_ms) / 1000.f);
    int32_t reachable_node_count = static_cast<uint32_t>(reachable_distance / xpc);

    uint32_t count = 0;
    uint64_t path_length = 0;
    float path_time = 0.f;

    rlog("ReachCheck", LOG_INFO, "Checking reachability with Span: " + std::to_string(reachable_node_count) + ":" + std::to_string(reachable_distance) );

    for(uint32_t i =0; i < map_size_angle; i ++){
        for(uint32_t j = 0; j < map_size_speed; j ++){
            result.edge_count += m_proxGraph.m_proxyEdgeList[i][j].size();
        }
    }

    // Assumption 2: We can leave each configuration through at least one edge (so can always start moving)
    rlog("ReachCheck", LOG_INFO, "Checking for unleavable start configurations...");
    for(int32_t i = 0; i < map_size_angle; i ++){
        for(int32_t j = 0; j < map_size_speed; j ++){
            if(m_proxGraph.m_proxyEdgeList[i][j].empty()){
                    rlog("ReachCheck", LOG_WARNING, "Found unleavable source configuration " + std::to_string(i) + ":" + std::to_string(j));
                    result.reachable = false;
                    auto unleavable_configuration = std::make_pair(i,j);
                    result.unleavable_start_configurations.push_back(unleavable_configuration);
                    return result;
            }
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Assumption 3: We can reach each configuration through at least one edge (so can always stop moving)
    rlog("ReachCheck", LOG_INFO, "Checking for unreachable end configurations...");
    for(int32_t ta = 0; ta < map_size_angle; ta ++){
        for(int32_t ts = 0; ts < map_size_speed; ts ++){
            bool found_link = false;

            for(int32_t sh = 0; sh < map_size_angle && !found_link; sh ++ ){
                for(int32_t sv = 0; sv < map_size_speed && !found_link; sv ++){
                    for(auto prim: m_proxGraph.m_proxyEdgeList[sh][sv]){
                        
                        if(prim.target.a == ta && prim.target.s == ts){
                            found_link = true;
                            break;
                        }
                    
                    }
                }
            }

            if(!found_link){
                    rlog("ReachCheck", LOG_WARNING, "Found unreachable target configuration " + std::to_string(ta) + ":" + std::to_string(ts));
                    result.reachable = false;
                    auto unreachable_config = std::make_pair(ta,ts);
                    result.unreachable_end_configuartions.push_back(unreachable_config);
                    return result;
            }
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for(uint32_t i = 0; i < worker_counter; i ++){
        m_lowLevelWorkers.push_back(std::thread(&CBSPlanner::low_level_astar_worker, this, i));
    }

    // Assumption 1: Every node for a full level is reachable
    uint32_t job_count = 0;
    for(int32_t v = 0; v < map_size_speed; v ++){
        if(!m_speedFactorIntermediate[v]){
            
            for(int32_t tx = - reachable_node_count; tx < reachable_node_count; tx ++){
                rlog("ReachCheck", LOG_INFO, "Checking reachability for speed: " + std::to_string(v) + ": " + std::to_string(m_speedFactorIntermediate[v]));
                for(int32_t ty = - reachable_node_count; ty < reachable_node_count; ty ++){

                    for(int32_t ta = 0; ta < map_size_angle; ta ++){
                        for(int32_t sh = 0; sh < map_size_angle; sh ++){
                    
                            dynamics::data::PoseByIndex start = {map_size_x/2,map_size_y/2,sh,v};
                            dynamics::data::PoseByIndex target_offset = {tx,ty,ta,v};
                            auto gTarget = toGlobalIndex(start, target_offset);
            
                            rlog("ReachCheck", LOG_INFO, "Checking reachability for position: " + std::to_string(tx) + ":" + std::to_string(ty) + ":" + std::to_string(ta));
                            
                            constraint_node node;
                            enqueue_astar( start, gTarget, node, job_count);
                            job_count += 1;
                        }
                    }
                }
            }
        }
    }

    await_astar_result(job_count);
    rlog("ReachCheck", LOG_INFO, "Got all reachability results... ");

    bool found_all_paths = true;
    for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
        auto res = m_lowLevelResults.at(i);
        if(res.found_path){
            count += 1;
            for(uint32_t i = 0; i < res.spline.size(); i ++){
                path_time += res.spline.at(i).time_ms;
            }
            
        }else{
            rlog("ReachCheck", LOG_INFO, "Failed for a configuration! ");
            result.reachable = false;
            return result;
        }
    }

    result.mean_time_length = path_time/static_cast<float>(count);
    result.mean_path_length = static_cast<float>(path_length)/static_cast<float>(count);
    return result;
}


LLResult CBSPlanner::astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target, std::vector<dynamics::data::PBIConstraint> obstacles){
    
    std::priority_queue<dynamics::data::LLNode> openQueue;
    std::unordered_set<dynamics::data::PoseByIndex> openSet;
    std::unordered_map<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex> cameFrom;
    std::unordered_map<dynamics::data::PoseByIndex, TraversableEdge> usedEdge;

    for(auto obstacle: obstacles){
        rlog("ASTAR", LOG_INFO, "A*S Obstacle: " + std::to_string(obstacle.x) + ":" + std::to_string(obstacle.y) + ":" + std::to_string(obstacle.t));
    }

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
        explored_nodes += 1;

        if(current.pose == target){
            rlog("ASTAR", LOG_INFO, "Found path for " + std::to_string(target.x) + ":" + std::to_string(target.y) + ":" + std::to_string(target.a));
            
            LLResult res;
            
            res.path = getPath(cameFrom, current.pose);
            res.spline = getSplines(cameFrom, usedEdge, current.pose);
            res.interprimitive = getInterPrimitivPositions(cameFrom, usedEdge, current.pose);

            res.found_path = true;

            return res;
        }

        bool discard = false;
        for(auto obstacle : obstacles){
            if( std::abs(current.timestep - obstacle.t) <= 1 && obstacle ==  current.pose){
                rlog("ASTAR", LOG_INFO, "2. Discarding neighbor due to conflict t: " + std::to_string(current.timestep) + " l: " + std::to_string(current.pose.x) + ":" + std::to_string(current.pose.y), ACONFLICT);
                discard = true;
                break;
            }
        }
        if(discard){
            continue;
        }


        openQueue.pop();
        for(auto rel_neighbor: m_proxGraph.m_proxyEdgeList[current.pose.a][current.pose.s]){

            dynamics::data::PoseByIndex gl_neighbor = toGlobalIndex(current.pose, rel_neighbor.target);
            auto neigh_pose = indexToPose(gl_neighbor);

            // Invalid position, so we can skip this
            if(!validatePosition(gl_neighbor)){
                continue;
            }
            
            // TODO POSSIBLY DO A LOOKUP USING UNORDERED MAP or KD Tree to avoid this computation every time
            bool discard_due_to_obstacle = false;
            for(auto obstacle : obstacles){
               
                // rlog("ASTAR", LOG_INFO, "Obstacle, time: " + std::to_string(current.timestep) + " l: " + std::to_string(gl_neighbor.x) + ":" + std::to_string(gl_neighbor.y), ACONFLICT);
               
                auto obstPose = indexToPose(obstacle);

                if( std::abs((current.timestep + 1) - obstacle.t) <= 1 && (obstPose.pos - neigh_pose.pos).norm() < safe_radius){
                    discard_due_to_obstacle = true;
                    // rlog("ASTAR", LOG_INFO, "1. Discarding neighbor due to conflict t: " + std::to_string(current.timestep) + "-" + std::to_string(std::abs(current.timestep - obstacle.t)) + " l: " + std::to_string(gl_neighbor.x) + ":" + std::to_string(gl_neighbor.y), ACONFLICT);
                    break;
                }

            }

            if(!discard_due_to_obstacle){

                auto current_pose = indexToPose(current.pose); 
                float dist = (neigh_pose.pos - current_pose.pos).norm();
                float tentative_score = gScore[current.pose] + dist; 
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

        
    }
    
    rlog("ASTAR", LOG_WARNING, "Found no viable path with #open: " + std::to_string(explored_nodes));
    rlog("ASTAR", LOG_WARNING, "Target: " + std::to_string(target.x) + ":" + std::to_string(target.y) + ":" + std::to_string(target.a));
    rlog("ASTAR", LOG_WARNING, "Start: " + std::to_string(start.x) + ":" + std::to_string(start.y) + ":" + std::to_string(start.a));
    
    // writeVisitedNodesToDisk(start,target,cameFrom);
    LLResult res;
    res.found_path = false;
    res.spline.clear();
    
    return res;
}

std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> CBSPlanner::getPath(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, dynamics::data::PoseByIndex& target){
    auto current = target;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> result = std::make_shared<std::vector<dynamics::data::PoseByIndex>>();
    uint32_t index = 0;
    do{

        result->push_back(current);
        current = predecessor[current];
        rlog("GetPath", LOG_INFO, "P: " + std::to_string(current.x) + ":" + std::to_string(current.y) + ":" + std::to_string(current.a) + ":" + std::to_string(current.s) + " index: " + std::to_string(index), GETPATH);

        index ++;
    } while(predecessor.find(current) != predecessor.end());
    
    result->push_back(current);

    return result;
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

std::vector<dynamics::data::Pose2WithTime> CBSPlanner::getSplines(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, std::unordered_map<dynamics::data::PoseByIndex,TraversableEdge>& edge_map, dynamics::data::PoseByIndex target){
    auto current = target;
    std::vector<dynamics::data::PoseByIndex> nodes;
    std::vector<TraversableEdge> edges;

    do{
        nodes.push_back(current);
        edges.push_back(edge_map[current]);
        current = predecessor[current];
    }while(predecessor.find(current) != predecessor.end());

    nodes.push_back(current);
    edges.push_back(edge_map[current]);

    std::vector<dynamics::data::Pose2WithTime> result;
    
    uint32_t time_index = 0;
    dynamics::data::Pose2D veh_pose = indexToPose(nodes.at( nodes.size() - 1));

    for(int64_t i = nodes.size() - 1; i >= 0; i --){

        dynamics::data::Pose2WithTime next_with_time;
        next_with_time = indexToPose(nodes.at(i)); 
        next_with_time.time_ms = (time_index * 2 * timestep_ms); 
        result.push_back(next_with_time);

        time_index += 1;
    }
    
    return result;
}

std::vector<dynamics::data::Pose2WithTime> CBSPlanner::getInterPrimitivPositions(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, std::unordered_map<dynamics::data::PoseByIndex,TraversableEdge>& edge_map, dynamics::data::PoseByIndex target){
    auto current = target;
    std::vector<dynamics::data::PoseByIndex> nodes;
    std::vector<TraversableEdge> edges;

    do{
        nodes.push_back(current);
        edges.push_back(edge_map[current]);
        current = predecessor[current];
    }while(predecessor.find(current) != predecessor.end());

    nodes.push_back(current);
    edges.push_back(edge_map[current]);

    std::vector<dynamics::data::Pose2WithTime> result;
    uint32_t time_index = 0;
    float half_step =  (timestep_ms / 2);
    
    for(int64_t i = nodes.size() - 1; i > 0; i --){

        dynamics::data::Pose2D veh_pose = indexToPose(nodes.at(i));
        dynamics::data::Pose2D next_pose;

        for(float ts = 0; ts < timestep_ms; ts += 50.f){    
            next_pose = dynamics::SimpleDynamicsModel::computeNextPose(veh_pose, edges.at(i - 1).link.s_a, edges.at(i - 1).link.s_v, ts);
            dynamics::data::Pose2WithTime next_with_time;
            next_with_time = next_pose;
            next_with_time.time_ms = time_index * 2 * timestep_ms + ts;
            result.push_back(next_with_time);
        }
        
        for(float ts = 0; ts < timestep_ms; ts += 50.f){
            auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPose(next_pose, edges.at(i - 1).link.s_a_2, edges.at(i - 1).link.s_v_2, ts);
            dynamics::data::Pose2WithTime next_pose2_with_time;
            next_pose2_with_time = next_pose2;
            next_pose2_with_time.time_ms = time_index * 2 * timestep_ms + timestep_ms + ts;
            result.push_back(next_pose2_with_time);
        }


        time_index += 1;
    }
    
    dynamics::data::Pose2D veh_pose = indexToPose(nodes.at(0));
    dynamics::data::Pose2WithTime next_with_time;
    next_with_time = veh_pose;
    next_with_time.time_ms = time_index * 2 * timestep_ms;
    result.push_back(next_with_time);

    return result;
}


void CBSPlanner::low_level_astar_worker(uint32_t threadid){
    while(m_keepThreadsAlive){
        
        bool hasJob = false;
        LLJob job;

        m_lowLevelSearchJobLock.lock();
        // rlog("low_level_astar_worker", LOG_WARNING, "Remaining queue size: " + std::to_string(m_lowLevelJobs.size()));
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
        
        LLResult res = astar(job.start_positions, job.target_positions, job.avoid);
        res.job_id = job.job_id;
        res.car_id = job.car_id;
        
        m_lowLevelSearchResultsLock.lock();
        m_lowLevelResults.push_back(res);
        m_lowLevelSearchResultsLock.unlock();
    }
}

void CBSPlanner::enqueue_astar(dynamics::data::PoseByIndex& start, dynamics::data::PoseByIndex& target, constraint_node& constraint, uint32_t& id){
    LLJob job;
    
    job.job_id = id;
    job.avoid.clear();
    job.avoid = constraint.avoid; // this should just copy the constraints with correct ids
    job.start_positions = start;
    job.target_positions = target;
    job.car_id = id;

    m_lowLevelSearchJobLock.lock();
    m_lowLevelJobs.push_back(job);
    m_lowLevelSearchJobLock.unlock();
}

void CBSPlanner::await_astar_result(uint32_t count){
     while(true){
        m_lowLevelSearchResultsLock.lock();
        if(m_lowLevelResults.size() == count){
            m_lowLevelSearchResultsLock.unlock();
            break;
        }
        m_lowLevelSearchResultsLock.unlock();
       std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

constraint_node CBSPlanner::cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions){
    std::cout << "CBS start" << std::endl;
    uint32_t node_id = 0;

    for(uint32_t i = 0; i < worker_counter; i ++){
        m_lowLevelWorkers.push_back(std::thread(&CBSPlanner::low_level_astar_worker, this, i));
    }

    // Enqueu all jobs to astar workers
    std::priority_queue<constraint_node> openSet;
    constraint_node node;
    for(uint32_t i = 0; i < start_positions.size(); i ++){
        enqueue_astar( start_positions.at(i),target_positions.at(i), node, i);
    }

    //Wait for all threads to termiante
   
    std::cout << "[INFO CBS] All threads returned" << std::endl;
    // Compute SIC by hop count
    
    await_astar_result(start_positions.size());

    uint64_t sic = 0;
    for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
        
        if(!m_lowLevelResults.at(i).found_path){
            std::cout << "CBS INFEASIBLE due to not finding initial path" << std::endl;
            return constraint_node();
        }
        sic += m_lowLevelResults.at(i).path->size();
    }
    // Create initial constraint node
    rlog("cbs", LOG_INFO, "All results feasible, now starting main cbs iterations.");

    node.sic = sic;
    node.node_id = node_id;
    node.avoid.clear();
    for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
        node.result[m_lowLevelResults.at(i).car_id] = m_lowLevelResults.at(i);
    }

    m_lowLevelResults.clear();
    m_lowLevelJobs.clear();
    
    std::cout << "[INFO CBS] Finished intial low level phase for all vehicles" << std::endl;


    // Insert initial constraint node into list
    openSet.push(node);
    while(!openSet.empty()){
        constraint_node node = openSet.top();
        openSet.pop();

        
        std::cout << "[INFO CBS] Iteration, openSet: " << openSet.size() << std::endl;
        std::cout << "[INFO CBS] SIC: " << node.sic << std::endl;
        std::cout << "[INFO CBS] Current Node ID: " << node.node_id << std::endl;
        std::cout << "[INFO CBS] Obstacles:";
        for(auto obst: node.avoid){
            std::cout << "[" + std::to_string(obst.x) + ":" + std::to_string(obst.y) + ":" + std::to_string(obst.t) + "],";
        }
        std::cout << std::endl;

        //Validate Solution in first node to find conflicts
        std::array<dynamics::data::PoseByIndex,2> conflicting_poses; 
        int32_t conflict_step = -1;
        std::array<int32_t,2> conflicting_vehicles = {-1,-1};

        for(int32_t car_index = 0; car_index < start_positions.size(); car_index ++){
            for (int32_t car_index2 = car_index + 1; car_index2 < start_positions.size(); car_index2 ++){
                
                for(int32_t i = 1; i < std::max(node.result[car_index].path->size(), node.result[car_index2].path->size()); i ++){
                
                    int32_t path_index_car_a = node.result[car_index].path->size() - std::min(i, (int32_t)node.result[car_index].path->size());
                    int32_t path_index_car_b = node.result[car_index2].path->size() - std::min(i, (int32_t)node.result[car_index2].path->size());

                    auto pose_car_a = indexToPose(node.result[car_index].path->at(path_index_car_a));
                    auto pose_car_b = indexToPose(node.result[car_index2].path->at(path_index_car_b));
                    
                    //Level 1: Detection of close intersection
                    if((pose_car_a.pos - pose_car_b.pos).norm() < safe_radius){
                        
                         rlog("CBS", LOG_INFO, "Found L1 Conflict... A " + std::to_string(path_index_car_a) + ":" + std::to_string(node.result[car_index].path->size()), FCONFLICT);
                         rlog("CBS", LOG_INFO, "Pose A " + std::to_string(pose_car_a.pos[0]) + ":" + std::to_string(pose_car_a.pos[1]), FCONFLICT);
                         rlog("CBS", LOG_INFO, "Found L1 Conflict... B " + std::to_string(path_index_car_b) + ":" + std::to_string(node.result[car_index2].path->size()), FCONFLICT);
                         rlog("CBS", LOG_INFO, "Pose B " + std::to_string(pose_car_b.pos[0]) + ":" + std::to_string(pose_car_b.pos[1]), FCONFLICT);

                        //Level 2: Switch to denser representation
                        for(int32_t interprim = 20 * (i - 1); interprim < std::max( node.result[car_index].interprimitive.size(), node.result[car_index2].interprimitive.size()) &&
                                                              interprim < 20 * (i + 1); interprim ++ ){

            
                            int32_t inter_path_index_car_a = node.result[car_index].interprimitive.size()  - std::max(1, std::min(interprim, (int32_t)node.result[car_index].interprimitive.size()));
                            int32_t inter_path_index_car_b = node.result[car_index2].interprimitive.size() - std::max(1, std::min(interprim, (int32_t)node.result[car_index2].interprimitive.size()));

                            rlog("CBS", LOG_INFO, "Testing L2 Conflict... A " + std::to_string(inter_path_index_car_a) + ":" + std::to_string(node.result[car_index].interprimitive.size()), FCONFLICT);
                            rlog("CBS", LOG_INFO, "Testing L2 Conflict... B " + std::to_string(inter_path_index_car_b) + ":" + std::to_string(node.result[car_index2].interprimitive.size()), FCONFLICT);

                            auto inter_pose_car_a = node.result[car_index].interprimitive.at(inter_path_index_car_a);
                            auto inter_pose_car_b = node.result[car_index2].interprimitive.at(inter_path_index_car_b); 

                            rlog("CBS", LOG_INFO, "Pose L2 A " + std::to_string(inter_pose_car_a.pos[0]) + ":" + std::to_string(inter_pose_car_a.pos[1]), FCONFLICT);
                            rlog("CBS", LOG_INFO, "Pose L2 B " + std::to_string(inter_pose_car_b.pos[0]) + ":" + std::to_string(inter_pose_car_b.pos[1]), FCONFLICT);

                            if((inter_pose_car_a.pos - inter_pose_car_b.pos).norm() < safe_level2_rad){

                                conflict_step = i;
                                conflicting_vehicles = {car_index, car_index2};
                                conflicting_poses = {node.result[car_index].path->at(path_index_car_a), node.result[car_index2].path->at(path_index_car_b)};
                                
                                rlog("CBS", LOG_INFO, "Found L2 Conflict: " + std::to_string(conflict_step) + " with " +  std::to_string(conflicting_vehicles[0]) + ":" +  std::to_string(conflicting_vehicles[1]), FCONFLICT);
                                rlog("CBS", LOG_INFO, "Position A: " + std::to_string(node.result[car_index2].path->at(path_index_car_b).x) + ":" + std::to_string(node.result[car_index2].path->at(path_index_car_b).y) , FCONFLICT);
                                rlog("CBS", LOG_INFO, "Position B: " + std::to_string(node.result[car_index].path->at(path_index_car_a).x) + ":" +  std::to_string(node.result[car_index].path->at(path_index_car_a).y) , FCONFLICT);                                               
                            
                            }
                        }
                    }
                }   
            }
        }

        
        // Check if we have just obtained a valid solution -> terminate if yes
        if(conflict_step == -1){
            std::cout << "CBS SUCCESSFULL TERMINATION!" << std::endl;
            return node;
        }

        // Add new constraints and replan
        for(uint32_t vehicle_index = 0; vehicle_index < 2; vehicle_index ++){

            auto vehicle_in_conflict = conflicting_vehicles[vehicle_index];
            auto vehicle_conflict = conflicting_poses[vehicle_index];

            // Create new constraint node
            constraint_node constraint;
            constraint.sic = 0;

            dynamics::data::PBIConstraint constr;
            constr.id = vehicle_in_conflict;
            constr = vehicle_conflict;
            constr.t = conflict_step;

            std::cout << "[INFO CBS] Adding new constraint to situation: " << vehicle_conflict.x << ":" << vehicle_conflict.y << ":" << conflict_step << ":" << vehicle_in_conflict<< std::endl;

              if(std::find(node.avoid.begin(), node.avoid.end(), constr) == node.avoid.end()){
                constraint.avoid.push_back(constr);
            }else{
                rlog("CBS", LOG_ERROR, " Readding same conflict.... this should not happen", FCONFLICT);
            }
            
            constraint.avoid.insert(constraint.avoid.end(), node.avoid.begin(), node.avoid.end());
            

            // TODO: maybe restore astar compute from here
            // Just add path again until conflict with all neighbors as open nodes and recompute scores
            // potentially also adjust the heuristict to account for new obstacle

            // Enqueu all jobs to astar workers
            m_lowLevelResults.clear();

            for(uint32_t i = 0; i < start_positions.size(); i ++){
                enqueue_astar(start_positions.at(i), target_positions.at(i), constraint, i);
            }

            //Wait for all threads to termiante
            await_astar_result(start_positions.size());

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
            constraint.result.clear();
            constraint.node_id = node_id;
            node_id ++;

            for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
                constraint.result[m_lowLevelResults.at(i).car_id]= m_lowLevelResults.at(i);
            }
            m_lowLevelJobs.clear();

            // Add new constraint node if it is feasible
            if(found_paths_for_all_vehicles){
                openSet.push(constraint);
                rlog("CBS", LOG_INFO, "Adding new node with ID: " + std::to_string(constraint.node_id), FCONFLICT);
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
    std::cout << "CBS EMPTY OPEN SET... infeasible" << std::endl;
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

    cbs_paths["sizex"] = map_size_x;
    cbs_paths["sizey"] = map_size_y;

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