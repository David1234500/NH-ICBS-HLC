#include <Planner/CBSPlanner.hpp>
#include <Planner/Logger.hpp>
#include <Collision/CircleApproximation.hpp>
#include <Collision/CollisionDetectBase.hpp> 
#include <nlohmann/json.hpp>
#include <fstream>


#include <queue>

using json = nlohmann::json;


ReachabilityResult CBSPlanner::checkForReachability(){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t x_steps = Config::getInstance().getXstep();
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

    
    ReachabilityResult result;
    result.reachable = true;

    float reachable_distance = dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<float>(timestep_ms) / 1000.f);
    int32_t reachable_node_count = static_cast<uint32_t>(reachable_distance / dpc);

    uint32_t count = 0;
    uint64_t path_length = 0;
    float path_time = 0.f;

    rlog("ReachCheck", LOG_INFO, "Checking reachability with Span: " + std::to_string(reachable_node_count) + ":" + std::to_string(reachable_distance) );

    for(uint32_t i =0; i < map_size_angle; i ++){
        for(uint32_t j = 0; j < map_size_speed; j ++){
            result.edge_count += mp_comp.m_mpmap[i][j].size();
        }
    }

    // Assumption 2: We can leave each configuration through at least one edge (so can always start moving)
    rlog("ReachCheck", LOG_INFO, "Checking for unleavable start configurations...");
    for(int32_t i = 0; i < map_size_angle; i ++){
        for(int32_t j = zero_velocity_level; j < map_size_speed; j ++){
            if(mp_comp.m_mpmap[i][j].empty()){
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
        for(int32_t ts = zero_velocity_level; ts < map_size_speed; ts ++){
            bool found_link = false;

            for(int32_t sh = 0; sh < map_size_angle && !found_link; sh ++ ){
                for(int32_t sv = zero_velocity_level; sv < map_size_speed && !found_link; sv ++){
                    for(auto prim: mp_comp.m_mpmap[sh][sv]){
                        
                        if(ts == 0 && sv == 0){
                            found_link = true;
                            break;
                        }

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

    for(uint32_t i = 0; i < worker_count; i ++){
        m_lowLevelWorkers.push_back(std::thread(&CBSPlanner::low_level_astar_worker, this, i));
    }

    std::map<uint64_t, std::pair<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>> jobs;
    // Assumption 1: Every node for a full level is reachable
    uint32_t job_count = 0;
    for(int32_t v = 2; v < map_size_speed; v ++){
       
            
            for(int32_t tx = -reachable_node_count/2; tx < reachable_node_count/2; tx ++){
                rlog("ReachCheck", LOG_INFO, "Checking reachability for speed: " + std::to_string(v) );
                for(int32_t ty = -reachable_node_count/2; ty < reachable_node_count/2; ty ++){

                    for(int32_t ta = 0; ta < map_size_angle; ta ++){
                        for(int32_t sh = 0; sh < map_size_angle; sh ++){
                    
                            dynamics::data::PoseByIndex start = {30,30,sh,v};
                            dynamics::data::PoseByIndex target = {tx,ty,ta,v};
                            auto global = toGlobalIndex(start, target);
                           
                            jobs[job_count] = std::make_pair(start,global);
                            rlog("ReachCheck", LOG_INFO, "Checking reachability for position: " + std::to_string(tx) + ":" + std::to_string(ty) + ":" + std::to_string(ta));
                            
                            constraint_node node;
                            enqueue_astar( start, global, node, job_count);
                            job_count += 1;
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
            result.unreachable_configurations.push_back(jobs[res.job_id]);
        }
    }

    result.mean_time_length = path_time/static_cast<float>(count);
    result.mean_path_length = static_cast<float>(path_length)/static_cast<float>(count);
    return result;
}


void CBSPlanner::writeMultiplePathsToDisk(constraint_node cnode, std::string name){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t x_steps = Config::getInstance().getXstep();
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

    
    json cbs_paths;

    cbs_paths["sizex"] = x_steps;
    cbs_paths["sizey"] = x_steps;

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

CollisionInfo checkCollisionWithinConstraintNode(const constraint_node& constraintNode) {
    CircleApproximation collisionChecker;

    for (auto it1 = constraintNode.result.begin(); it1 != constraintNode.result.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != constraintNode.result.end(); ++it2) {
            const LLResult& llResult1 = it1->second;
            const LLResult& llResult2 = it2->second;

            CollisionInfo collisionInfo = collisionChecker.checkForCollision(llResult1.interprimitive, llResult2.interprimitive);
            if (collisionInfo.collision_occurred) {
                collisionInfo.car_index_1 = it1->first;
                collisionInfo.car_index_2 = it2->first;
                
                return collisionInfo;
            }
        }
    }

    return CollisionInfo(); // Return an empty CollisionInfo object if no collision occurred
}

constraint_node CBSPlanner::cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});
    static bool col_deb = Config::getInstance().get<bool>({"collision_detect","debug_mode"});
   
    rlog("CBS", LOG_DEBUG, "CBS start");
    uint32_t node_id = 0;

    m_keepThreadsAlive = true;
    for(uint32_t i = 0; i < worker_count; i ++){
        m_lowLevelWorkers.push_back(std::thread(&CBSPlanner::low_level_astar_worker, this, i));
    }

    // Enqueu all jobs to astar workers
    std::priority_queue<constraint_node> openSet;
    constraint_node node;
    for(uint32_t i = 0; i < start_positions.size(); i ++){
        enqueue_astar(start_positions.at(i),target_positions.at(i), node, i);
    }

    //Wait for all threads to termiante
    rlog("CBS", LOG_DEBUG, "All threads returned");
    await_astar_result(start_positions.size());

    uint64_t sic = 0;
    for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
        
        if(!m_lowLevelResults.at(i).found_path){
             rlog("CBS", LOG_ERROR, "CBS INFEASIBLE due to not finding initial path");
            return constraint_node();
        }
        sic += m_lowLevelResults.at(i).path->size();
    }
    // Create initial constraint node
    rlog("cbs", LOG_DEBUG, "All results feasible, now starting main cbs iterations.");

    node.sic = sic;
    node.node_id = node_id;
    node.father = 0;
    node.avoid.clear();
    for(auto res: m_lowLevelResults){
        node.result[res.car_id] = res;
    }

    m_lowLevelResults.clear();
    m_lowLevelJobs.clear();
    
   rlog("CBS", LOG_DEBUG, "Finished initial low level phase for all vehicles");


    // Insert initial constraint node into list
    openSet.push(node);
    while(!openSet.empty()){
        constraint_node node = openSet.top();
        openSet.pop();
        
        rlog("CBS", LOG_DEBUG, "Iteration, openSet: " + std::to_string(openSet.size()));
        rlog("CBS", LOG_DEBUG, "SIC: " + std::to_string(node.sic));
        rlog("CBS", LOG_DEBUG, "Current Node ID: " + std::to_string(node.node_id));
        rlog("CBS", LOG_DEBUG, "Conflict count: " + std::to_string(node.avoid.size()));
        
        if(col_deb){
            writeConstraintNodeToDisk(node,"node" + std::to_string(node.node_id) + ".json");
        }
        auto info = checkCollisionWithinConstraintNode(node);
       
        
        // Check if we have just obtained a valid solution -> terminate if yes
        if(!info.collision_occurred){
            rlog("CBS", LOG_DEBUG, "CBS SUCCESSFULL TERMINATION!");
            node.feasible = true;
            return node;
        }else{
            rlog("CBS", LOG_DEBUG, "CBS found conflict");
            if(col_deb){
                CollisionDetectBase::print_collision_info(info);
            }
        }

        // Add new constraints and replan
        for(uint32_t vehicle_index = 0; vehicle_index < 2; vehicle_index ++){

            // Create new constraint node
            constraint_node constraint;
            constraint.sic = 0;

            if(vehicle_index == 0){
                  
                // if(info.collision_with_veh_at_track_end == 1){
                //     continue;
                // }

                dynamics::data::PBIConstraint constr;
                constr.id = info.car_index_1;
                constr = info.pose1bi;
                constr.t = info.index1;
                constraint.avoid = node.avoid;

                if(info.collision_with_veh_at_track_end == 1){
                   constr.t = -1;
                }

                if(std::find(node.avoid.begin(), node.avoid.end(), constr) != node.avoid.end()){
                    constr.t = -1;
                }

                constraint.avoid.push_back(constr);
            }else{

                dynamics::data::PBIConstraint constr2;
                constr2.id = info.car_index_2;
                constr2 = info.pose2bi;
                constr2.t = info.index2;
                constraint.avoid = node.avoid;
                
                if(info.collision_with_veh_at_track_end == 2){
                    constr2.t = -1;
                }

                if(std::find(node.avoid.begin(), node.avoid.end(), constr2) != node.avoid.end()){
                    constr2.t = -1;
                }

                constraint.avoid.push_back(constr2); 
            }
            
            
            // Enqueue all jobs to astar workers
            m_lowLevelResults.clear();
            m_lowLevelJobs.clear();

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
                    sic += m_lowLevelResults.at(i).interprimitive.size();
                }else{
                    found_paths_for_all_vehicles = false;
                }
            }

            // Update constraint node with new information
            constraint.sic = sic;
            constraint.result.clear();
            constraint.node_id = node_id;
            constraint.father = node.node_id;
            node_id ++;

            for(uint32_t i = 0; i < m_lowLevelResults.size(); i ++){
                constraint.result[m_lowLevelResults.at(i).car_id]= m_lowLevelResults.at(i);
            }
            m_lowLevelResults.clear();
            m_lowLevelJobs.clear();

            // Add new constraint node if it is feasible
            if(found_paths_for_all_vehicles){
                openSet.push(constraint);
                rlog("CBS", LOG_DEBUG, "Adding new node with ID: " + std::to_string(constraint.node_id));
            }else{
                rlog("CBS", LOG_DEBUG, "CBS Found infeasible node");
            }
            if(vehicle_index == 0){
                info.one_feasible = found_paths_for_all_vehicles;
            }else{
                info.two_feasible = found_paths_for_all_vehicles;
            }
        }

        if(col_deb){
            writeCollisionInfoToDisc(info, node.result[info.car_index_1].interprimitive, node.result[info.car_index_2].interprimitive, node, "collision_in_node" + std::to_string(node.node_id) + ".json" );
        }
    }

    m_keepThreadsAlive = false;
    for(uint32_t i = 0; i < worker_count; i ++){
        m_lowLevelWorkers.at(i).join();
    }

    m_lowLevelResults.clear();
    m_lowLevelJobs.clear();

    constraint_node dnode;
    rlog("CBS", LOG_ERROR, "CBS EMPTY OPEN SET... infeasible");
    return dnode;
}


void CBSPlanner::writeConstraintNodeToDisk(constraint_node cnode, std::string name){
    json node;
    rlog("CBS", LOG_DEBUG, "Writing CBS Constraint Node to disk...");

    node["sic"] = cnode.sic;
    node["node_id"] = cnode.node_id;
    node["father"] = cnode.father;


    //Known conflicts to avoid (obstacles in next Conflict based search iteration)
    for(auto constr: cnode.avoid){
        json jconstr;
        jconstr["x"] = indexToPose(constr).pos[0];
        jconstr["y"] = indexToPose(constr).pos[1];
        jconstr["id"] = constr.id;
        jconstr["t"] = constr.t;
        node["avoid"].push_back(jconstr);
    }
    
    //Vehicle paths 
    for(auto vres: cnode.result){
        json path_vehicle;

        // first by visited nodes
        for(auto current: *vres.second.path){
            json pnode;
            pnode["x"] = indexToPose(current).pos[0];
            pnode["y"] = indexToPose(current).pos[1];
            pnode["s"] = indexToPose(current).vel;
            pnode["a"] = indexToPose(current).h;
            path_vehicle["path"].push_back(pnode);
        }

        //secondly by precise positions
        for(auto current: vres.second.interprimitive){
            json pnode;
            pnode["x"] = current.pos[0];
            pnode["y"] = current.pos[1];
            pnode["s"] = current.vel;
            pnode["a"] = current.h;
            pnode["t"] = current.time_ms;
            pnode["ti"] = current.path_depth_index;

            pnode["bnode"]["x"] =  indexToPose(current.baseNode).pos[0];
            pnode["bnode"]["y"] =  indexToPose(current.baseNode).pos[1];
            pnode["bnode"]["a"] =  indexToPose(current.baseNode).vel;
            pnode["bnode"]["s"] =  indexToPose(current.baseNode).h;
            path_vehicle["interprimitive"].push_back(pnode);
        }

        path_vehicle["id"] = vres.first;
        node["multipath"].push_back(path_vehicle);
    }

    //dump file to disc
    std::ofstream o(name);
    o << node << std::endl;
    o.close();
}
