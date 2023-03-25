#include <Planner/CBSPlanner.hpp>
#include <Planner/Logger.hpp>

#include <nlohmann/json.hpp>
#include <fstream>


#include <queue>

using json = nlohmann::json;


ReachabilityResult CBSPlanner::checkForReachability(){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float xpc = Config::getInstance().get<float>({"disc","xstep"});
    float ypc = Config::getInstance().get<float>({"disc","ystep"});
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t x_steps = Config::getInstance().get<int32_t>({"map","x_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

    
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

    // Assumption 1: Every node for a full level is reachable
    uint32_t job_count = 0;
    for(int32_t v = 0; v < map_size_speed; v ++){
       
            
            for(int32_t tx = - reachable_node_count; tx < reachable_node_count; tx ++){
                rlog("ReachCheck", LOG_INFO, "Checking reachability for speed: " + std::to_string(v) );
                for(int32_t ty = - reachable_node_count; ty < reachable_node_count; ty ++){

                    for(int32_t ta = 0; ta < map_size_angle; ta ++){
                        for(int32_t sh = 0; sh < map_size_angle; sh ++){
                    
                            dynamics::data::PoseByIndex start = {0,0,sh,v};
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


void CBSPlanner::writeMultiplePathsToDisk(constraint_node cnode, std::string name){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float xpc = Config::getInstance().get<float>({"disc","xstep"});
    float ypc = Config::getInstance().get<float>({"disc","ystep"});
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t x_steps = Config::getInstance().get<int32_t>({"map","x_steps"});
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

constraint_node CBSPlanner::cbs(std::vector<dynamics::data::PoseByIndex> start_positions, std::vector<dynamics::data::PoseByIndex> target_positions){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float xpc = Config::getInstance().get<float>({"disc","xstep"});
    float ypc = Config::getInstance().get<float>({"disc","ystep"});
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

   
    std::cout << "CBS start" << std::endl;
    uint32_t node_id = 0;

    for(uint32_t i = 0; i < worker_count; i ++){
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

        writeConstraintNodeToDisk(node,"node" + std::to_string(node.node_id) + ".json");
        std::cout << std::endl;

        //Validate Solution in first node to find conflicts
        std::array<dynamics::data::PoseByIndex,2> conflicting_poses; 
        int32_t conflict_step = -1;
        std::array<int32_t,2> conflicting_vehicles = {-1,-1};

        // for(int32_t car_index = 0; car_index < start_positions.size(); car_index ++){
        //     for (int32_t car_index2 = car_index + 1; car_index2 < start_positions.size(); car_index2 ++){
                
        //         uint32_t max_interprim_length = std::max(node.result[car_index].interprimitive.size(), node.result[car_index2].interprimitive.size());
        //         for(uint32_t inter_prim_index = 0; inter_prim_index < max_interprim_length; inter_prim_index ++){
                    
        //             uint32_t int_prim_index_clamped_car1 = std::min(inter_prim_index, (uint32_t) node.result[car_index].interprimitive.size() - 1);
        //             uint32_t int_prim_index_clamped_car2 = std::min(inter_prim_index, (uint32_t) node.result[car_index2].interprimitive.size() - 1);
                    
        //             auto inter_pose_car_1 = node.result[car_index].interprimitive.at(int_prim_index_clamped_car1);
        //             auto inter_pose_car_2 = node.result[car_index2].interprimitive.at(int_prim_index_clamped_car2); 

        //             if((inter_pose_car_1.pos - inter_pose_car_2.pos).norm() < safe_level2_rad){

        //                 conflict_step = node.result[car_index].interprimitive.at(int_prim_index_clamped_car1).time_index;
        //                 conflicting_vehicles = {car_index, car_index2};
        //                 conflicting_poses = { node.result[car_index].interprimitive.at(int_prim_index_clamped_car1).baseNode, node.result[car_index2].interprimitive.at(int_prim_index_clamped_car2).baseNode };
                        
        //                 rlog("CBS", LOG_INFO, "Found L2 Conflict: " + std::to_string(conflict_step) + " with " +  std::to_string(conflicting_vehicles[0]) + ":" +  std::to_string(conflicting_vehicles[1]));
        //                 rlog("CBS", LOG_INFO, "Position A: " + std::to_string(node.result[car_index].interprimitive.at(int_prim_index_clamped_car1).pos[0]) + ":" + std::to_string(node.result[car_index].interprimitive.at(int_prim_index_clamped_car1).pos[1]) );
        //                 rlog("CBS", LOG_INFO, "Position B: " + std::to_string(node.result[car_index2].interprimitive.at(int_prim_index_clamped_car2).pos[0]) + ":" +  std::to_string(node.result[car_index2].interprimitive.at(int_prim_index_clamped_car2).pos[1]) );       
        //                 rlog("CBS", LOG_INFO, "Base Node: " + std::to_string(node.result[car_index2].interprimitive.at(int_prim_index_clamped_car2).baseNode.x) + ":" +  std::to_string(node.result[car_index2].interprimitive.at(int_prim_index_clamped_car2).baseNode.y) );                                                                                       
        //                 break;

        //             }
        //         }

                // for(int32_t i = 1; i < std::max(node.result[car_index].path->size(), node.result[car_index2].path->size()); i ++){
                    


                //     int32_t path_index_car_a = node.result[car_index].path->size() - std::min(i, (int32_t)node.result[car_index].path->size());
                //     int32_t path_index_car_b = node.result[car_index2].path->size() - std::min(i, (int32_t)node.result[car_index2].path->size());

                //     auto pose_car_a = indexToPose(node.result[car_index].path->at(path_index_car_a));
                //     auto pose_car_b = indexToPose(node.result[car_index2].path->at(path_index_car_b));
                    
                //     //Level 1: Detection of close intersection
                //     if((pose_car_a.pos - pose_car_b.pos).norm() < safe_radius){
                        
                //          rlog("CBS", LOG_INFO, "Found L1 Conflict... A " + std::to_string(path_index_car_a) + ":" + std::to_string(node.result[car_index].path->size()));
                //          rlog("CBS", LOG_INFO, "Pose A " + std::to_string(pose_car_a.pos[0]) + ":" + std::to_string(pose_car_a.pos[1]));
                //          rlog("CBS", LOG_INFO, "Found L1 Conflict... B " + std::to_string(path_index_car_b) + ":" + std::to_string(node.result[car_index2].path->size()));
                //          rlog("CBS", LOG_INFO, "Pose B " + std::to_string(pose_car_b.pos[0]) + ":" + std::to_string(pose_car_b.pos[1]));

                //         //Level 2: Switch to denser representation
                //         for(int32_t interprim = 20 * (i - 1); interprim < std::max( node.result[car_index].interprimitive.size(), node.result[car_index2].interprimitive.size()) &&
                //                                               interprim < 20 * (i + 1); interprim ++ ){

            
                //             int32_t inter_path_index_car_a = node.result[car_index].interprimitive.size()  - std::max(1, std::min(interprim, (int32_t)node.result[car_index].interprimitive.size()));
                //             int32_t inter_path_index_car_b = node.result[car_index2].interprimitive.size() - std::max(1, std::min(interprim, (int32_t)node.result[car_index2].interprimitive.size()));

                //             rlog("CBS", LOG_INFO, "Testing L2 Conflict... A " + std::to_string(inter_path_index_car_a) + ":" + std::to_string(node.result[car_index].interprimitive.size()));
                //             rlog("CBS", LOG_INFO, "Testing L2 Conflict... B " + std::to_string(inter_path_index_car_b) + ":" + std::to_string(node.result[car_index2].interprimitive.size()));

                //             auto inter_pose_car_a = node.result[car_index].interprimitive.at(inter_path_index_car_a);
                //             auto inter_pose_car_b = node.result[car_index2].interprimitive.at(inter_path_index_car_b); 

                //             rlog("CBS", LOG_INFO, "Pose L2 A " + std::to_string(inter_pose_car_a.pos[0]) + ":" + std::to_string(inter_pose_car_a.pos[1]));
                //             rlog("CBS", LOG_INFO, "Pose L2 B " + std::to_string(inter_pose_car_b.pos[0]) + ":" + std::to_string(inter_pose_car_b.pos[1]));

                //             if((inter_pose_car_a.pos - inter_pose_car_b.pos).norm() < safe_level2_rad){

                                // conflict_step = i;
                                // conflicting_vehicles = {car_index, car_index2};
                                // conflicting_poses = {node.result[car_index].path->at(path_index_car_a ), node.result[car_index2].path->at(path_index_car_b)};
                                
                //                 rlog("CBS", LOG_INFO, "Found L2 Conflict: " + std::to_string(conflict_step) + " with " +  std::to_string(conflicting_vehicles[0]) + ":" +  std::to_string(conflicting_vehicles[1]));
                //                 rlog("CBS", LOG_INFO, "Position A: " + std::to_string(node.result[car_index2].path->at(path_index_car_b).x) + ":" + std::to_string(node.result[car_index2].path->at(path_index_car_b).y) );
                //                 rlog("CBS", LOG_INFO, "Position B: " + std::to_string(node.result[car_index].path->at(path_index_car_a).x) + ":" +  std::to_string(node.result[car_index].path->at(path_index_car_a).y) );                                               
                            
                //             }
                //         }
                //     }
                // }   
        //     }
        // }

        
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
                rlog("CBS", LOG_ERROR, " Readding same conflict.... this should not happen");
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
                rlog("CBS", LOG_INFO, "Adding new node with ID: " + std::to_string(constraint.node_id));
            }else{
                std::cout << "CBS Found infeasible node" << std::endl;
            }
        }
    }

    m_keepThreadsAlive = false;
    for(uint32_t i = 0; i < worker_count; i ++){
        m_lowLevelWorkers.at(i).join();
    }

    constraint_node dnode;
    std::cout << "CBS EMPTY OPEN SET... infeasible" << std::endl;
    return dnode;
}


/*  uint64_t sic = 0;
    uint32_t node_id = 0;
    std::vector<dynamics::data::PBIConstraint> avoid;
    std::map<int32_t, LLResult> result;
*/

void CBSPlanner::writeConstraintNodeToDisk(constraint_node cnode, std::string name){
    json node;
    rlog("CBS", LOG_INFO, "Writing CBS Constraint Node to disk...");

    node["sic"] = cnode.sic;
    node["node_id"] = cnode.node_id;

    for(auto constr: cnode.avoid){
        json jconstr;
        jconstr["x"] = indexToPose(constr).pos[0];
        jconstr["y"] = indexToPose(constr).pos[1];
        jconstr["id"] = constr.id;
        jconstr["t"] = constr.t;
        node["avoid"].push_back(jconstr);
    }

    for(auto vres: cnode.result){
        json path_vehicle;
        for(auto current: *vres.second.path){
            json pnode;
            pnode["x"] = indexToPose(current).pos[0];
            pnode["y"] = indexToPose(current).pos[1];
            pnode["s"] = indexToPose(current).vel;
            pnode["a"] = indexToPose(current).h;
            path_vehicle["path"].push_back(pnode);
        }

        for(auto current: vres.second.interprimitive){
            json pnode;
            pnode["x"] = current.pos[0];
            pnode["y"] = current.pos[1];
            pnode["s"] = current.vel;
            pnode["a"] = current.h;
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
