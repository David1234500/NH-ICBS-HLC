#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <Planner/Logger.hpp>

#include <cmath>
#include <iostream>
#include <thread>
#include <fstream>


#include <nlohmann/json.hpp>

using json = nlohmann::json;

ProxyGraph::ProxyGraph(){

}

void ProxyGraph::workerThreadProxyEdges(uint32_t index){
    while(!m_terminateProxyThreads){
        
        bool hasTask = false;
        ProxyTask threadTask;

        m_proxyTaskMutex.lock();
        if(!m_proxyTaskQueue.empty()){
            threadTask = *m_proxyTaskQueue.begin();
            m_proxyTaskQueue.erase(m_proxyTaskQueue.begin());
            rlog("workerProxyEdges", LOG_INFO," Working now on task: " + std::to_string(threadTask.txi) +":"+ std::to_string(threadTask.tyi) + ":"+ std::to_string(threadTask.cai) +":"+ std::to_string(threadTask.csi), MOTIONPRIM);
                    
            hasTask = true;
        }else{
            hasTask = false;
        }
        m_proxyTaskMutex.unlock();

        if(hasTask){
            
            dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * static_cast<float>(threadTask.cai), m_speedsFactor[threadTask.csi] * dynamics::SimpleDynamicsModel::velocity_limit()};
           
                for(int32_t a = 0; a < map_size_angle; a ++){
                
                    //Compute best fit settings set to get from the current to the target location
                    dynamics::data::PoseByIndex target_pose_by_index = {threadTask.txi,threadTask.tyi, a, threadTask.tsi};
                    dynamics::data::Pose2D target_pose = {{-(m_proxyMapCarOffset * xpc) + (xpc*threadTask.txi), -(m_proxyMapCarOffset * ypc) + (ypc*threadTask.tyi)},  api * static_cast<float>(a), m_speedsFactor[threadTask.tsi] * dynamics::SimpleDynamicsModel::velocity_limit()};


                    double speed_delta = state_change_fit_allowed_speed_difference;
                    if(threadTask.isIntermediate){
                        speed_delta += 0.1;
                    }
                    auto epose = dynamics::SimpleDynamicsModel::computeBestFit(veh_pose, target_pose_by_index, target_pose, threadTask.tstep, speed_delta);
                    
                    //Discard if error greater than half of the angular resolution 
                    if(epose.a_error > state_change_fit_quality_angle){
                        continue;
                    }
                    if(epose.p_error > state_change_fit_quality_position){
                        continue;
                    }

                    // Found a link to the neighbor, add an edge for this one
                    TraversableEdge nt_edge = {epose, target_pose_by_index};
                    
                    rlog("workerProxyEdges", LOG_INFO,"Added edge to " + std::to_string(epose.pos[0]) +":"+ std::to_string(epose.pos[1]) 
                    + " -> s" + std::to_string(epose.s_a) +"_"+ std::to_string(epose.s_v) + "e" + std::to_string(epose.p_error) +  "p" + std::to_string(a) +"_"+ std::to_string(threadTask.tsi), MOTIONPRIM);
                    
                    // add new edge to the edgelist
                    m_proxyTaskMutex.lock();
                    m_proxyEdgeList[threadTask.cai][threadTask.csi].push_back(nt_edge);  
                    m_proxyTaskMutex.unlock();
                
            }
    
        }else{
            return;
        }
        hasTask = false;
    }
}

void ProxyGraph::computeProxyEdges(){
    float reachable_distance = dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<float>(timestep_ms) / 1000.f);

    //Compute size of the proxy field based on reachable distance at the given speed
    uint32_t reachable_node_count = static_cast<uint32_t>(reachable_distance / xpc);
    uint32_t reachable_node_span = (2 * reachable_node_count); 
    m_proxyMapReachableSpan = reachable_node_span;
    m_proxyMapCarOffset = reachable_node_count; 

    std::cout << "[INFO] Created all proxy nodes: [#] " << reachable_node_span * reachable_node_span * map_size_angle * map_size_speed << std::endl;
    std::cout << "[INFO] Reachable distance: [cm] " << reachable_distance << std::endl;
    std::cout << "[INFO] X Extends: [cm] " << -(reachable_node_count * xpc) << " - " <<  -(reachable_node_count * xpc) + (xpc*reachable_node_span) << " stp:" << xpc << " cnt:" << reachable_node_span << std::endl;
    std::cout << "[INFO] Y Extends: [cm] " << -(reachable_node_count * ypc) << " - " <<  -(reachable_node_count * ypc) + (ypc*reachable_node_span) << " stp:" << ypc << " cnt:" << reachable_node_span << std::endl;
    std::cout << "[INFO] A Extends: [rad] " << 0 << " - " <<  api * map_size_angle << " stp:" << api << " cnt:" << map_size_angle << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Compute for each heading and speed from our original vehicle
    for(int32_t sv = 0; sv < map_size_speed; sv ++){
        for(int32_t sh = 0; sh < map_size_angle; sh ++){
            for(int32_t ts = std::max(0, sv - 1); ts < std::min(map_size_speed, sv + 1); ts ++){

                //Compute pose of current node/vehicle
                dynamics::data::Pose2D veh_pose = {{0.f,0.f},  api * static_cast<float>(sh),m_speedsFactor[sv] * dynamics::SimpleDynamicsModel::velocity_limit()};

                //Check for each candidate node if we can find a connection between these two, but use all system threads
                for(int32_t x = 0; x <= reachable_node_span; x ++){
                    for(int32_t y = 0; y <= reachable_node_span; y ++){
                            
                        if(x == m_proxyMapCarOffset && y == m_proxyMapCarOffset){
                            continue;
                        }

                        if(sv == zero_velocity_level && ts == zero_velocity_level){
                            continue;
                        }

                        ProxyTask nTask;    
                        nTask = {x,y,ts,sh,sv,timestep_ms, m_speedFactorIntermediate[ts] || m_speedFactorIntermediate[sv]};
                        m_proxyTaskQueue.push_back(nTask);
                    }
                }  
            } 
        }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::vector<std::thread> workers;
    for(uint32_t i = 0; i < worker_counter; i++){
        workers.push_back(std::thread(&ProxyGraph::workerThreadProxyEdges, this, i));
    }
    for(uint32_t i = 0; i < worker_counter; i++){
        workers.at(i).join();
    }
}


void ProxyGraph::writeGraphToDisk(std::string name ){
   json proxy_map_dump;

    proxy_map_dump["info"]["m_proxyMapReachableSpan"] = m_proxyMapReachableSpan;
    proxy_map_dump["info"]["m_proxyMapCarOffset"] = m_proxyMapCarOffset;
    proxy_map_dump["info"]["size_a"] = map_size_angle;
    proxy_map_dump["info"]["size_s"] = map_size_speed;

    // Dump array to list
    for(uint32_t x = 0; x <= m_proxyMapReachableSpan; x ++){
        for(uint32_t y = 0; y <= m_proxyMapReachableSpan; y ++){
            for(uint32_t a = 0; a < map_size_angle; a ++){
                for(uint32_t s = 0; s < map_size_speed; s ++){
                    //Compute position, heading and velocity of the node
                    json node;
                    node["pose"]["x"] =  -(m_proxyMapCarOffset * xpc) + (xpc*x);
                    node["pose"]["y"] =  -(m_proxyMapCarOffset * ypc) + (ypc*y);
                    node["pose"]["h"] =  api * static_cast<float>(a);
                    node["pose"]["v"] =  m_speedsFactor[s] * dynamics::SimpleDynamicsModel::velocity_limit();

                    node["pose"]["xi"] = x;
                    node["pose"]["yi"] = y;
                    node["pose"]["ai"] = a;
                    node["pose"]["si"] = s;

                    proxy_map_dump["map"].push_back(node);
                }
            }
        }
    }

    for(uint32_t i = 0; i < map_size_angle; i ++){
        for(uint32_t j = 0; j < map_size_speed; j ++){
            for(auto edge: m_proxyEdgeList[i][j]){
                json jedge;
                jedge["source"]["a"] = i;
                jedge["source"]["s"] = j;
            
                dynamics::data::Pose2D next_pose;
                dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * i, edge.link.vel};
                for(float ts = 0; ts <= timestep_ms; ts += 25.f){
                
                    
                    next_pose = dynamics::SimpleDynamicsModel::computeNextPose(veh_pose, edge.link.s_a, edge.link.s_v, ts);
                    
                    json point;
                    point["x"] = next_pose.pos[0];
                    point["y"] = next_pose.pos[1];
                    jedge["curve"].push_back(point);

                }
                
                // veh_pose = {{0.f,0.f}, api * i, edge.link.vel};
                for(float ts = 0; ts <= timestep_ms; ts += 25.f){
                

                    auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPose(next_pose, edge.link.s_a_2, edge.link.s_v_2, ts);
                    
                    json point;
                    point["x"] = next_pose2.pos[0];
                    point["y"] = next_pose2.pos[1];
                    jedge["curve"].push_back(point);
                }

                jedge["target"]["x"] =  -(m_proxyMapCarOffset * xpc) + (xpc*edge.target.x);
                jedge["target"]["y"] =  -(m_proxyMapCarOffset * ypc) + (ypc*edge.target.y);
                jedge["target"]["s"] =  m_speedsFactor[edge.target.s] * dynamics::SimpleDynamicsModel::velocity_limit();
                jedge["target"]["a"] = api * static_cast<float>(edge.target.a);

                jedge["settings"]["a1"] = edge.link.s_a;
                jedge["settings"]["a2"] = edge.link.s_a_2;
                jedge["settings"]["v1"] = edge.link.s_v;
                jedge["settings"]["v2"] = edge.link.s_v_2;

                jedge["error"]["ae"] = edge.link.a_error;
                jedge["error"]["pe"] = edge.link.p_error;

                jedge["targeti"]["s"] = edge.target.s;
                jedge["targeti"]["a"] = edge.target.a;
                jedge["targeti"]["x"] = edge.target.x;
                jedge["targeti"]["y"] = edge.target.y;

                proxy_map_dump["edges"].push_back(jedge);
            }
        }
    }

    //dump file to disc
    std::ofstream o(name);
    o << proxy_map_dump << std::endl;
    o.close();
}

void ProxyGraph::loadGraphFromDisk(){
    loadGraphFromDisk("proxy_state_graph.json");
}

void ProxyGraph::loadGraphFromDisk(std::string path){
    std::ifstream ifs(path);
    json jf = json::parse(ifs);
    m_proxyMapReachableSpan = jf["info"]["m_proxyMapReachableSpan"];
    m_proxyMapReachableSpan += 1;
    m_proxyMapCarOffset = jf["info"]["m_proxyMapCarOffset"];

    std::cout << "Loading map with: " << jf["map"].size() << " and expected " << m_proxyMapReachableSpan * m_proxyMapReachableSpan * map_size_angle * map_size_speed <<std::endl;
    assert(jf["map"].size() == m_proxyMapReachableSpan * m_proxyMapReachableSpan * map_size_angle * map_size_speed);
    uint32_t index = 0;

    for(auto edge: jf["edges"]){
        TraversableEdge tedge;

        tedge.link.s_a = edge["settings"]["a1"];
        tedge.link.s_a_2 = edge["settings"]["a2"];
        tedge.link.s_v = edge["settings"]["v1"];
        tedge.link.s_v_2 = edge["settings"]["v2"];

        tedge.link.a_error = edge["error"]["ae"];
        tedge.link.p_error = edge["error"]["pe"];

        tedge.link.h = edge["target"]["a"];
        tedge.link.pos[1] = edge["target"]["y"];
        tedge.link.pos[0] = edge["target"]["x"];
        tedge.link.vel = edge["target"]["s"];

        tedge.target.s = edge["targeti"]["s"];
        tedge.target.a = edge["targeti"]["a"];
        tedge.target.x = edge["targeti"]["x"];
        tedge.target.y = edge["targeti"]["y"];

        m_proxyEdgeList[edge["source"]["a"]][edge["source"]["s"]].push_back(tedge);
    }
}
