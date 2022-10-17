#include <Planner/DirectedSearchProxy.hpp>

#include <cmath>
#include <iostream>
#include <thread>
#include <fstream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

DirectedSearchProxy::DirectedSearchProxy(){}

void DirectedSearchProxy::workerThreadProxyEdges(uint32_t index){
    while(!m_terminateProxyThreads){
        
        bool hasTask = false;
        ProxyTask threadTask;

        m_proxyTaskMutex.lock();
        if(!m_proxyTaskQueue.empty()){
            threadTask = *m_proxyTaskQueue.begin();
            m_proxyTaskQueue.erase(m_proxyTaskQueue.begin());
            std::cout << "[INFO]["<< index <<"] Working now on task: " << threadTask.txi <<":"<< threadTask.tyi << ":"  << threadTask.cai <<":"<< threadTask.csi << std::endl;
            hasTask = true;
        }else{
            hasTask = false;
        }
        m_proxyTaskMutex.unlock();

        if(hasTask){
            
            dynamics::data::Pose2D veh_pose = {{0.f,0.f}, m_proxyMap[0][0][threadTask.cai][threadTask.csi].rel_pose.h,  m_proxyMap[0][0][threadTask.cai][threadTask.csi].rel_pose.vel};
            for(int32_t a = 0; a < map_size_angle; a ++){
                
                //Constraint speed changes to one level up, same or one level down [current_speed_level - 1, current_speed_level + 1]
                for(int32_t s = std::max(0, threadTask.csi - 1); s < std::min(map_size_speed, threadTask.csi + 1); s ++){
            
                    //Compute best fit settings set to get from the current to the target location
                    dynamics::data::PoseByIndex target_pose_by_index = {threadTask.txi,threadTask.tyi, a, s};
                    
                    auto epose = dynamics::SimpleDynamicsModel::computeBestFit(veh_pose, target_pose_by_index, m_proxyMap[threadTask.txi][threadTask.tyi][a][s].rel_pose, threadTask.tstep);
                    
                    //Discard if error greater than half of the angular resolution 
                    if(epose.a_error > state_change_fit_quality_angle){
                        continue;
                    }
                    if(epose.p_error > state_change_fit_quality_position){
                        continue;
                    }

                    // Found a link to the neighbor, add an edge for this one
                    TraversableEdge nt_edge = {epose, target_pose_by_index};
                    
                    std::cout << "[INFO]["<< index <<"] Added edge to " << std::to_string(epose.pos[0]) <<":"<< std::to_string(epose.pos[1]) 
                    << " >s" << epose.s_a <<"_"<< epose.s_v << "e" << epose.p_error <<  "p" << a <<"_"<< s << std::endl;
                    
                    // add new edge to the edgelist
                    m_proxyTaskMutex.lock();
                    m_proxyEdgeList[threadTask.cai][threadTask.csi].push_back(nt_edge);  
                    m_proxyTaskMutex.unlock();
                    
                }
            }

        }else{
            return;
        }
        hasTask = false;
    }
}

void DirectedSearchProxy::computeProxyEdges(){
    float base_node_distance = m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<float>(timestep_ms) / 1000.f);
    uint32_t map_node_count = static_cast<uint32_t>(map_size_x_cm / base_node_distance);
    float api = 2 * PI / static_cast<float>(m_config_map_size_angle);

    LatticeNode  lattice[m_config_map_size_speed][m_config_map_size_speed][map_size_angle][map_size_speed];

    for(int32_t x = -m_config_map_size_speed; x < m_config_map_size_speed; x ++){
        for(int32_t y = -m_config_map_size_speed; y < m_config_map_size_speed; y ++){
            for(int32_t a = 0; a < map_size_angle; a ++){
                for(int32_t s = 0; s < map_size_speed; s ++){

                    lattice[x][y][a][s].rel_pose.pos[0] = (base_node_distance*x);
                    lattice[x][y][a][s].rel_pose.pos[1] = (base_node_distance*y);
                    lattice[x][y][a][s].rel_pose.h = api * static_cast<float>(a);
                    lattice[x][y][a][s].rel_pose.vel = m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();
                
                }
            }
        }
    }

    //Angles that we need to check, assume symmetry for all points
    uint32_t map_angle_offset = m_config_map_size_angle / 4; 
    

    for(uint32_t a = 0; a < map_angle_offset; a ++){
        float heading = api * static_cast<float>(a);

        for(uint32_t s = 0; s < map_size_speed; s ++){
            float velocity = m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();
            
            auto epose = dynamics::SimpleDynamicsModel::computeBestFit(veh_pose, target_pose_by_index, m_proxyMap[threadTask.txi][threadTask.tyi][a][s].rel_pose, );

        }
        //m_proxyEdgeList[][]
    }
    

    

   

    // std::this_thread::sleep_for(std::chrono::seconds(1));
    

    // // Compute for each heading and speed from our original vehicle
    // for(int32_t i = 0; i < map_size_speed; i ++){
    //     for(int32_t j = 0; j < map_size_angle; j ++){
            
    //         //Compute pose of current node/vehicle
    //         dynamics::data::Pose2D veh_pose = {{0.f,0.f}, m_proxyMap[0][0][j][i].rel_pose.h, m_proxyMap[0][0][j][i].rel_pose.vel};

    //         //Check for each candidate node if we can find a connection between these two, but use all system threads
    //         for(int32_t x = 0; x <= reachable_node_span; x ++){
    //             for(int32_t y = 0; y <= reachable_node_span; y ++){
                     
    //                 if(x == m_proxyMapCarOffset && y == m_proxyMapCarOffset){
    //                     continue;
    //                 }

    //                 ProxyTask nTask;    
    //                 nTask = {x,y,j,i,timestep_ms};
    //                 m_proxyTaskQueue.push_back(nTask);
                
    //             }
    //         }   
    //     }
    // }

    // std::this_thread::sleep_for(std::chrono::seconds(1));

    // std::vector<std::thread> workers;
    // for(uint32_t i = 0; i < worker_counter; i++){
    //     workers.push_back(std::thread(&DirectedSearchProxy::workerThreadProxyEdges, this, i));
    // }
    // for(uint32_t i = 0; i < worker_counter; i++){
    //     workers.at(i).join();
    // }
}


void DirectedSearchProxy::writeGraphToDisk(){
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
                    node["pose"]["x"] =  m_proxyMap[x][y][a][s].rel_pose.pos[0];
                    node["pose"]["y"] =  m_proxyMap[x][y][a][s].rel_pose.pos[1];
                    node["pose"]["h"] =  m_proxyMap[x][y][a][s].rel_pose.h;
                    node["pose"]["v"] =  m_proxyMap[x][y][a][s].rel_pose.vel;

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
                for(float ts = 0; ts <= timestep_ms; ts += 25.f){
                
                    dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * i, edge.link.vel};
                    next_pose = dynamics::SimpleDynamicsModel::computeNextPose(veh_pose, edge.link.s_a, edge.link.s_v, ts);
                    
                    json point;
                    point["x"] = next_pose.pos[0];
                    point["y"] = next_pose.pos[1];
                    jedge["curve"].push_back(point);

                }
                
                for(float ts = 0; ts <= timestep_ms; ts += 25.f){
                
                    dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * i, edge.link.vel};
                    auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPose(next_pose, edge.link.s_a_2, edge.link.s_v_2, ts);
                    
                    json point;
                    point["x"] = next_pose2.pos[0];
                    point["y"] = next_pose2.pos[1];
                    jedge["curve"].push_back(point);
                }

                

                jedge["target"]["x"] = m_proxyMap[edge.target.x][edge.target.y][0][0].rel_pose.pos[0];
                jedge["target"]["y"] = m_proxyMap[edge.target.x][edge.target.y][0][0].rel_pose.pos[1];
                jedge["target"]["s"] = m_proxyMap[edge.target.x][edge.target.y][edge.target.a][edge.target.s].rel_pose.vel;
                jedge["target"]["a"] = m_proxyMap[edge.target.x][edge.target.y][edge.target.a][edge.target.s].rel_pose.h;

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
    std::ofstream o("proxy_state_graph.json");
    o << proxy_map_dump << std::endl;
    o.close();
}

void DirectedSearchProxy::loadGraphFromDisk(){
    loadGraphFromDisk("proxy_state_graph.json");
}

void DirectedSearchProxy::loadGraphFromDisk(std::string path){
    std::ifstream ifs(path);
    json jf = json::parse(ifs);
    m_proxyMapReachableSpan = jf["info"]["m_proxyMapReachableSpan"];
    m_proxyMapReachableSpan += 1;
    m_proxyMapCarOffset = jf["info"]["m_proxyMapCarOffset"];

    std::cout << "Loading map with: " << jf["map"].size() << " and expected " << m_proxyMapReachableSpan * m_proxyMapReachableSpan * map_size_angle * map_size_speed <<std::endl;
    assert(jf["map"].size() == m_proxyMapReachableSpan * m_proxyMapReachableSpan * map_size_angle * map_size_speed);
    uint32_t index = 0;
    
    for(auto node: jf["map"]){

        uint32_t x = node["pose"]["xi"];
        uint32_t y = node["pose"]["yi"];
        uint32_t a = node["pose"]["ai"];
        uint32_t s = node["pose"]["si"];
        
        m_proxyMap[x][y][a][s].rel_pose.h = node["pose"]["h"];
        m_proxyMap[x][y][a][s].rel_pose.vel = node["pose"]["v"];
        m_proxyMap[x][y][a][s].rel_pose.pos[0] = node["pose"]["x"];
        m_proxyMap[x][y][a][s].rel_pose.pos[1] = node["pose"]["y"];
    }


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


void DirectedSearchProxy::printPositions(){
    json proxy_map_dump;
    // Dump array to list
    for(uint32_t x = 0; x < m_proxyMapReachableSpan; x ++){
        for(uint32_t y = 0; y < m_proxyMapReachableSpan; y ++){
            for(uint32_t a = 0; a < map_size_angle; a ++){
                for(uint32_t s = 0; s < map_size_speed; s ++){
                    //Compute position, heading and velocity of the node
                    json node;
                    node["pose"]["xi"] = x;
                    node["pose"]["yi"] = y;
                    node["pose"]["ai"] = a;
                    node["pose"]["si"] = s;
                    node["pose"]["x"] =  m_proxyMap[x][y][a][s].rel_pose.pos[0];
                    node["pose"]["y"] =  m_proxyMap[x][y][a][s].rel_pose.pos[1];
                    node["pose"]["h"] =  m_proxyMap[x][y][a][s].rel_pose.h;
                    node["pose"]["v"] =  m_proxyMap[x][y][a][s].rel_pose.vel;
                    proxy_map_dump["map"].push_back(node);
                }
            }
        }
    }
    //dump file to disc
    std::ofstream o("proxy_state_nodes.json");
    o << proxy_map_dump << std::endl;
    o.close();
}