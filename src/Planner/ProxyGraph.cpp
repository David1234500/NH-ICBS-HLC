#include <Planner/ProxyGraph.hpp>

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
            std::cout << "[INFO]["<< index <<"] Working now on task: " << threadTask.txi <<":"<< threadTask.tyi << ":"  << threadTask.cai <<":"<< threadTask.csi << std::endl;
            hasTask = true;
        }else{
            hasTask = false;
        }
        m_proxyTaskMutex.unlock();

        if(hasTask){
            
            dynamics::data::Pose2D veh_pose = {{0.f,0.f}, m_proxyMap[0][0][threadTask.cai][threadTask.csi].rel_pose.h,  m_proxyMap[0][0][threadTask.cai][threadTask.csi].rel_pose.vel};
            for(int32_t a = 0; a < map_size_angle; a ++){
                for(int32_t s = 0; s < map_size_speed; s ++){
            
                //Compute best fit settings set to get from the current to the target location
                dynamics::data::PoseByIndex target_pose_by_index = {threadTask.txi,threadTask.tyi, a, s};
                auto epose = dynamics::SimpleDynamicsModel::computeBestFit(veh_pose, target_pose_by_index, m_proxyMap[threadTask.txi][threadTask.tyi][a][s].rel_pose, threadTask.tstep);
                
                //Discard if error greater than half of the angular resolution 
                if(epose.a_error >  api / 4){ //TODO: MAKE THIS VALUE ADJUSTABLE, CURRENTLY WEIGHT JUST SET ARBITRARILY
                    continue;
                }
                if(epose.p_error > xpc / 20){ //TODO: MAKE THIS VALUE ADJUSTABLE
                    continue;
                }

                // Found a link to the neighbor, add an edge for this one
                TraversableEdge nt_edge = {epose, target_pose_by_index};
                
                std::cout << "[INFO]["<< index <<"] Added edge to " << std::to_string(epose.pos[0]) <<":"<< std::to_string(epose.pos[1]) 
                << " >s" << epose.s_a <<"_"<< epose.s_v << "e" << epose.p_error <<  "p" << a <<"_"<< s << std::endl;
                m_proxyEdgeList[threadTask.cai].push_back(nt_edge);  
                }
            }
        }else{
            return;
        }
        hasTask = false;
    }
}

void ProxyGraph::computeProxyEdges(){

    float time_set_ms = 250.f; // 500 ms timestep
    float spi = dynamics::SimpleDynamicsModel::velocity_limit() / 2; //TODO CHANGE TO NOT HARD CODED VALUE HERE
    float reachable_distance = dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<float>(time_set_ms) / 1000.f);

    //Compute size of the proxy field based on reachable distance at the given speed
    uint32_t reachable_node_count = static_cast<uint32_t>(reachable_distance / xpc);
    uint32_t reachable_node_span = (2*reachable_node_count) + 1; 
    m_proxyMapReachableSpan = reachable_node_span;
    m_proxyMapCarOffset = reachable_node_count; 

    std::cout << "[INFO] Allocated all proxy nodes." << std::endl;

    //Set the positions of each proxy node relative to the car
    for(uint32_t x = 0; x < reachable_node_span; x ++){
        for(uint32_t y = 0; y < reachable_node_span; y ++){
            for(uint32_t a = 0; a < map_size_angle; a ++){
                for(uint32_t s = 0; s < map_size_speed; s ++){
                    //Compute position, heading and velocity of the node

                    m_proxyMap[x][y][a][s].rel_pose.pos[0] = -(reachable_node_count * xpc) + (xpc*x);
                    m_proxyMap[x][y][a][s].rel_pose.pos[1] = -(reachable_node_count * ypc) + (ypc*y);
                    m_proxyMap[x][y][a][s].rel_pose.h = api * static_cast<float>(a);
                    m_proxyMap[x][y][a][s].rel_pose.vel = m_speedsFactor[s] * dynamics::SimpleDynamicsModel::velocity_limit();

                }
            }
        }
    }

    std::cout << "[INFO] Created all proxy nodes: [#] " << reachable_node_span * reachable_node_span * map_size_angle * map_size_speed << std::endl;
    std::cout << "[INFO] Reachable distance: [cm] " << reachable_distance << std::endl;
    std::cout << "[INFO] Car position : [cm] " << m_proxyMap[m_proxyMapCarOffset][m_proxyMapCarOffset][0][0].rel_pose.pos[0] << ":" << m_proxyMap[m_proxyMapCarOffset][m_proxyMapCarOffset][0][0].rel_pose.pos[0] << std::endl;
    std::cout << "[INFO] X Extends: [cm] " << -(reachable_node_count * xpc) << " - " <<  -(reachable_node_count * xpc) + (xpc*reachable_node_span) << " stp:" << xpc << " cnt:" << reachable_node_span << std::endl;
    std::cout << "[INFO] Y Extends: [cm] " << -(reachable_node_count * ypc) << " - " <<  -(reachable_node_count * ypc) + (ypc*reachable_node_span) << " stp:" << ypc << " cnt:" << reachable_node_span << std::endl;
    std::cout << "[INFO] A Extends: [rad] " << 0 << " - " <<  api * map_size_angle << " stp:" << api << " cnt:" << map_size_angle << std::endl;
    std::cout << "[INFO] S Extends: [cm/s] " <<  m_speedsFactor[0] * dynamics::SimpleDynamicsModel::velocity_limit() << " - " <<   m_speedsFactor[4] * dynamics::SimpleDynamicsModel::velocity_limit() << " stp:" << spi 
                                      << " cnt:" << map_size_speed << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    

    // Compute for each heading and speed from our original vehicle
    for(int32_t i = 0; i < map_size_speed; i ++){
        for(int32_t j = 0; j < map_size_angle; j ++){
            
            //Compute pose of current node/vehicle
            dynamics::data::Pose2D veh_pose = {{0.f,0.f}, m_proxyMap[0][0][j][i].rel_pose.h, m_proxyMap[0][0][j][i].rel_pose.vel};
            
            //Check for each candidate node if we can find a connection between these two, but use all system threads
            for(int32_t x = 0; x < reachable_node_span; x ++){
                for(int32_t y = 0; y < reachable_node_span; y ++){
                     
                    if(x == m_proxyMapCarOffset && y == m_proxyMapCarOffset){
                        continue;
                    }

                    //Compute heading vector of car in this state
                    
                    dynamics::Vector2Df unit_vec(1,0);
                    
                    if(veh_pose.vel < 0.f){
                        unit_vec[0] = -1.f;
                    }


                    Eigen::Rotation2Df rot_to_heading_mx(map_size_angle * api);
                    auto heading_vec = rot_to_heading_mx * unit_vec;
                    auto can_node_pos = m_proxyMap[x][y][0][0].rel_pose.pos;
                    
                    // Compute angle difference and threshold it
                    auto t1_dot =  heading_vec.dot(can_node_pos);
                    auto t2_n = heading_vec.norm() * can_node_pos.norm();
                    auto t3 = asin(t1_dot / t2_n);

                    //Check that angle between these is smaller than threshold to reduce compute time
                    if(fabs(t3) < 2 * dynamics::SimpleDynamicsModel::angle_limit()){
                        std::cout << "[INFO] Adding Task for x " << x << ":y " << y <<  "a " << j << ":s " << i << " asin(B)" << t3 << std::endl;
                        std::lock_guard<std::mutex> lock(m_proxyTaskMutex);    
                        
                        ProxyTask nTask;    
                        nTask = {x,y,j,i,time_set_ms};
                        m_proxyTaskQueue.push_back(nTask);
                    }
                }
            }   
        }
    }

    std::vector<std::thread> workers;
    for(uint32_t i = 0; i < 10; i++){
        workers.push_back(std::thread(&ProxyGraph::workerThreadProxyEdges, this, i));
    }
    for(uint32_t i = 0; i < 10; i++){
        workers.at(i).join();
    }
}


void ProxyGraph::writeGraphToDisk(){
   json proxy_map_dump;
    // Dump array to list
    for(uint32_t x = 0; x < m_proxyMapReachableSpan; x ++){
        for(uint32_t y = 0; y < m_proxyMapReachableSpan; y ++){
            for(uint32_t a = 0; a < map_size_angle; a ++){
                for(uint32_t s = 0; s < map_size_speed; s ++){
                    //Compute position, heading and velocity of the node
                    json node;
                    node["pose"]["x"] =  m_proxyMap[x][y][a][s].rel_pose.pos[0];
                    node["pose"]["y"] =  m_proxyMap[x][y][a][s].rel_pose.pos[1];
                    node["pose"]["h"] =  m_proxyMap[x][y][a][s].rel_pose.h;
                    node["pose"]["v"] =  m_proxyMap[x][y][a][s].rel_pose.vel;
                    proxy_map_dump["map"].push_back(node);
                }
            }
        }
    }

    for(uint32_t i = 0; i < map_size_angle; i ++){
        for(auto edge: m_proxyEdgeList[i]){
            json jedge;
            jedge["source"]["a"] = i;
            
            
            if(edge.link.s_a_2 != 0.f){

                dynamics::data::Pose2D next_pose;
                for(float ts = 0; ts < 125.f; ts += 25.f){
                
                    dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * i, edge.link.vel};
                    next_pose = dynamics::SimpleDynamicsModel::computeNextPose(veh_pose, edge.link.s_a, edge.link.s_v, ts);
                    
                    json point;
                    point["x"] = next_pose.pos[0];
                    point["y"] = next_pose.pos[1];
                    jedge["curve"].push_back(point);

                }
                
                for(float ts = 0; ts < 125.f; ts += 25.f){
                
                    dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * i, edge.link.vel};
                    auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPose(next_pose, edge.link.s_a_2, edge.link.s_v_2, ts);
                    
                    json point;
                    point["x"] = next_pose2.pos[0];
                    point["y"] = next_pose2.pos[1];
                    jedge["curve"].push_back(point);
                }

            }else{
                for(float ts = 0; ts < 250.f; ts += 25.f){
                
                    dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * i, edge.link.vel};
                    auto next_pose = dynamics::SimpleDynamicsModel::computeNextPose(veh_pose, edge.link.s_a, edge.link.s_v, ts);
                    
                    json point;
                    point["x"] = next_pose.pos[0];
                    point["y"] = next_pose.pos[1];
                    jedge["curve"].push_back(point);

                }
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

    //dump file to disc
    std::ofstream o("proxy_state_graph.json");
    o << proxy_map_dump << std::endl;
    o.close();
}

void ProxyGraph::printPositions(){
    json proxy_map_dump;
    // Dump array to list
    for(uint32_t x = 0; x < m_proxyMapReachableSpan; x ++){
        for(uint32_t y = 0; y < m_proxyMapReachableSpan; y ++){
            for(uint32_t a = 0; a < map_size_angle; a ++){
                for(uint32_t s = 0; s < map_size_speed; s ++){
                    //Compute position, heading and velocity of the node
                    json node;
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