#include <Planner/StateGraph.hpp>

#include <cmath>
#include <iostream>
#include <thread>

StateGraph::StateGraph(){


}

void StateGraph::workerThreadProxyEdges(uint32_t index){
    float spi = dynamics::SimpleDynamicsModel::velocity_limit() / 2; //TODO CHANGE TO NOT HARD CODED VALUE HERE
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
            
            dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * threadTask.cai,-dynamics::SimpleDynamicsModel::velocity_limit() + (spi * threadTask.csi)};
            for(uint32_t a = 1; a < map_size_angle; a ++){
                for(uint32_t s = 1; s < map_size_angle; s ++){
            
                //Compute best fit settings set to get from the current to the target location
                dynamics::data::PoseByIndex target_pose_by_index = {threadTask.txi,threadTask.tyi,a,s};
                auto epose = dynamics::SimpleDynamicsModel::computeBestFitSingleStep(veh_pose, target_pose_by_index, m_proxyMap[threadTask.txi][threadTask.tyi][a][s].rel_pose, threadTask.tstep);
                
                //Discard if error greater than half of the angular resolution 
                if(epose.a_error >  api / 2){ //TODO: MAKE THIS VALUE ADJUSTABLE, CURRENTLY WEIGHT JUST SET ARBITRARILY
                    continue;
                }
                if(epose.p_error > xpc / 2){ //TODO: MAKE THIS VALUE ADJUSTABLE
                    continue;
                }

                // Found a link to the neighbor, add an edge for this one
                TraversableEdge nt_edge = {epose, target_pose_by_index};
                std::cout << "[INFO]["<< index <<"] Added edge to " << std::to_string(epose.pos[0]) <<":"<< std::to_string(epose.pos[1]) << std::endl;
                m_proxyEdgeList[a].push_back(nt_edge);  
                }
            }
        }else{
            return;
        }
        hasTask = false;
    }
}

void StateGraph::computeProxyEdges(){

    float time_step_sec = 0.25f; // 500 ms timestep
    float spi = dynamics::SimpleDynamicsModel::velocity_limit() / 2; //TODO CHANGE TO NOT HARD CODED VALUE HERE
    float reachable_distance = dynamics::SimpleDynamicsModel::velocity_limit() * (float)time_step_sec;

    //Compute size of the proxy field based on reachable distance at the given speed
    uint32_t reachable_node_count = static_cast<uint32_t>(reachable_distance / xpc);
    uint32_t reachable_node_span = (2*reachable_node_count); 
    uint32_t index_car = reachable_node_count; 

    std::cout << "[INFO] Allocated all proxy nodes." << std::endl;

    //Set the positions of each proxy node relative to the car
    for(uint32_t x = 0; x < reachable_node_span; x ++){
        for(uint32_t y = 0; y < reachable_node_span; y ++){
            for(uint32_t a = 0; a < map_size_angle; a ++){
                for(uint32_t s = 0; s < map_size_speed; s ++){
                    //Compute position, heading and velocity of the node

                    m_proxyMap[x][y][a][s].rel_pose.pos[0] = -(reachable_node_count * xpc) + (xpc*x);
                    m_proxyMap[x][y][a][s].rel_pose.pos[1] = -(reachable_node_count * ypc) + (ypc*y);
                    m_proxyMap[x][y][a][s].rel_pose.h = api * map_size_angle;
                    m_proxyMap[x][y][a][s].rel_pose.vel = -dynamics::SimpleDynamicsModel::velocity_limit() + (spi * s);
                }
            }
        }
    }

    std::cout << "[INFO] Created all proxy nodes: [#] " << reachable_node_span * reachable_node_span * map_size_angle * map_size_speed << std::endl;
    std::cout << "[INFO] Reachable distance: [cm] " << reachable_distance << std::endl;
    std::cout << "[INFO] Car position : [cm] " << m_proxyMap[index_car][index_car][0][0].rel_pose.pos[0] << ":" << m_proxyMap[index_car][index_car][0][0].rel_pose.pos[0] << std::endl;
    std::cout << "[INFO] X Extends: [cm] " << -(reachable_node_count * xpc) << " - " <<  -(reachable_node_count * xpc) + (xpc*reachable_node_span) << " stp:" << xpc << " cnt:" << reachable_node_span << std::endl;
    std::cout << "[INFO] Y Extends: [cm] " << -(reachable_node_count * ypc) << " - " <<  -(reachable_node_count * ypc) + (ypc*reachable_node_span) << " stp:" << ypc << " cnt:" << reachable_node_span << std::endl;
    std::cout << "[INFO] A Extends: [rad] " << 0 << " - " <<  api * map_size_angle << " stp:" << api << " cnt:" << map_size_angle << std::endl;
    std::cout << "[INFO] S Extends: [cm/s] " << -dynamics::SimpleDynamicsModel::velocity_limit() << " - " <<  -dynamics::SimpleDynamicsModel::velocity_limit() + (spi * map_size_speed) << " stp:" << spi 
                                      << " cnt:" << map_size_speed << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Compute for each heading and speed from our original vehicle
    for(uint32_t i = 0; i < map_size_speed; i ++){
        for(uint32_t j = 0; j < map_size_angle; j ++){
            
            //Compute pose of current node/vehicle
            dynamics::data::Pose2D veh_pose = {{0.f,0.f},api * j,-dynamics::SimpleDynamicsModel::velocity_limit() + (spi * i)};
            
            //Check for each candidate node if we can find a connection between these two, but use all system threads
            for(uint32_t x = 1; x < reachable_node_span; x ++){
                for(uint32_t y = 1; y < reachable_node_span; y ++){
                     
                    //Compute heading vector of car in this state
                    
                    dynamics::Vector2Df unit_vec(1,0);
                    
                    if(){

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
                        nTask = {x,y,j,i,time_step_sec};
                        m_proxyTaskQueue.push_back(nTask);
                    }
                }
            }   
        }
    }

    std::vector<std::thread> workers;
    for(uint32_t i = 0; i < 10; i++){
        workers.push_back(std::thread(&StateGraph::workerThreadProxyEdges, this, i));
    }
    for(uint32_t i = 0; i < 10; i++){
        workers.at(i).join();
    }
}


void StateGraph::printEdges(uint32_t index){
    std::cout << "For index " << index <<  " has edge count " << m_proxyEdgeList[index].size() << std::endl;
    std::cout << "Target Position, " << std::endl;
    for(uint32_t i = 0; i < m_proxyEdgeList[index].size(); i++){
        auto edge = m_proxyEdgeList[index].at(i);
        std::cout << edge.link.pos << "; " << edge.link.h << "; " << edge.link.s_a_val[0] << "; " << edge.link.vel << "; " << edge.link.s_v_val[0] << std::endl;
    }   
}