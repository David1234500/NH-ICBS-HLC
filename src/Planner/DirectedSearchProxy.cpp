#include <Planner/DirectedSearchProxy.hpp>

#include <cmath>
#include <iostream>
#include <thread>
#include <fstream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

DirectedSearchProxy::DirectedSearchProxy(){}

void DirectedSearchProxy::computeProxyEdges(){
    m_base_node_distance = m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<double>(m_config_ts_base) / 1000.f);
    uint32_t map_node_count = static_cast<uint32_t>(m_config_map_size_x_cm / m_base_node_distance);
    double api = 2 * PI / static_cast<double>(m_config_map_size_angle);

    std::vector<double> velocities;
    for(uint32_t s = 0; s < m_config_map_size_speed; s ++){
        double velocity = m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();
        std::cout << "velocity values: " << velocity << std::endl;
        velocities.push_back(velocity);
    }

    // int32_t map_angle_offset = m_config_map_size_angle / 4; 
    std::map<uint32_t, std::map<uint32_t, std::shared_ptr<std::vector<dynamics::data::Pose2DWithMotionData>>>> reachable_set_map;

    for(int32_t a = 0; a <= m_config_map_size_angle; a ++){
        double heading = api * static_cast<double>(a);

        for(int32_t s = 0; s < m_config_map_size_speed; s ++){
            double velocity = m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();

            dynamics::data::Pose2D current_pose = {{0.f,0.f},heading,velocity};
            auto set = dynamics::SimpleDynamicsModel::computeReachableSet(current_pose, m_config_ts_min, m_config_ts_max, velocities);
            reachable_set_map[a][s] = set;
        }
    }

    // for(int32_t a = 0; a <= m_config_map_size_angle; a ++){
    //     double heading = api * static_cast<double>(a);

    //     int32_t vel_to_zero_1 = std::max(0, m_config_map_size_zero_velocity_level - 1);
    //     int32_t vel_to_zero_2 = std::min(m_config_map_size_zero_velocity_level + 1, m_config_map_size_speed);
        
    //     MotionPrimitiv prim;
    //     prim.is_zero_connection = true;
    //     prim.link = {heading, vel}
    //     prim.target = {0,0,a,0};
    //     motion_primitive_map[a][vel_to_zero_1].push_back(prim);

    // }

    std::vector<std::thread> workers;

    for(int32_t a = 0; a <= m_config_map_size_angle; a ++){
        double heading = api * static_cast<double>(a);

        for(int32_t s = 0; s < m_config_map_size_speed; s ++){

                for(int32_t x = -m_config_map_extent; x <= m_config_map_extent; x ++){
                    for(int32_t y = -m_config_map_extent; y <= m_config_map_extent; y ++){
                    
                    if(x == 0 && y == 0){
                        continue;
                    }

                    if(std::abs(m_config_speedsFactor[s]) < 0.01f){
                        continue;
                    }

                    dynamics::data::Pose2D target_pose;
                    target_pose.pos[0] = (m_base_node_distance*x);
                    target_pose.pos[1] = (m_base_node_distance*y);
                    target_pose.h = api * static_cast<double>(a);
                    target_pose.vel = m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();

                    searchJob job = {m_config_map_size_angle, x,y,a,api,m_base_node_distance,target_pose, reachable_set_map};
                    job_queue.push_back(job);
                }
            }
        }
    }
    for(uint32_t i = 0; i < 12; i ++){
        workers.push_back(std::thread(&DirectedSearchProxy::dispatch, this));
    }

    for(uint32_t i = 0; i < 12; i ++){
        workers.at(i).join();
    }

}

void DirectedSearchProxy::dispatch(){
    while(true){
        searchJob job;

        job_mutex.lock();
        
        if(job_queue.empty()){
            job_mutex.unlock();
            return;
        }else{
            job = job_queue.back();
            job_queue.pop_back();
            std::cout << "Queue size: " << std::to_string(job_queue.size()) << std::endl;

            job_mutex.unlock();
        }
        
        worker(job);
    }
}

void DirectedSearchProxy::worker(searchJob job){
     for(int32_t source_a = 0; source_a <= m_config_map_size_angle; source_a ++){

        for(int32_t source_s = 0; source_s < m_config_map_size_speed; source_s ++){

            for(auto reachable_pose: *job.reachable_set_map[source_a][source_s]){

                double position_pose_error = (reachable_pose.pos - job.target_pose.pos).norm(); 
                double angle_pose_error =  std::abs(reachable_pose.h - job.target_pose.h);

                if(position_pose_error > job.base_node_distance * state_change_fit_quality_position){
                    continue;
                }

                if(angle_pose_error > job.api * state_change_fit_quality_angle){
                    continue;
                }

                MotionPrimitiv prim;
                prim.link = reachable_pose;
                prim.target = {job.x,job.y,job.a,reachable_pose.target_vel_index};
                motion_primitive_map_mutex.lock();
                motion_primitive_map[source_a][source_s].push_back(prim);
                motion_primitive_map_mutex.unlock();

                std::cout << std::endl;
                std::cout << "pose " << reachable_pose.pos[0] <<":"<< reachable_pose.pos[1] << ":" << reachable_pose.h << ":" << reachable_pose.vel << std::endl;
                std::cout << "target" << job.x <<":"<< job.y << ":" << job.a << std::endl;
                std::cout << "angle_pose_error " << angle_pose_error << std::endl;
                std::cout << "position_pose_error " << position_pose_error << std::endl;
                std::cout << "s_a " << reachable_pose.s_a << ":" << reachable_pose.s_a_2 << std::endl;
                
                break;
            }
        }
    }
}

void DirectedSearchProxy::writeGraphToDisk(){
    m_base_node_distance = m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<double>(m_config_ts_base) / 1000.f);
    uint32_t map_node_count = static_cast<uint32_t>(m_config_map_size_x_cm / m_base_node_distance);
    m_comp_map_size_x = map_node_count;
    m_comp_map_size_y = map_node_count;
    double api = 2 * PI / static_cast<double>(m_config_map_size_angle);

    json proxy_map_dump;

    proxy_map_dump["info"]["m_config_map_size_speed"] = m_config_map_size_speed;
    proxy_map_dump["info"]["m_config_map_extent"] = m_config_map_extent;
    proxy_map_dump["info"]["m_base_node_distance"] = m_base_node_distance;
    proxy_map_dump["info"]["m_config_baseVelocityFactor"] = m_config_baseVelocityFactor;
    proxy_map_dump["info"]["m_comp_map_size_x"] = m_comp_map_size_x;
    proxy_map_dump["info"]["m_comp_map_size_y"] = m_comp_map_size_y;

    for(auto speed: m_config_speedsFactor){
        proxy_map_dump["info"]["m_config_speedsFactor"].push_back(speed);
    }

    proxy_map_dump["info"]["size_a"] = m_config_map_size_angle;
    proxy_map_dump["info"]["size_s"] = m_config_map_size_speed;
    proxy_map_dump["info"]["size_y_cm"] = m_config_map_size_y_cm;
    proxy_map_dump["info"]["size_x_cm"] = m_config_map_size_x_cm;

     for(int32_t x = -m_config_map_extent; x <= m_config_map_extent; x ++){
        for(int32_t y = -m_config_map_extent; y <= m_config_map_extent; y ++){
            for(int32_t a = 0; a < m_config_map_size_angle; a ++){
                for(int32_t s = 0; s < m_config_map_size_speed; s ++){

                    json node;
                    node["pose"]["x"] =  (m_base_node_distance*x);
                    node["pose"]["y"] =  (m_base_node_distance*y);
                    node["pose"]["h"] =  api * static_cast<double>(a);
                    node["pose"]["v"] =  m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();

                    node["pose"]["xi"] = x;
                    node["pose"]["yi"] = y;
                    node["pose"]["ai"] = a;
                    node["pose"]["si"] = s;

                    proxy_map_dump["map"].push_back(node);
                }
            }
        }
    }

    for(uint32_t i = 0; i < m_config_map_size_angle; i ++){
        for(uint32_t j = 0; j < m_config_map_size_speed; j ++){
            for(auto edge: motion_primitive_map[i][j]){
                json jedge;
                jedge["source"]["a"] = i;
                jedge["source"]["s"] = j;
                
                double velocity = m_config_speedsFactor[j] * (m_config_baseVelocityFactor - 0.1 ) * dynamics::SimpleDynamicsModel::velocity_limit();;
                double heading = api * static_cast<double>(i);
                dynamics::data::Pose2D start_pose = {{0.f,0.f}, heading, velocity};
                uint32_t sim_vel_intp = 2;
                double timestep = static_cast<double>(edge.link.ts_ms);
                auto pose_series = dynamics::SimpleDynamicsModel::computePoseSeries(start_pose, edge.link.s_a, edge.link.s_a_2, edge.link.start_vel, edge.link.target_vel, sim_vel_intp, timestep, 0);
                
                for(auto pose: pose_series){
                
                    json point;
                    point["x"] = pose.pos[0];
                    point["y"] = pose.pos[1];
                    jedge["curve"].push_back(point);
                }
                

                jedge["t_pose"]["x"] = pose_series.back().pos[0];
                jedge["t_pose"]["y"] = pose_series.back().pos[1];
                jedge["t_pose"]["s"] = pose_series.back().vel;
                jedge["t_pose"]["a"] = pose_series.back().h;

                jedge["settings"]["a1"] = edge.link.s_a;
                jedge["settings"]["a2"] = edge.link.s_a_2;
                jedge["settings"]["ts"] = edge.link.ts_ms;
                jedge["settings"]["sv"] = edge.link.start_vel;
                jedge["settings"]["tv"] = edge.link.target_vel;

                jedge["t_index"]["s"] = edge.target.s;
                jedge["t_index"]["a"] = edge.target.a;
                jedge["t_index"]["x"] = edge.target.x;
                jedge["t_index"]["y"] = edge.target.y;

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
    
    m_comp_map_size_x = jf["info"]["m_comp_map_size_x"];
    m_comp_map_size_y = jf["info"]["m_comp_map_size_y"];
    
    m_config_map_size_speed = jf["info"]["m_config_map_size_speed"];
    m_config_map_extent = jf["info"]["m_config_map_extent"];
    m_config_baseVelocityFactor = jf["info"]["m_config_baseVelocityFactor"];
    m_base_node_distance = jf["info"]["m_base_node_distance"];

    m_config_map_size_angle = jf["info"]["size_a"] ;
    m_config_map_size_speed = jf["info"]["size_s"];

    m_config_map_size_y_cm = jf["info"]["size_y_cm"];
    m_config_map_size_x_cm = jf["info"]["size_y_cm"];

    m_config_speedsFactor.clear();
    
    for(auto speed: jf["info"]["m_config_speedsFactor"]){
        m_config_speedsFactor.push_back(speed);
    }

    uint32_t index = 0;
    
    for(auto edge: jf["edges"]){
        MotionPrimitiv tedge;

        tedge.link.s_a = edge["settings"]["a1"];
        tedge.link.s_a_2 = edge["settings"]["a2"];
        tedge.link.ts_ms = edge["settings"]["ts"];
        tedge.link.start_vel = edge["settings"]["sv"];
        tedge.link.target_vel = edge["settings"]["tv"];

        tedge.link.h = edge["t_pose"]["a"];
        tedge.link.pos[1] = edge["t_pose"]["y"];
        tedge.link.pos[0] = edge["t_pose"]["x"];
        tedge.link.vel = edge["t_pose"]["s"];

        tedge.target.s = edge["t_index"]["s"];
        tedge.target.a = edge["t_index"]["a"];
        tedge.target.x = edge["t_index"]["x"];
        tedge.target.y = edge["t_index"]["y"];

        motion_primitive_map[edge["source"]["a"]][edge["source"]["s"]].push_back(tedge);
    }
}

