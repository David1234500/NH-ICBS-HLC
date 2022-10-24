#include <Planner/DirectedSearchProxy.hpp>

#include <cmath>
#include <iostream>
#include <thread>
#include <fstream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

DirectedSearchProxy::DirectedSearchProxy(){}

void DirectedSearchProxy::computeProxyEdges(){
    double base_node_distance = m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<double>(m_config_ts_min) / 1000.f);
    uint32_t map_node_count = static_cast<uint32_t>(map_size_x_cm / base_node_distance);
    double api = 2 * PI / static_cast<double>(m_config_map_size_angle);

    std::vector<double> velocities;
    for(uint32_t s = 0; s < map_size_speed; s ++){
        double velocity = m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();
        std::cout << "velocity values: " << velocity << std::endl;
        velocities.push_back(velocity);
    }

    uint32_t map_angle_offset = m_config_map_size_angle / 4; 
    std::map<uint32_t, std::map<uint32_t, std::shared_ptr<std::vector<dynamics::data::Pose2DWithMotionData>>>> reachable_set_map;

    for(uint32_t a = 0; a < map_angle_offset; a ++){
        double heading = api * static_cast<double>(a);

        for(uint32_t s = 0; s < map_size_speed; s ++){
            double velocity = m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();

            dynamics::data::Pose2D current_pose = {{0.f,0.f},heading,velocity};
            auto set = dynamics::SimpleDynamicsModel::computeReachableSet(current_pose, m_config_ts_min, m_config_ts_max, velocities);
            reachable_set_map[a][s] = set;
        }
    }
    
    double negativ_direction_shift_cm = base_node_distance * (-m_config_map_size_speed - 1);
    double positiv_direction_shift_cm = base_node_distance * (m_config_map_size_speed + 1);

    for(int32_t a = 0; a < map_angle_offset; a ++){
        double heading = api * static_cast<double>(a);

        for(int32_t s = 0; s < map_size_speed; s ++){

                for(int32_t x = -m_config_map_size_speed; x <= m_config_map_size_speed; x ++){
                    for(int32_t y = -m_config_map_size_speed; y <= m_config_map_size_speed; y ++){
                    
                    if(x == 0 && y == 0){
                        continue;
                    }

                    dynamics::data::Pose2D target_pose;
                    target_pose.pos[0] = (base_node_distance*x);
                    target_pose.pos[1] = (base_node_distance*y);
                    target_pose.h = api * static_cast<double>(a);
                    target_pose.vel = m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();

                    for(auto reachable_pose: *reachable_set_map[a][s]){

                        double position_pose_error = (reachable_pose.pos - target_pose.pos).norm(); 
                        double angle_pose_error =  std::abs(reachable_pose.h - target_pose.h);

                        if(angle_pose_error > base_node_distance * state_change_fit_quality_position){
                            continue;
                        }
                        if(position_pose_error > api * state_change_fit_quality_angle){
                            continue;
                        }

                        // Motion Primitive for upper right quadrant with positiv velocity
                        MotionPrimitiv prim;
                        prim.link = reachable_pose;
                        prim.target = {x,y,a,reachable_pose.target_vel_index};
                        motion_primitive_map[a][s].push_back(prim);

                        std::cout << std::endl;
                        std::cout << "pose " << reachable_pose.pos[0] <<":"<< reachable_pose.pos[1] << ":" << reachable_pose.h << ":" << reachable_pose.vel << std::endl;
                        std::cout << "target" << x <<":"<< y << ":" << a << ":" << s << std::endl;
                        std::cout << "angle_pose_error " << angle_pose_error << std::endl;
                        std::cout << "position_pose_error " << position_pose_error << std::endl;
                        std::cout << "s_a " << reachable_pose.s_a << ":" << reachable_pose.s_a_2 << std::endl;

                        break;

                        // Motion Primitive for upper left quadrant with positiv velocity
                        
                        // prim.link = reachable_pose;
                        // dynamics::data::Vector2Df neg_x_offset_cm = {negativ_direction_shift_cm, 0.f};
                        // prim.link.pos -= neg_x_offset_cm;
                        // prim.target = {x - m_config_map_size_speed,y,a,s};
                        // motion_primitive_map[a + map_angle_offset][s].push_back(prim);

                        // // Motion Primitive for lower left quadrant with positiv velocity
                        
                        // prim.link = reachable_pose;
                        // neg_x_offset_cm = {negativ_direction_shift_cm, positiv_direction_shift_cm};
                        // prim.link.pos -= neg_x_offset_cm;
                        // prim.target = {x - m_config_map_size_speed - 1,y + m_config_map_size_speed + 1,a,s};
                        // motion_primitive_map[a + 2*map_angle_offset][s].push_back(prim);

                        // // Motion Primitive for lower right quadrant with positiv velocity
                        
                        // prim.link = reachable_pose;
                        // neg_x_offset_cm = {0.f, positiv_direction_shift_cm};
                        // prim.link.pos -= neg_x_offset_cm;
                        // prim.target = {x,y + m_config_map_size_speed + 1,a,s};
                        // motion_primitive_map[a + 3*map_angle_offset][s].push_back(prim);

                        // Negativ Velocity

                        

                       

                    }
                }
            }
        }
    }
}


void DirectedSearchProxy::writeGraphToDisk(){
    double base_node_distance = m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<double>(m_config_ts_min) / 1000.f);
    uint32_t map_node_count = static_cast<uint32_t>(map_size_x_cm / base_node_distance);
    double api = 2 * PI / static_cast<double>(m_config_map_size_angle);

   json proxy_map_dump;

    proxy_map_dump["info"]["m_config_map_size_speed"] = m_config_map_size_speed;
    proxy_map_dump["info"]["m_config_baseVelocityFactor"] = m_config_baseVelocityFactor;

    proxy_map_dump["info"]["size_a"] = map_size_angle;
    proxy_map_dump["info"]["size_s"] = map_size_speed;

     for(int32_t x = -m_config_map_size_speed; x <= m_config_map_size_speed; x ++){
        for(int32_t y = -m_config_map_size_speed; y <= m_config_map_size_speed; y ++){
            for(int32_t a = 0; a < map_size_angle; a ++){
                for(int32_t s = 0; s < map_size_speed; s ++){

                    json node;
                    node["pose"]["x"] =  (base_node_distance*x);
                    node["pose"]["y"] =  (base_node_distance*y);
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

    for(uint32_t i = 0; i < map_size_angle; i ++){
        for(uint32_t j = 0; j < map_size_speed; j ++){
            for(auto edge: motion_primitive_map[i][j]){
                json jedge;
                jedge["source"]["a"] = i;
                jedge["source"]["s"] = j;
                
                double velocity = m_config_speedsFactor[j] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();;
                double heading = api * static_cast<double>(i);
                dynamics::data::Pose2D start_pose = {{0.f,0.f}, heading, velocity};
                uint32_t sim_vel_intp = 8;
                double timestep = static_cast<double>(edge.link.ts_ms);
                auto pose_series = dynamics::SimpleDynamicsModel::computePoseSeries(start_pose, edge.link.s_a, edge.link.s_a_2, edge.link.start_vel, edge.link.target_vel, sim_vel_intp, timestep);
                
                for(auto pose: pose_series){
                
                    json point;
                    point["x"] = pose.pos[0];
                    point["y"] = pose.pos[1];
                    jedge["curve"].push_back(point);
                }
                

                jedge["t_pose"]["x"] = edge.link.pos[0];
                jedge["t_pose"]["y"] = edge.link.pos[1];
                jedge["t_pose"]["s"] = edge.link.vel;
                jedge["t_pose"]["a"] = edge.link.h;

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
    // m_proxyMapReachableSpan = jf["info"]["m_proxyMapReachableSpan"];
    // m_proxyMapReachableSpan += 1;
    // m_proxyMapCarOffset = jf["info"]["m_proxyMapCarOffset"];

    uint32_t index = 0;
    
    for(auto edge: jf["edges"]){
        MotionPrimitiv tedge;

        tedge.link.s_a = edge["settings"]["a1"];
        tedge.link.s_a_2 = edge["settings"]["a2"];


        tedge.link.h = edge["target"]["a"];
        tedge.link.pos[1] = edge["target"]["y"];
        tedge.link.pos[0] = edge["target"]["x"];
        tedge.link.vel = edge["target"]["s"];

        tedge.target.s = edge["targeti"]["s"];
        tedge.target.a = edge["targeti"]["a"];
        tedge.target.x = edge["targeti"]["x"];
        tedge.target.y = edge["targeti"]["y"];

        // m_proxyEdgeList[edge["source"]["a"]][edge["source"]["s"]].push_back(tedge);
    }
}

