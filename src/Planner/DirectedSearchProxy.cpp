#include <Planner/DirectedSearchProxy.hpp>

#include <cmath>
#include <iostream>
#include <thread>
#include <fstream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

DirectedSearchProxy::DirectedSearchProxy(){}

void DirectedSearchProxy::computeProxyEdges(){
    float base_node_distance = m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<float>(timestep_ms) / 1000.f);
    uint32_t map_node_count = static_cast<uint32_t>(map_size_x_cm / base_node_distance);
    float api = 2 * PI / static_cast<float>(m_config_map_size_angle);

    LatticeNode lattice[m_config_map_size_speed][m_config_map_size_speed][map_size_angle][map_size_speed];

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
    std::map<uint32_t, std::map<uint32_t, std::shared_ptr<std::vector<dynamics::data::Pose2DWithMotionData>>>> reachable_set_map;

    for(uint32_t a = 0; a < map_angle_offset; a ++){
        float heading = api * static_cast<float>(a);

        for(uint32_t s = 0; s < map_size_speed; s ++){
            float velocity = m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();

            dynamics::data::Pose2D current_pose = {{0.f,0.f},heading,velocity};
            auto set = dynamics::SimpleDynamicsModel::computeReachableSet(current_pose, m_config_ts_min, m_config_ts_max);
            reachable_set_map[a][s] = set;
        }
    }
    
    float negativ_direction_shift_cm = base_node_distance * (-m_config_map_size_speed - 1);
    float positiv_direction_shift_cm = base_node_distance * (m_config_map_size_speed +  1);
    for(int32_t a = 0; a < map_angle_offset; a ++){
        float heading = api * static_cast<float>(a);

        for(int32_t s = 0; s < map_size_speed; s ++){
            float velocity = m_config_speedsFactor[s] * m_config_baseVelocityFactor * dynamics::SimpleDynamicsModel::velocity_limit();

                for(int32_t x = -m_config_map_size_speed; x < m_config_map_size_speed; x ++){
                    for(int32_t y = -m_config_map_size_speed; y < m_config_map_size_speed; y ++){

                    dynamics::data::Pose2D target_pose =  lattice[x][y][a][s].rel_pose;
                    for(auto reachable_pose: *reachable_set_map[a][s]){
                        float position_pose_error = (reachable_pose.pos - target_pose.pos).norm(); 
                        float angle_pose_error =  std::abs(reachable_pose.h - target_pose.h);

                        if(angle_pose_error > state_change_fit_quality_angle){
                            continue;
                        }
                        if(position_pose_error > state_change_fit_quality_position){
                            continue;
                        }

                        // Motion Primitive for upper right quadrant with positiv velocity
                        MotionPrimitiv prim;
                        prim.link = reachable_pose;
                        prim.target = {x,y,a,s};
                        motion_primitive_map[a][s].push_back(prim);

                        // Motion Primitive for upper left quadrant with positiv velocity
                        MotionPrimitiv prim;
                        prim.link = reachable_pose;
                        dynamics::data::Vector2Df neg_x_offset_cm = {negativ_direction_shift_cm, 0.f};
                        prim.link.pos -= neg_x_offset_cm;
                        prim.target = {x - m_config_map_size_speed,y,a,s};
                        motion_primitive_map[a + map_angle_offset][s].push_back(prim);

                        // Motion Primitive for lower left quadrant with positiv velocity
                        MotionPrimitiv prim;
                        prim.link = reachable_pose;
                        dynamics::data::Vector2Df neg_x_offset_cm = {negativ_direction_shift_cm, positiv_direction_shift_cm};
                        prim.link.pos -= neg_x_offset_cm;
                        prim.target = {x - m_config_map_size_speed - 1,y + m_config_map_size_speed + 1,a,s};
                        motion_primitive_map[a + 2*map_angle_offset][s].push_back(prim);

                        // Motion Primitive for lower right quadrant with positiv velocity
                        MotionPrimitiv prim;
                        prim.link = reachable_pose;
                        dynamics::data::Vector2Df neg_x_offset_cm = {0.f, positiv_direction_shift_cm};
                        prim.link.pos -= neg_x_offset_cm;
                        prim.target = {x,y + m_config_map_size_speed + 1,a,s};
                        motion_primitive_map[a + 3*map_angle_offset][s].push_back(prim);


                        //TODO ADD negativ velocities here

                        std::cout << "YEAH FOUND NEW EDGE" << std::endl;
                    }
                }
            }
        }
    }
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