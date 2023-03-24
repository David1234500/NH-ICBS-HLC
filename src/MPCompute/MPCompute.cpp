#include <MPCompute/MPCompute.hpp>
#include <nlohmann/json.hpp>


using json = nlohmann::json;



struct Vector2iComparator {
    bool operator()(const Eigen::Vector2i& a, const Eigen::Vector2i& b) const {
        if (a.x() == b.x()) {
            return a.y() < b.y();
        }
        return a.x() < b.x();
    }
};

MPCompute::MPCompute(){

}

MPCompute::~MPCompute(){

}

bool MPCompute::isInsideParallelogram(const Eigen::Vector2i& point, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
    Eigen::Matrix2d A;
    A << v1.x(), v2.x(),
         v1.y(), v2.y();
    
    Eigen::Vector2d b = point.cast<double>();
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

    return (x[0] >= 0) && (x[1] >= 0) && (x[0] <= 1) && (x[1] <= 1);
}

std::vector<Eigen::Vector2i> MPCompute::integerPointsInParallelogram(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
    std::vector<Eigen::Vector2i> points;
    Eigen::Vector2i min_corner = v1.cwiseMin(v2).cast<int>();
    Eigen::Vector2i max_corner = v1.cwiseMax(v2).cast<int>();

    for (int i = min_corner.x(); i <= max_corner.x(); ++i) {
        for (int j = min_corner.y(); j <= max_corner.y(); ++j) {
            Eigen::Vector2i point(i, j);
            if (isInsideParallelogram(point, v1, v2)) {
                points.push_back(point);
            }
        }
    }

    return points;
}


void MPCompute::computeMPEdges(){
    float reachable_distance = dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<float>(timestep_ms) / 1000.f);

    //Compute size of the mp field based on reachable distance at the given speed
    int32_t reachable_node_count = static_cast<uint32_t>(reachable_distance / xpc);
    uint32_t reachable_node_span = (2 * reachable_node_count); 
    m_mpMapReachableSpan = reachable_node_span;
    m_mpMapCarOffset = reachable_node_count; 

    // Compute for each heading and speed from our original vehicle
    for(int32_t sv = zero_velocity_level; sv < map_size_speed; sv ++){
        for(int32_t sh = 0; sh < map_size_angle / 4; sh ++){
            for(int32_t ts =  std::max(zero_velocity_level, sv - 1); ts < std::min(map_size_speed, sv + 2); ts ++){

                //Compute pose of current node/vehicle
                auto lim_vecs = dynamics::SimpleDynamicsModel::vector_limits(sh * api, 1.0f);
                dynamics::data::Vector2Df vbase = {1.0f, 0.f};
                auto vvec = Eigen::Rotation2Df(sh*api) * vbase;
                
                int dotProduct = lim_vecs[0].dot(lim_vecs[1]);
                double a_norm = lim_vecs[0].norm();
                double b_norm = lim_vecs[1].norm();
                double cosTheta = static_cast<double>(dotProduct) / (a_norm * b_norm);
                double angleRadians = std::acos(cosTheta);

                float reach_dist_vel_level = dynamics::SimpleDynamicsModel::velocity_limit() * (0.5f * (m_speedsFactor[sv] + m_speedsFactor[ts])) * (static_cast<float>(timestep_ms) / 1000.f);
                double base_angle_radians = (M_PI - angleRadians) / 2.0;
                double side_length = reach_dist_vel_level / std::sin(base_angle_radians);
                auto p_integer_hull = integerPointsInParallelogram(lim_vecs[0].cast<double>() * (side_length / a_norm),lim_vecs[1].cast<double>() * (side_length / b_norm));
                
                std::set<Eigen::Vector2i, Vector2iComparator> pi_integer_hull;
                for(auto p: p_integer_hull){
                    Eigen::Vector2i pbi = {p[0] / xpc, p[1] / ypc};
                    pi_integer_hull.insert(pbi);
                }

                rlog("workerMPEdges", LOG_INFO," Creating task for configuration: [" + std::to_string(angleRadians) + ", " + std::to_string(p_integer_hull.size()) + ", "+ std::to_string(pi_integer_hull.size()) + ", " + std::to_string(sv) +", "+ std::to_string(sh) + ", "+ std::to_string(ts) + ", " + std::to_string(reachable_node_count) + "]");
                rlog("workerMPEdges", LOG_INFO," L1 Vector: [" + std::to_string(lim_vecs[0][0]) + ", " + std::to_string(lim_vecs[0][1]) +"]");
                rlog("workerMPEdges", LOG_INFO," L2 Vector: [" + std::to_string(lim_vecs[1][0]) + ", " + std::to_string(lim_vecs[1][1]) +"]");
                rlog("workerMPEdges", LOG_INFO," Veh Vector: [" + std::to_string(vvec[0]) + ", " + std::to_string(vvec[1]) +"]");

                for(auto ip: pi_integer_hull){
                    int32_t x = ip[0];
                    int32_t y = ip[1];

                    if(x == 0 && y == 0){
                        continue;
                    }

                    if(sv == zero_velocity_level && ts == zero_velocity_level){
                        continue;
                    }

                    MPTask nTask;    
                    nTask = {x,y,ts,sh,sv,timestep_ms};

                    m_mpTaskQueue.push_back(nTask);
                }
            } 
        }
    }
    rlog("workerMPEdges", LOG_INFO," Done, Queued " + std::to_string(m_mpTaskQueue.size()) + " tasks");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::vector<std::thread> workers;
    for(uint32_t i = 0; i < worker_counter; i++){
        workers.push_back(std::thread(&MPCompute::workerThreadMPEdges, this, i));
    }
    for(uint32_t i = 0; i < worker_counter; i++){
        workers.at(i).join();
    }
}


void MPCompute::writeGraphToDisk(std::string name ){
   json mp_map_dump;

    mp_map_dump["info"]["m_mpMapReachableSpan"] = m_mpMapReachableSpan;
    mp_map_dump["info"]["m_mpMapCarOffset"] = m_mpMapCarOffset;
    mp_map_dump["info"]["size_a"] = map_size_angle;
    mp_map_dump["info"]["size_s"] = map_size_speed;

    // Dump array to list
    for(int32_t x = -static_cast<int32_t>(m_mpMapCarOffset); x < static_cast<int32_t>(m_mpMapCarOffset); x ++){
        for(int32_t y = -static_cast<int32_t>(m_mpMapCarOffset); y < static_cast<int32_t>(m_mpMapCarOffset); y ++){
            for(int32_t a = 0; a < map_size_angle; a ++){
                for(int32_t s = 0; s < map_size_speed; s ++){
                    //Compute position, heading and velocity of the node
                    json node;
                    node["pose"]["x"] =  (xpc*x);
                    node["pose"]["y"] =  (ypc*y);
                    node["pose"]["h"] =  api * static_cast<float>(a);
                    node["pose"]["v"] =  m_speedsFactor[s] * dynamics::SimpleDynamicsModel::velocity_limit();

                    node["pose"]["xi"] = x;
                    node["pose"]["yi"] = y;
                    node["pose"]["ai"] = a;
                    node["pose"]["si"] = s;

                    mp_map_dump["map"].push_back(node);
                }
            }
        }
    }

    for(uint32_t i = 0; i < map_size_angle; i ++){
        for(uint32_t j = 0; j < map_size_speed; j ++){
            for(auto edge: m_mpmap[i][j]){
                json jedge;
                jedge["source"]["a"] = i;
                jedge["source"]["s"] = j;
            
                dynamics::data::Pose2D next_pose;
                dynamics::data::Pose2D veh_pose = {{0.f,0.f}, api * i, edge.link.vel};
                for(float ts = 0; ts <= timestep_ms; ts += 50.f){
                    next_pose = dynamics::SimpleDynamicsModel::computeNextPose(veh_pose, edge.link.s_a, edge.link.s_v, ts);
                    json point;
                    point["x"] = next_pose.pos[0];
                    point["y"] = next_pose.pos[1];
                    jedge["curve"].push_back(point);

                }
                
                // veh_pose = {{0.f,0.f}, api * i, edge.link.vel};
                for(float ts = 0; ts <= timestep_ms; ts += 50.f){
                    auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPose(next_pose, edge.link.s_a_2, edge.link.s_v_2, ts);
                    json point;
                    point["x"] = next_pose2.pos[0];
                    point["y"] = next_pose2.pos[1];
                    jedge["curve"].push_back(point);
                }

                jedge["target"]["x"] = (xpc * edge.target.x);
                jedge["target"]["y"] = (ypc * edge.target.y);
                jedge["target"]["s"] = m_speedsFactor[edge.target.s] * dynamics::SimpleDynamicsModel::velocity_limit();
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

                mp_map_dump["edges"].push_back(jedge);
            }
        }
    }

    //dump file to disc
    std::ofstream o(name);
    o << mp_map_dump << std::endl;
    o.close();
}

void MPCompute::loadGraphFromDisk(){
    loadGraphFromDisk("mp_state_graph.json");
}

void MPCompute::loadGraphFromDisk(std::string path){
    std::ifstream ifs(path);
    json jf = json::parse(ifs);
    m_mpMapReachableSpan = jf["info"]["m_mpMapReachableSpan"];
    m_mpMapReachableSpan += 1;
    m_mpMapCarOffset = jf["info"]["m_mpMapCarOffset"];

    std::cout << "Loading map with: " << jf["map"].size() << " and expected " << m_mpMapReachableSpan * m_mpMapReachableSpan * map_size_angle * map_size_speed <<std::endl;
    assert(jf["map"].size() == m_mpMapReachableSpan * m_mpMapReachableSpan * map_size_angle * map_size_speed);
    uint32_t index = 0;

    for(auto edge: jf["edges"]){
        MotionPrimitive tedge;

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

        m_mpmap[edge["source"]["a"]][edge["source"]["s"]].push_back(tedge);
    }
}
