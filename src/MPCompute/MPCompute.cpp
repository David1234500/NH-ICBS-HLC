#include <MPCompute/MPCompute.hpp>
#include <nlohmann/json.hpp>
#include <vector>
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

void MPCompute::addSymetricMPs(double st1, double st2, double vel1, double vel2, dynamics::data::PoseByIndex tp_bi, int32_t h_sw_beg, int32_t tht_cai, int32_t tht_csi, int32_t tht_tsi){
    int32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    float api = Config::getInstance().get<float>({"disc","hstep"});

    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t x_steps = Config::getInstance().getXstep();
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});
    

    dynamics::data::Pose2D sp = {{0,0},tht_cai * api,0};
    auto p_1 = dynamics::SimpleDynamicsModel::computeNextPose(sp, st1, vel1, timestep_ms);
    auto p_r = dynamics::SimpleDynamicsModel::computeNextPose(p_1, st2, vel2, timestep_ms);

    // Q1 MP
    dynamics::data::Pose2DWithError epose;
    epose = p_r;
    epose = tp_bi;

    epose.s_a = st1;
    epose.s_a_2 = st2;
    epose.s_v = vel1;
    epose.s_v_2 = vel2;
    epose.bi_pose = tp_bi;
    MotionPrimitive mp_q1 = {epose, tp_bi};

    // Q2 MP
    double h_q2 = p_r.h + 0.5 * PI;
    Eigen::Rotation2Df rot_q2(0.5 * PI);
    dynamics::data::Position2Df pd_q2 = rot_q2 * p_r.pos;
    uint32_t hi_q2 = (h_sw_beg + (map_size_angle/4)) % map_size_angle;
    
    dynamics::data::Pose2D p_q2 = {pd_q2, h_q2, p_r.vel};
    dynamics::data::PoseByIndex pi_q2 = {round(pd_q2[0] / dpc),round(pd_q2[1] / dpc), hi_q2,  tht_tsi};

    dynamics::data::Pose2DWithError epose_q2;
    epose_q2 = p_q2;
    epose_q2.s_a = st1;
    epose_q2.s_a_2 = st2;
    epose_q2.s_v = vel1;
    epose_q2.s_v_2 = vel2;
    epose_q2.bi_pose = pi_q2;
    
    MotionPrimitive mp_q2 = {epose_q2, pi_q2};
    
    // Q3 MP
    double h_q3 = p_r.h + PI;
    Eigen::Rotation2Df rot_q3(PI);
    dynamics::data::Position2Df pd_q3 = rot_q3 * p_r.pos;
    uint32_t hi_q3 = (h_sw_beg + (map_size_angle/2)) % map_size_angle;
    
    dynamics::data::Pose2D p_q3 = {pd_q3, h_q3, p_r.vel};
    dynamics::data::PoseByIndex pi_q3 = {round(pd_q3[0] / dpc),round(pd_q3[1] / dpc), hi_q3,  tht_tsi};

    dynamics::data::Pose2DWithError epose_q3;
    epose_q3 = p_q3;
    epose_q3.s_a = st1;
    epose_q3.s_a_2 = st2;
    epose_q3.s_v = vel1;
    epose_q3.s_v_2 = vel2;
    epose_q3.bi_pose = pi_q3;
    MotionPrimitive mp_q3 = {epose_q3, pi_q3};

    // Q4 MP
    double h_q4 = p_r.h + 1.5f * PI;
    Eigen::Rotation2Df rot_q4(1.5f * PI);
    dynamics::data::Position2Df pd_q4 = rot_q4 * p_r.pos;
    uint32_t hi_q4 = (h_sw_beg + 3 * (map_size_angle/4)) % map_size_angle;
    
    dynamics::data::Pose2D p_q4 = {pd_q4, h_q4, p_r.vel};
    dynamics::data::PoseByIndex pi_q4 = {round(pd_q4[0] / dpc),round(pd_q4[1] / dpc), hi_q4,  tht_tsi};

    dynamics::data::Pose2DWithError epose_q4;
    epose_q4 = p_q4;
    epose_q4.s_a = st1;
    epose_q4.s_a_2 = st2;
    epose_q4.s_v = vel1;
    epose_q4.s_v_2 = vel2;
    epose_q4.bi_pose = pi_q4;
    MotionPrimitive mp_q4 = {epose_q4, pi_q4};

    m_mpTaskMutex.lock();
    m_mpmap[tht_cai                       ][tht_csi].push_back(mp_q1);
    m_mpmap[tht_cai +   (map_size_angle/4)][tht_csi].push_back(mp_q2);
    m_mpmap[tht_cai +   (map_size_angle/2)][tht_csi].push_back(mp_q3);
    m_mpmap[tht_cai + 3*(map_size_angle/4)][tht_csi].push_back(mp_q4);  
    m_mpTaskMutex.unlock();
}

bool MPCompute::isInsideParallelogram(const Eigen::Vector2i& point, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
    Eigen::Matrix2d A;
    A << v1.x(), v2.x(),
         v1.y(), v2.y();
    
    Eigen::Vector2d b = point.cast<double>();
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

    return (x[0] >= 0) && (x[1] >= 0) && (x[0] <= 1) && (x[1] <= 1);
}

std::vector<Eigen::Vector2i> MPCompute::integerPointsInParallelogram(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, int32_t xstep) {
    std::vector<Eigen::Vector2i> points;
    int32_t dpc = Config::getInstance().get<int32_t>({"disc","dstep"});
    for (int i = -dpc * xstep; i <= dpc * xstep; ++i) {
        for (int j =  -dpc * xstep; j <= dpc * xstep; ++j) {
            Eigen::Vector2i point(i, j);
            if (isInsideParallelogram(point, v1, v2)) {
                points.push_back(point);
            }
        }
    }

    return points;
}

 std::pair<std::vector<Eigen::Vector2i>,uint32_t> MPCompute::computeConicIntegerHull(float heading, float vel_modifier){
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"}); 
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    int32_t xstep = Config::getInstance().getXstep();
    float hstep = Config::getInstance().get<float>({"disc","hstep"});

    auto lim_vecs = dynamics::SimpleDynamicsModel::vector_limits(heading, vel_modifier * vlevels[2] * dynamics::SimpleDynamicsModel::velocity_limit());
    dynamics::data::Vector2Df vbase = {1.0f, 0.f};
    auto vvec = Eigen::Rotation2Df(heading) * vbase;

    int dotProduct = lim_vecs[0].dot(lim_vecs[1]);
    double a_norm = lim_vecs[0].norm();
    double b_norm = lim_vecs[1].norm();
    double cosTheta = static_cast<double>(dotProduct) / (a_norm * b_norm);
    double angleRadians = std::acos(cosTheta);

    float reach_dist_vel_level = dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<float>(timestep_ms) / 1000.f);
    double base_angle_radians = (M_PI - angleRadians) / 2.0;
    double side_length = reach_dist_vel_level / std::sin(base_angle_radians);
    auto p_integer_hull = integerPointsInParallelogram((lim_vecs[0].cast<double>()/a_norm) * side_length,(lim_vecs[1].cast<double>()/b_norm)* side_length,xstep);
                
    rlog("computeConicIntegerHull", LOG_INFO," L1 Vector: [" + std::to_string(lim_vecs[0][0]) + ", " + std::to_string(lim_vecs[0][1]) +"]");
    rlog("computeConicIntegerHull", LOG_INFO," L2 Vector: [" + std::to_string(lim_vecs[1][0]) + ", " + std::to_string(lim_vecs[1][1]) +"]");
    rlog("computeConicIntegerHull", LOG_INFO," Veh Vector: [" + std::to_string(vvec[0]) + ", " + std::to_string(vvec[1]) +"]");
    

    return std::pair<std::vector<Eigen::Vector2i>,uint32_t>(p_integer_hull, round(angleRadians / hstep));
}

void MPCompute::writeIntegerHullToDisc( std::vector<Eigen::Vector2i> hull, int32_t sv, int32_t ts, float head, std::string name){
    json hull_json;

    hull_json["source_velocity"] = sv;
    hull_json["target_velocity"] = ts;
    hull_json["heading"] = head;

    if(hull.empty()){
        return;
    }

    for(auto vec: hull){
        json point;
        point["x"] = vec[0];
        point["y"] = vec[1];
        hull_json["points"].push_back(point);
    }

    std::ofstream o(name);
    o << hull_json << std::endl;
    o.close();
}

void MPCompute::computeMPEdges(){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

    float reachable_distance = dynamics::SimpleDynamicsModel::velocity_limit() * (static_cast<float>(timestep_ms) / 1000.f);
    //Compute size of the mp field based on reachable distance at the given speed
    m_mpMapReachableNodeCount = static_cast<uint32_t>(reachable_distance / dpc);

    // Compute for each heading and speed from our original vehicle
    for(int32_t sv = 0; sv < map_size_speed; sv ++){ 
        for(int32_t sh = 0; sh < map_size_angle / 4; sh ++){
            for(int32_t ts = std::max(0, sv - 1); ts < std::min(map_size_speed, sv + 2); ts ++){

                if(sv == zero_velocity_level && ts == zero_velocity_level){
                    continue;
                }

                float factor = 0.f;
                if(ts <= zero_velocity_level && sv <= zero_velocity_level){
                    factor = -1.f;
                }
                if(ts >=zero_velocity_level && sv >= zero_velocity_level){
                    factor = 1.f;
                }

                auto hull_angle_pair = computeConicIntegerHull(sh * api, factor);

                std::set<Eigen::Vector2i, Vector2iComparator> pi_integer_hull;
                std::vector<Eigen::Vector2i> pi_hull;
                for(auto p: hull_angle_pair.first){
                    Eigen::Vector2i pbi = {p[0] / dpc, p[1] / dpc};
                    pi_integer_hull.insert(pbi);
                    pi_hull.push_back(pbi);
                }
                writeIntegerHullToDisc(pi_hull, sv, ts, sh * api, "hull_"+std::to_string(sh)+ "_" +std::to_string(sv)+ "_" +std::to_string(ts)+".json");
                

                rlog("computeConicIntegerHull", LOG_INFO," Creating task for configuration: ["+ std::to_string(hull_angle_pair.first.size()) + ", "+ std::to_string(pi_integer_hull.size()) + ", " + std::to_string(sv) +", "+ std::to_string(sh) + ", "+ std::to_string(ts) + ", " + std::to_string(m_mpMapReachableNodeCount) +  ", " + std::to_string(hull_angle_pair.second) + "]");

                for(auto ip: pi_integer_hull){
                    int32_t x = ip[0];
                    int32_t y = ip[1];

                    if(x == 0 && y == 0){
                        continue;
                    }

                    MPTask nTask;    
                    nTask = {x,y,ts,sh,sv, hull_angle_pair.second};

                    m_mpTaskQueue.push_back(nTask);
                }
            } 
        }
    }
    rlog("workerMPEdges", LOG_INFO," Done, Queued " + std::to_string(m_mpTaskQueue.size()) + " tasks");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::vector<std::thread> workers;
    for(uint32_t i = 0; i < worker_count; i++){
        workers.push_back(std::thread(&MPCompute::workerThreadMPEdges, this, i));
    }
    for(uint32_t i = 0; i < worker_count; i++){
        workers.at(i).join();
    }
}


void MPCompute::writeGraphToDisk(std::string name ){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

   
    json mp_map_dump;
    mp_map_dump["info"]["m_mpMapReachableNodeCount"] = m_mpMapReachableNodeCount;
    mp_map_dump["info"]["size_a"] = map_size_angle;
    mp_map_dump["info"]["size_s"] = map_size_speed;

    // Dump array to list
    for(int32_t x = -static_cast<int32_t>(m_mpMapReachableNodeCount); x < static_cast<int32_t>(m_mpMapReachableNodeCount); x ++){
        for(int32_t y = -static_cast<int32_t>(m_mpMapReachableNodeCount); y < static_cast<int32_t>(m_mpMapReachableNodeCount); y ++){
            for(int32_t a = 0; a < map_size_angle; a ++){
                for(int32_t s = 0; s < map_size_speed; s ++){
                    //Compute position, heading and velocity of the node
                    json node;
                    node["pose"]["x"] = (dpc*x);
                    node["pose"]["y"] = (dpc*y);
                    node["pose"]["h"] = api * static_cast<float>(a);
                    node["pose"]["v"] = vlevels[s] * dynamics::SimpleDynamicsModel::velocity_limit();

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

                jedge["target"]["x"] = (dpc * edge.target.x);
                jedge["target"]["y"] = (dpc * edge.target.y);
                jedge["target"]["s"] = vlevels[edge.target.s] * dynamics::SimpleDynamicsModel::velocity_limit();
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
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    int32_t mp_sample_count = Config::getInstance().get<int32_t>({"border_detect","mp_sample_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});


    std::ifstream ifs(path);
    json jf = json::parse(ifs);
    m_mpMapReachableNodeCount = jf["info"]["m_mpMapReachableNodeCount"];

    std::cout << "Loading map with: " << jf["map"].size() << " and expected " << 2*m_mpMapReachableNodeCount * 2*m_mpMapReachableNodeCount * map_size_angle * map_size_speed <<std::endl;
    assert(jf["map"].size() ==  2*m_mpMapReachableNodeCount * 2*m_mpMapReachableNodeCount * map_size_angle * map_size_speed);
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

        for (int32_t i = 0; i < edge["curve"].size(); i += (edge["curve"].size() / mp_sample_count)) {
            dynamics::data::Pose2D pose;
            pose.pos[0] = edge["curve"].at(i)["x"];
            pose.pos[1] = edge["curve"].at(i)["y"];
            tedge.trajectory.push_back(pose);
        }

        m_mpmap[edge["source"]["a"]][edge["source"]["s"]].push_back(tedge);
    }
}
