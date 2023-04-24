#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>

void writePosesToDisc(std::vector<dynamics::data::Pose2D> path, std::string name){
    nlohmann::json astar_path;
    
    for(auto current: path){
        nlohmann::json node;
        node["x"] = current.pos[0];
        node["y"] = current.pos[1];

        astar_path["path"].push_back(node);
    }
    //dump file to disc
    std::ofstream o(name);
    o << astar_path << std::endl;
    o.close();
}

int main() {

    dynamics::data::Pose2D sp = {{100.f,0.f},1.0 * PI,100.f};
    std::vector<dynamics::data::Pose2D> resulting_poses;

    for(float ts = 0; ts < 1000.f; ts += 50.f){
        resulting_poses.push_back( dynamics::SimpleDynamicsModel::computeNextPose(sp, 0.1f, 100.f, ts));
         
    }
    std::cout << "p1 " << std::to_string(resulting_poses.back().pos[0]) << ":"
                          << std::to_string(resulting_poses.back().pos[1]) << ":"
                          << std::to_string(resulting_poses.back().vel)    << ":"
                          << std::to_string(resulting_poses.back().h)     << std::endl;

    for(float ts = 0; ts < 1000.f; ts += 50.f){
        resulting_poses.push_back( dynamics::SimpleDynamicsModel::computeNextPose(sp, -0.1f, 100.f, ts));
         
    }
    std::cout << "p2 " << std::to_string(resulting_poses.back().pos[0]) << ":"
                          << std::to_string(resulting_poses.back().pos[1]) << ":"
                          << std::to_string(resulting_poses.back().vel)    << ":"
                          << std::to_string(resulting_poses.back().h)     << std::endl;

    for(float ts = 0; ts < 1000.f; ts += 50.f){
        resulting_poses.push_back( dynamics::SimpleDynamicsModel::computeNextPose(sp, 2.f * PI - 0.2f, 100.f, ts));
         
    }
    std::cout << "p3 " << std::to_string(resulting_poses.back().pos[0]) << ":"
                          << std::to_string(resulting_poses.back().pos[1]) << ":"
                          << std::to_string(resulting_poses.back().vel)    << ":"
                          << std::to_string(resulting_poses.back().h)     << std::endl;

    for(float ts = 0; ts < 1000.f; ts += 50.f){
        resulting_poses.push_back( dynamics::SimpleDynamicsModel::computeNextPose(sp, + 0.2f, 100.f, ts));
        
    }
    std::cout << "p4 " << std::to_string(resulting_poses.back().pos[0]) << ":"
                          << std::to_string(resulting_poses.back().pos[1]) << ":"
                          << std::to_string(resulting_poses.back().vel)    << ":"
                          << std::to_string(resulting_poses.back().h)     << std::endl;
    
    for(float ts = 0; ts < 1000.f; ts += 50.f){
        resulting_poses.push_back( dynamics::SimpleDynamicsModel::computeNextPose(sp, 0.4f, 100.f, ts));
        
    }
    std::cout << "p5 " << std::to_string(resulting_poses.back().pos[0]) << ":"
                          << std::to_string(resulting_poses.back().pos[1]) << ":"
                          << std::to_string(resulting_poses.back().vel)    << ":"
                          << std::to_string(resulting_poses.back().h)     << std::endl;

    for(float ts = 0; ts < 1000.f; ts += 50.f){
        resulting_poses.push_back( dynamics::SimpleDynamicsModel::computeNextPose(sp, -0.4f, 100.f, ts));
        
    }
    std::cout << "p6 " << std::to_string(resulting_poses.back().pos[0]) << ":"
                          << std::to_string(resulting_poses.back().pos[1]) << ":"
                          << std::to_string(resulting_poses.back().vel)    << ":"
                          << std::to_string(resulting_poses.back().h)     << std::endl;

    writePosesToDisc(resulting_poses, "stm_test.json");

    return 0;
}

// planner->writePathToDisk(*res.path, "path1.json");