
#include <MPCompute/MPNLOpt.hpp>
#include <MPCompute/MPNLOptSingle.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>

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

int main(){
    static uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    static float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    static int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    static std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});


    MPNLOptSingle opt;
    
    MPNLOptSingle::mpnl_args_t args; 
    args.sp = {{0.f,0.f},0.f,50.f};
    args.tp = {{40.f,40.f},1.0f * PI,50.f};

    opt.prepare(&args, true, false, nlopt::GN_ISRES); // GN_DIRECT,  GN_ESCH and GN_ISRES, GN_CRS2_LM (best)

    auto res = opt.optimize();

    auto rpose = MPNLOptSingle::eval_two(res.result, &args);

    std::cout << "result" << std::endl;
    std::cout << "obj " << std::to_string(res.objf_val) << std::endl;
    std::cout << "retcode " << std::to_string(res.retcode) << std::endl;
    std::cout << "rt " << std::to_string(res.rt_ms) << std::endl;
    std::cout << "c_st_a " << std::to_string(res.result[0]) << ", "<< std::to_string(res.result[1]) << std::endl;
    std::cout << "c_vcc " << std::to_string(res.result[2]) << ", "<< std::to_string(res.result[3]) << std::endl;
    std::cout << "res_pose ((" << std::to_string(rpose.pos[0]) << ", "<< std::to_string(rpose.pos[1]) << "), " + std::to_string(rpose.h) +", " + std::to_string(rpose.vel) + ")"<< std::endl;

    std::vector<dynamics::data::Pose2D> resulting_pose;
    for(float ts = 0; ts < timestep_ms; ts += 50.f){
        resulting_pose.push_back( dynamics::SimpleDynamicsModel::computeNextPose(args.sp, res.result[0], args.sp.vel + 0.5f * res.result[2], ts));
    }

    auto sp2 = resulting_pose.back();
    for(float ts = 0; ts < timestep_ms; ts += 50.f){
        resulting_pose.push_back( dynamics::SimpleDynamicsModel::computeNextPose(sp2, res.result[1],sp2.vel + 0.5f * res.result[3], ts));
    }

    writePosesToDisc(resulting_pose, "mpnlopt_single_test.json");
    return 0;
}