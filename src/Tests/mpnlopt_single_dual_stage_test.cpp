
#include <MPCompute/MPNLOpt.hpp>
#include <MPCompute/MPNLOptSingleDualStage.hpp>
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
    MPNLOptSingleDualStage opt;
    
    MPNLOptSingleDualStage::mpnl_args_t args; 
    args.sp = {{0.f,0.f},0.f,100.f};
    args.tp = {{150.f,0.f},0.0f * PI,100.f};

    opt.prepare(args, true, false, nlopt::GN_ISRES); // GN_DIRECT,  GN_ESCH and GN_ISRES, GN_CRS2_LM (best)

    auto res = opt.optimize();

    auto rpose = MPNLOptSingleDualStage::eval_two(res.result_prim, &args);

    std::cout << "result primary" << std::endl;
    std::cout << "obj " << std::to_string(res.objf_val_prim) << std::endl;
    std::cout << "retcode " << std::to_string(res.retcode_prim) << std::endl;
    std::cout << "rt " << std::to_string(res.rt_ms_prim) << std::endl;
    std::cout << "c_st_a " << std::to_string(res.result_prim[0]) << ", "<< std::to_string(res.result_prim[1]) << std::endl;
    std::cout << "c_vcc " << std::to_string(res.result_prim[2]) << ", "<< std::to_string(res.result_prim[3]) << std::endl;
    std::cout << "res_pose ((" << std::to_string(rpose.pos[0]) << ", "<< std::to_string(rpose.pos[1]) << "), " + std::to_string(rpose.h) +", " + std::to_string(rpose.vel) + ")"<< std::endl;
    
    rpose = MPNLOptSingleDualStage::eval_two(res.result_sec, &args);
    std::cout << "\n result secondary" << std::endl;
    std::cout << "obj " << std::to_string(res.objf_val_sec) << std::endl;
    std::cout << "retcode " << std::to_string(res.retcode_sec) << std::endl;
    std::cout << "rt " << std::to_string(res.rt_ms_sec) << std::endl;
    std::cout << "c_st_a " << std::to_string(res.result_sec[0]) << ", "<< std::to_string(res.result_sec[1]) << std::endl;
    std::cout << "c_vcc " << std::to_string(res.result_sec[2]) << ", "<< std::to_string(res.result_sec[3]) << std::endl;
    std::cout << "res_pose ((" << std::to_string(rpose.pos[0]) << ", "<< std::to_string(rpose.pos[1]) << "), " + std::to_string(rpose.h) +", " + std::to_string(rpose.vel) + ")"<< std::endl;


    // std::vector<dynamics::data::Pose2D> resulting_pose;
    // for(float ts = 0; ts < timestep_ms ts += 50.f){
    //     resulting_pose.push_back( dynamics::SimpleDynamicsModel::computeNextPose(args.sp, res.result[0], args.sp.vel + res.result[2], ts));
    // }

    // auto sp2 = resulting_pose.back();
    // for(float ts = 0; ts < timestep_ms ts += 50.f){
    //     resulting_pose.push_back( dynamics::SimpleDynamicsModel::computeNextPose(sp2, res.result[1],sp2.vel + res.result[3], ts));
    // }

    // writePosesToDisc(resulting_pose, "mpnlopt_single_test.json");
    return 0;
}