#include <MPCompute/MPNLOptParameters.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <memory>

int main() {
    int32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    float api = Config::getInstance().get<float>({"disc","hstep"});

    float fit_quality_angle = Config::getInstance().get<int>({"MPComputeBruteForce","fit_quality_angle"});
    float fit_quality_position = Config::getInstance().get<int>({"MPComputeBruteForce","fit_quality_position"});
    float fit_allowed_speed_difference = Config::getInstance().get<int>({"MPComputeBruteForce","fit_allowed_speed_difference"});

    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});

    std::shared_ptr<MPNLOptParameters> param_solver = std::make_shared<MPNLOptParameters>();

    MPNLOptParameters::mpnl_param_args_t args;
    param_solver->prepare(&args);
    auto ret = param_solver->optimize();
    param_solver->visualize_results(&ret);

    std::cout << "Completed computation" << std::endl;
    std::cout << "Node spacing [cm]: " << std::to_string(ret.result[MPNLOptParameters::c_lamb]) << std::endl;
    std::cout << "Node spacing tolerance [cm]: " << std::to_string(ret.result[MPNLOptParameters::c_lamth]) << std::endl;
    std::cout << "Acceleration 1 [cm/s]: " << std::to_string(ret.result[MPNLOptParameters::c_acc_1]) << std::endl;
    std::cout << "Acceleration 2 [cm/s]: " << std::to_string(ret.result[MPNLOptParameters::c_acc_2]) << std::endl;
    std::cout << "Velocity Level [cm/s]: " << std::to_string((ret.result[MPNLOptParameters::c_acc_2] + ret.result[MPNLOptParameters::c_acc_1]) * (timestep_ms / 1000.f)) << std::endl;
    std::cout << "Heading tolerance [rad]: " << std::to_string(ret.result[MPNLOptParameters::c_hth]) << std::endl;
    std::cout << "Steering Angle for {0,0, hstep} -> {lambda, 0, hstep} [rad]: " << std::to_string(ret.result[MPNLOptParameters::c_st_1]) << std::endl;
    std::cout << "Steering Angle for {0,0, 0} -> {lambda, lambda, hstep} [rad]: " << std::to_string(ret.result[MPNLOptParameters::c_st_2]) << std::endl;
    std::cout << "Steering Angle for {0,0, 2 * hstep} -> {0, lambda, 2 * hstep} [rad]: " << std::to_string(ret.result[MPNLOptParameters::c_st_3]) << std::endl;

}
