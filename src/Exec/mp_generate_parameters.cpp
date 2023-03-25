#include <MPCompute/MPNLOptParameters.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <memory>

int main() {
    int32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float xpc = Config::getInstance().get<float>({"disc","xstep"});
    float ypc = Config::getInstance().get<float>({"disc","ystep"});
    float api = Config::getInstance().get<float>({"disc","hstep"});

    float fit_quality_angle = Config::getInstance().get<int>({"MPComputeBruteForce","fit_quality_angle"});
    float fit_quality_position = Config::getInstance().get<int>({"MPComputeBruteForce","fit_quality_position"});
    float fit_allowed_speed_difference = Config::getInstance().get<int>({"MPComputeBruteForce","fit_allowed_speed_difference"});

    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t x_steps = Config::getInstance().get<int32_t>({"map","x_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});

    std::shared_ptr<MPNLOptParameters> param_solver = std::make_shared<MPNLOptParameters>();

    MPNLOptParameters::mpnl_param_args_t args;
    args.st_val = 10;
    args.ts_ms = timestep_ms;

    args.obj_lam_target = 15.f;

    param_solver->prepare(&args);
    param_solver->optimize();
    std::cout << "Completed computation" << std::endl;

}
