#include <MPComputeNL/MPNLOptParameters.hpp>
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

    int32_t uni_sample_count = Config::getInstance().get<int32_t>({"mpnl_param_args","uni_sample_count"});
    int32_t rand_sample_count = Config::getInstance().get<int32_t>({"mpnl_param_args","rand_sample_count"});

    auto ub = Config::getInstance().get<std::vector<double>>({"mpnl_param_args", "ub"});
    auto lb = Config::getInstance().get<std::vector<double>>({"mpnl_param_args", "lb"});

    std::vector<double> lower(MPNLOptParameters::c_acc_12 + 1);
    lower.at(MPNLOptParameters::c_lamb) = lb[0];
    lower.at(MPNLOptParameters::c_lamth) = lb[1];
    lower.at(MPNLOptParameters::c_hth) = lb[2];
    for(uint32_t i = MPNLOptParameters::c_st_1; i <= MPNLOptParameters::c_st_212; i ++){
        lower.at(i) = lb[3];
    }
    for(uint32_t i = MPNLOptParameters::c_acc_1; i <= MPNLOptParameters::c_acc_12; i ++){
        lower.at(i) = lb[4];
    }
    
        
    std::vector<double> upper(MPNLOptParameters::c_acc_12 + 1);
    upper.at(MPNLOptParameters::c_lamb) = ub[0];
    upper.at(MPNLOptParameters::c_lamth) = ub[1];
    upper.at(MPNLOptParameters::c_hth) = ub[2];
    for(uint32_t i = MPNLOptParameters::c_st_1; i <= MPNLOptParameters::c_st_212; i ++){
        upper.at(i) = ub[3];
    }
    for(uint32_t i = MPNLOptParameters::c_acc_1; i <= MPNLOptParameters::c_acc_12; i ++){
        upper.at(i) = ub[4];
    }

    std::cout << "Upper: ";
    for(auto v: upper){
            std::cout << ", "<< std::to_string(v);
    }
     std::cout << std::endl;

    std::cout << "Lower: ";
    for(auto v: lower){
            std::cout << ", "<< std::to_string(v);
    }
     std::cout << std::endl;

    auto rand_samples = generateRandomSamples(lower, upper, rand_sample_count);
    std::vector<std::vector<double>> samples;
    // samples.insert(samples.end(), uni_samples.begin(), uni_samples.end());
    samples.insert(samples.end(), rand_samples.begin(), rand_samples.end());
    
    uint32_t id = 0;
    for(auto sample: samples){
        
        std::cout << "Sample Vec: ";
        for(auto v: sample){
            std::cout << ", " << std::to_string(v);
        }
        std::cout << std::endl;


        std::shared_ptr<MPNLOptParameters> param_solver = std::make_shared<MPNLOptParameters>();

        MPNLOptParameters::mpnl_param_args_t args;
        auto obj = param_solver->prepare(&args, nlopt::GN_CRS2_LM);
        args.init_guess = sample;

        auto ret = param_solver->optimize();
        

        if(ret.retcode > 0 && ret.retcode < 4){
            param_solver->visualize_results(&ret,obj, id, true);
            std::cout << "Completed computation with success!" << std::endl;
            std::cout << "Node spacing [cm]: " << std::to_string(ret.result[MPNLOptParameters::c_lamb]) << std::endl;
            std::cout << "Node spacing tolerance [cm]: " << std::to_string(ret.result[MPNLOptParameters::c_lamth]) << std::endl;
            std::cout << "Acceleration 1 [cm/s]: " << std::to_string(ret.result[MPNLOptParameters::c_acc_1]) << std::endl;
            std::cout << "Acceleration 2 [cm/s]: " << std::to_string(ret.result[MPNLOptParameters::c_acc_2]) << std::endl;
            std::cout << "Velocity Level [cm/s]: " << std::to_string((ret.result[MPNLOptParameters::c_acc_2] + ret.result[MPNLOptParameters::c_acc_1]) * (timestep_ms / 1000.f)) << std::endl;
            std::cout << "Heading tolerance [rad]: " << std::to_string(ret.result[MPNLOptParameters::c_hth]) << std::endl;
            std::cout << "Result Vec: ";
            for(auto v: ret.result){
                std::cout << std::to_string(v);
            }
            std::cout << std::endl;
            break;
        }else{
            param_solver->visualize_results(&ret,obj, id, false);
            std::cout << "Completed computation but failed!" << std::endl;
        }
        id += 1;
    }


}
