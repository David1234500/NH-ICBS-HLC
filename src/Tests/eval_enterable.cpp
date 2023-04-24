
#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>
#include <thread>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

int main() {


    CBSPlanner planner;
    planner.preparePoseLuT();
    std::cout << "Loading graph from disk!" << std::endl;
    planner.mp_comp.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;

    static uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    static float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t step_x = Config::getInstance().getXstep();
    static int32_t step_y = Config::getInstance().getYstep();
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    static int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    static std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});


     for(uint32_t i = 0; i < worker_count; i ++){
        planner.m_lowLevelWorkers.push_back(std::thread(&CBSPlanner::low_level_astar_worker, &planner, i));
    }

    std::map<uint64_t, std::pair<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>> jobs;
    // Assumption 1: Every node for a full level is reachable
    uint32_t job_count = 0;
    for(int32_t v = 2; v < map_size_speed; v ++){
            for(int32_t tx = 0; tx < step_x; tx ++){
                for(int32_t ty = 0; ty < step_y; ty ++){
                    for(int32_t ta = 0; ta < map_size_angle; ta ++){
                    
                            dynamics::data::PoseByIndex start = {step_x/2,step_y/2,2,v};
                            dynamics::data::PoseByIndex target = {tx,ty,ta,v};
                           
                            jobs[job_count] = std::make_pair(start,target);
                            rlog("ReachCheck", LOG_INFO, "Checking reachability for position: " + std::to_string(tx) + ":" + std::to_string(ty) + ":" + std::to_string(ta));
                            
                            constraint_node node;
                            planner.enqueue_astar( start, target, node, job_count);
                            job_count += 1;
                    }
                
            }
        }
    }

    planner.await_astar_result(job_count);
    rlog("ReachCheck", LOG_INFO, "Got all reachability results... ");

   nlohmann::json unreachable_configs;
    for (uint32_t i = 0; i < planner.m_lowLevelResults.size(); i++) {
        auto res = planner.m_lowLevelResults.at(i);
        if (!res.found_path) {
            rlog("ReachCheck", LOG_INFO, "Failed for a configuration! ");

            // Retrieve the start and target poses for the unreachable configuration
            auto job = jobs.find(res.job_id);
            if (job != jobs.end()) {
                auto start_pose = job->second.first;
                auto target_pose = job->second.second;

                // Add the unreachable configuration to the JSON object
                nlohmann::json config;
                config["job_id"] = res.job_id;
                config["car_id"] = res.car_id;
                config["start_pose"] = {
                    {"x", start_pose.x},
                    {"y", start_pose.y},
                    {"a", start_pose.a},
                    {"s", start_pose.s},
                    {"t", start_pose.t},
                };
                config["target_pose"] = {
                    {"x", target_pose.x},
                    {"y", target_pose.y},
                    {"a", target_pose.a},
                    {"s", target_pose.s},
                    {"t", target_pose.t},
                };
                unreachable_configs.push_back(config);
            }else{
                assert(false);
            }
        }
    }

    // Save the unreachable configurations to a JSON file
    if (!unreachable_configs.empty()) {
        std::ofstream unreachable_configs_file("unenterable.json");
        unreachable_configs_file << unreachable_configs.dump(4); // 4 spaces for indentation
        unreachable_configs_file.close();
    }
    return 0;
}



