#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <Planner/Logger.hpp>
#include <iostream>

#include <nlohmann/json.hpp>
#include <fstream> 

using json = nlohmann::json;

int main() {


std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
json result;

for(float test_timestep = 100.f; test_timestep <= 600.f; test_timestep += 50.f){
    for(uint32_t node_count = 40; node_count < 150; node_count += 10){
        for(float quality = 1.f; quality < 6.f; quality += 0.5f){

        state_change_fit_quality_angle = api / quality;
        state_change_fit_quality_position =  xpc / quality;
        recompute_inferred_values();

        json test;
        test["settings"]["quality"] = quality;
        test["settings"]["test_timestep"] = test_timestep;
        
        timestep_ms = test_timestep;
        test["settings"]["node_count"] = node_count;
        map_size_x = node_count;
        map_size_y = node_count;

        test["settings"]["map_size_angle"] = map_size_angle;
        test["settings"]["xpc"] = xpc;
        test["settings"]["ypc"] = ypc;
        test["settings"]["api"] = api;

        test["settings"]["safe_radius"] = safe_radius;
        test["settings"]["zero_velocity_level"] = zero_velocity_level;

        for(auto vel: m_speedsFactor){
            test["settings"]["m_speedsFactor"].push_back(vel);
        }

        test["settings"]["state_change_fit_quality_angle"] = state_change_fit_quality_angle;
        test["settings"]["state_change_fit_quality_position"] = state_change_fit_quality_position;
        test["settings"]["state_change_fit_allowed_speed_difference"] = state_change_fit_allowed_speed_difference;

        std::string name = "mp_eval_" + std::to_string(test_timestep) + "_" + std::to_string(node_count);
        rlog("mp_space", LOG_INFO, "PARAMETER TEST: " + std::to_string(test_timestep) + ":" + std::to_string(node_count));
        
        planner->m_proxGraph.computeProxyEdges();
        planner->m_proxGraph.writeGraphToDisk(name + ".json");

        test["result"]["graph"] = name + ".json";
        auto reach = planner->checkForReachability();
        test["result"]["reachability"] = reach.reachable;
        test["result"]["edge_count"] = reach.edge_count;
        test["result"]["mean_path_length"] = reach.mean_path_length;

        std::chrono::milliseconds msc = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        uint32_t ms = msc.count();
        test["time"] = ms;

        std::ofstream o(name + "_info.json");
        o << test << std::endl;
        o.close();
        }
    }
}

std::cout << "Completed computation" << std::endl;

}
