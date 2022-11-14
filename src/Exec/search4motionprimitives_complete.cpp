#include <Planner/CBSPlanner.hpp>
#include <Planner/Logger.hpp>
#include <iostream>

#include <nlohmann/json.hpp>
#include <fstream> 

using json = nlohmann::json;

int main() {


std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
json result;

for(float test_timestep_base = 200; test_timestep_base <= 700.f; test_timestep_base += 100.f){
    for(float test_timestep_extent =  100; test_timestep_extent <= test_timestep_base; test_timestep_extent += 100.f){
        for(float quality = 1.f; quality < 6.f; quality += 1.f){
        
        json test;
        test["settings"]["m_config_baseVelocityFactor"] =  planner->m_proxGraph.m_config_baseVelocityFactor;
        test["settings"]["m_base_node_distance"] =  planner->m_proxGraph.m_base_node_distance;

        for(auto vel: planner->m_proxGraph.m_config_speedsFactor){
            test["settings"]["m_speedsFactor"].push_back(vel);
        }

        test["settings"]["m_config_map_size_zero_velocity_level"] = planner->m_proxGraph.m_config_map_size_zero_velocity_level;
        test["settings"]["m_config_map_size_speed"] = planner->m_proxGraph.m_config_map_size_speed;
        test["settings"]["m_config_map_extent"] = planner->m_proxGraph.m_config_map_extent;

       
        test["settings"]["m_config_map_size_x_cm"] = planner->m_proxGraph.m_config_map_size_x_cm;
        test["settings"]["m_config_map_size_y_cm"] = planner->m_proxGraph.m_config_map_size_y_cm;
        test["settings"]["m_config_map_size_angle"] = planner->m_proxGraph.m_config_map_size_angle;

        
        planner->m_proxGraph.m_config_state_change_fit_quality_angle = 1.f/quality;    
        test["settings"]["m_config_state_change_fit_quality_angle"] = 1.f/quality;

        planner->m_proxGraph.m_config_state_change_fit_quality_position = 1.f/quality;    
        test["settings"]["m_config_state_change_fit_quality_position"] = 1.f/quality;


        planner->m_proxGraph.m_config_ts_base = test_timestep_base;
        test["settings"]["m_config_ts_base"] = planner->m_proxGraph.m_config_ts_base;
        planner->m_proxGraph.m_config_ts_min = test_timestep_base - test_timestep_extent;
        test["settings"]["m_config_ts_min"] = planner->m_proxGraph.m_config_ts_min;
        planner->m_proxGraph.m_config_ts_max = test_timestep_base + test_timestep_extent;
        test["settings"]["m_config_ts_max"] = planner->m_proxGraph.m_config_ts_max;
        

        std::string name = "mp_eval_" + std::to_string(test_timestep_base) + "_" + std::to_string(test_timestep_extent)  + "_" + std::to_string(quality);
        rlog("mp_space", LOG_INFO, "PARAMETER TEST: " + std::to_string(test_timestep_base) + ":" + std::to_string(test_timestep_extent));
        
        planner->m_proxGraph.computeProxyEdges();

        test["computed"]["m_comp_map_size_x"] = planner->m_proxGraph.m_comp_map_size_x;
        test["computed"]["m_comp_map_size_y"] = planner->m_proxGraph.m_comp_map_size_y;

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