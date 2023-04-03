
#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include <tuple>


void writeAllUnreachableConfigurationsToDisk(const std::string& filename, const std::vector<std::tuple<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex, std::vector<dynamics::data::PoseByIndex>>>& unreachable_configs) {
    json j;
    j["UnreachableConfigurations"] = json::array();

    for (const auto& unreach : unreachable_configs) {
        const dynamics::data::PoseByIndex& start = std::get<0>(unreach);
        const dynamics::data::PoseByIndex& target = std::get<1>(unreach);
        const std::vector<dynamics::data::PoseByIndex>& found_nodes = std::get<2>(unreach);

        json config;
        config["Start"] = {start.x, start.y, start.a, start.s};
        config["Target"] = {target.x, target.y, target.a, target.s};

        bool found_configuration = !found_nodes.empty();
        config["FoundConfiguration"] = found_configuration;

        if (found_configuration) {
            config["FoundNodes"] = json::array();

            for (const auto& node : found_nodes) {
                config["FoundNodes"].push_back({node.x, node.y, node.a, node.s});
            }
        }

        j["UnreachableConfigurations"].push_back(config);
    }

    std::ofstream file(filename);
    if (file.is_open()) {
        file << j.dump(4); // Indentation of 4 spaces
        file.close();
    } else {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
    }
}

void writeConfigurationToDisk(const std::string& filename, int count, const dynamics::data::PoseByIndex& start, const dynamics::data::PoseByIndex& target, bool found_configuration, const std::vector<dynamics::data::PoseByIndex>& found_nodes) {
    json j;

    j["Configuration"] = count;
    j["Start"] = {start.x, start.y, start.a, start.s};
    j["Target"] = {target.x, target.y, target.a, target.s};
    j["FoundConfiguration"] = found_configuration;

    if (found_configuration) {
        j["FoundNodes"] = json::array();

        for (const auto& node : found_nodes) {
            j["FoundNodes"].push_back({node.x, node.y, node.a, node.s});
        }
    }

    std::ofstream file(filename);
    if (file.is_open()) {
        file << j.dump(4); // Indentation of 4 spaces
        file.close();
    } else {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
    }
}

int main() {
    static uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    static float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    static int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    static std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});


    std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
    std::cout << "Loading graph from disk!" << std::endl;
    planner->mp_comp.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;
    auto res = planner->checkForReachability();

    std::vector<std::tuple<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex, std::vector<dynamics::data::PoseByIndex>>> unreachable_configs;
    if(!res.reachable){
        std::cout << "Unreachable Configuration:" << std::endl;
        uint32_t count = 0;
        for(auto unreach: res.unreachable_configurations){
            std::cout << "Configuration: " << count << std::endl; 
            std::cout << "Start: " << unreach.first.x << ":" << unreach.first.y << ":" << unreach.first.a << ":" << unreach.first.s << std::endl;
            std::cout << "Target: " << unreach.second.x << ":" << unreach.second.y << ":" << unreach.second.a << ":" << unreach.second.s << std::endl;
            count += 1;

            std::vector<dynamics::data::PoseByIndex> config;
            bool found_configuration = false;
            for(int32_t a = 0; a < map_size_angle; a ++ ){
                for(int32_t s = 0; s < map_size_speed; s ++){
                    for(auto prim: planner->mp_comp.m_mpmap[a][s]){
                        if(prim.target.a == unreach.second.a && prim.target.s == unreach.second.s){
                            std::cout << "Found node with target heading and speed config... " << prim.target.x << ":" << prim.target.y << ":" << prim.target.a << ":" << prim.target.s << std::endl;
                            config.push_back(prim.target);
                            found_configuration = true;
                        }
                    }
                }
            }
            if(!found_configuration){
                std::cout << "There is no node to this configuration " << unreach.second.a << ":" << unreach.second.s << std::endl;
            }
            unreachable_configs.push_back(std::make_tuple(unreach.first, unreach.second, config));
            writeConfigurationToDisk("unreachable_configuration" + std::to_string(count) + ".json", count, unreach.first, unreach.second, found_configuration, config);
        }
        writeAllUnreachableConfigurationsToDisk("unreachable_config.json", unreachable_configs);
    }
}



