#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

void run_cbs_experiment(const std::string& input_filename, const std::string& output_filename) {
    auto& config = Config::getInstance("config.json");
    
    std::ifstream input_file(input_filename);
    json experiments_json;
    input_file >> experiments_json;
    
    std::cout << "Finished loading graph from disk!" << std::endl;

    static int32_t driving_velocity_level = Config::getInstance().get<int32_t>({"velocity","driving_velocity_level"});

    json result_json;
    size_t experiment_index = 0;
    for (const auto& experiment : experiments_json) {
        std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
        std::cout << "Loading graph from disk!" << std::endl;
        planner->mp_comp.loadGraphFromDisk();

        std::vector<dynamics::data::PoseByIndex> starts;
        std::vector<dynamics::data::PoseByIndex> targets;

        for (const auto& start_json : experiment["starts"]) {
            starts.push_back({
                start_json["x"].get<int32_t>(),
                start_json["y"].get<int32_t>(),
                start_json["a"].get<int32_t>(),
                start_json["s"].get<int32_t>(),
                start_json["t"].get<int32_t>()
            });
        }

        for (const auto& target_json : experiment["targets"]) {
            targets.push_back({
                target_json["x"].get<int32_t>(),
                target_json["y"].get<int32_t>(),
                target_json["a"].get<int32_t>(),
                target_json["s"].get<int32_t>(),
                target_json["t"].get<int32_t>()
            });
        }

        auto start_time = system_clock::now();
        auto result = planner->cbs(starts, targets);
        auto end_time = system_clock::now();
        auto elapsed_time = duration_cast<milliseconds>(end_time - start_time).count();
        
        if(end_time - start_time > std::chrono::duration(seconds(255))){
            result.feasible = false;
        }

        json experiment_result = {
            {"experiment_index", experiment_index},
            {"feasible", result.feasible},
            {"compute_time", elapsed_time},
            {"max_node_id", result.node_id}
        };

        if (result.feasible) {
            json paths, interprimitives;
            for (const auto& res : result.result) {
                json interprimitive = json::array();
                for (const auto& p : res.second.interprimitive) {
                    json baseNode = {
                        {"x", p.baseNode.x},
                        {"y", p.baseNode.y},
                        {"a", p.baseNode.a},
                        {"s", p.baseNode.s},
                        {"t", p.baseNode.t}
                    };
                    json j = {
                        {"posx", p.pos[0]},
                        {"posy", p.pos[1]},
                        {"h", p.h},
                        {"vel", p.vel},
                        {"time_ms", p.time_ms},
                        {"baseNode",baseNode}
                    };
                    interprimitive.push_back(j);
                }
                interprimitives[std::to_string(res.first)] = interprimitive;
            }
            experiment_result["interprimitives"] = interprimitives;
        }

        result_json.push_back(experiment_result);
        experiment_index++;
    }

    std::ofstream output_file(output_filename);
    output_file << result_json.dump(4);
}

int main(int argc, char** argv) {
    std::string input_filename = "./experiment.json";
    std::string output_filename = "./results.json";
    run_cbs_experiment(input_filename, output_filename);
    return 0;
}
