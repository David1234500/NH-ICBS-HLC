#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <cstdlib>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

std::string getTimestamp() {
    auto now = std::time(nullptr);
    std::stringstream timestamp;
    timestamp << std::put_time(std::localtime(&now), "%Y%m%d%H%M%S");
    return timestamp.str();
}

nlohmann::json to_json(const constraint_node& node) {
    nlohmann::json json_node;

    json_node["sic"] = node.sic;
    json_node["node_id"] = node.node_id;
    json_node["father"] = node.father;
    json_node["feasible"] = node.feasible;

    nlohmann::json avoid_constraints = nlohmann::json::array();
    for (const auto& constraint : node.avoid) {
        nlohmann::json json_constraint;
        json_constraint["x"] = constraint.x;
        json_constraint["y"] = constraint.y;
        json_constraint["t"] = constraint.t;
        json_constraint["id"] = constraint.id;
        avoid_constraints.push_back(json_constraint);
    }

    json_node["avoid"] = avoid_constraints;

    return json_node;
}

struct Task {
    dynamics::data::PoseByIndex start2;
    dynamics::data::PoseByIndex end2;
};

std::mutex tasks_mutex;
std::mutex results_mutex;
std::condition_variable tasks_cv;
std::atomic<bool> all_tasks_generated(false);

void worker_thread(nlohmann::json &infeasible_configs, std::vector<Task> &tasks, dynamics::data::PoseByIndex startv1, dynamics::data::PoseByIndex targetv2) {
    CBSPlanner planner;
    planner.preparePoseLuT();
    planner.mp_comp.loadGraphFromDisk();

    while (true) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(tasks_mutex);
            tasks_cv.wait(lock, [&] { return !tasks.empty() || all_tasks_generated; });

            if (tasks.empty()) {
                break;
            }

            std::cout << std::to_string(tasks.size()) << std::endl;

            task = tasks.back();
            tasks.pop_back();
        }

        std::vector<dynamics::data::PoseByIndex> targets;
        std::vector<dynamics::data::PoseByIndex> starts;
        
        starts.push_back(startv1);
        starts.push_back(task.start2);

        targets.push_back(targetv2);
        targets.push_back(task.end2);

        

        auto result = planner.cbs(starts, targets);

        if (!result.feasible) {
            std::unique_lock<std::mutex> lock(results_mutex);
            nlohmann::json config;
            config["car_id"] = 1;
            config["start_pose"] = {
                {"x", task.start2.x},
                {"y", task.start2.y},
                {"a", task.start2.a},
                {"s", task.start2.s},
                {"t", task.start2.t},
            };
            config["target_pose"] = {
                {"x", task.end2.x},
                {"y", task.end2.y},
                {"a", task.end2.a},
                {"s", task.end2.s},
                {"t", task.end2.t},
            };
            config["cbs_result"] = to_json(result);
            infeasible_configs["v2"].push_back(config); 
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <vehicle_one_start_x> <vehicle_one_start_y> <vehicle_one_target_x> <vehicle_one_target_y>" << std::endl;
        return 1;
    }

    int vehicle_one_start_x = std::atoi(argv[1]);
    int vehicle_one_start_y = std::atoi(argv[2]);
    int vehicle_one_target_x = std::atoi(argv[3]);
    int vehicle_one_target_y = std::atoi(argv[4]);
    int vehicle_one_start_h = std::atoi(argv[5]);
    int vehicle_one_target_h = std::atoi(argv[6]);

    CBSPlanner planner;
    planner.preparePoseLuT();
    std::cout << "Loading graph from disk!" << std::endl;
    planner.mp_comp.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;

    std::vector<dynamics::data::PoseByIndex> targets;
    std::vector<dynamics::data::PoseByIndex> starts;

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

    dynamics::data::PoseByIndex end = {vehicle_one_target_x, vehicle_one_target_y, vehicle_one_start_h, zero_velocity_level};
    dynamics::data::PoseByIndex start = {vehicle_one_start_x, vehicle_one_start_y, vehicle_one_target_h, zero_velocity_level};
    targets.push_back(end);
    starts.push_back(start);

    nlohmann::json infeasible_configs;

    json config2;
    config2["start_pose"] = {
                        {"x", start.x},
                        {"y", start.y},
                        {"a", start.a},
                        {"s", start.s},
                        {"t", start.t},
                    };

    config2["target_pose"] = {
                        {"x", end.x},
                        {"y", end.y},
                        {"a", end.a},
                        {"s", end.s},
                        {"t", end.t},
                    };

    infeasible_configs["v1"] = config2;

    std::vector<Task> tasks;

    // Generate tasks for all possible start configurations for the second vehicle
    for (int32_t tx = 0; tx < step_x; tx++) {
        for (int32_t ty = 0; ty < step_y; ty++) {
            for (int32_t ta = 0; ta < map_size_angle; ta++) {
                dynamics::data::PoseByIndex end2 = {step_x / 2, step_y / 2, 6, zero_velocity_level};
                dynamics::data::PoseByIndex start2 = {tx, ty, ta, zero_velocity_level};
                tasks.push_back({start2, end2});
            }
        }
    }

    std::vector<std::thread> threads;
    int thread_count = 20;
    for (int i = 0; i < thread_count; i++) {
        threads.emplace_back(worker_thread, std::ref(infeasible_configs), std::ref(tasks), start ,end);
    }

    all_tasks_generated = true;
    tasks_cv.notify_all();

    for (auto &thread : threads) {
        thread.join();
    }

    // Save the infeasible configurations to a JSON file
    std::ofstream infeasible_configs_file("infeasible_configs_2v_" + getTimestamp() + ".json");
    infeasible_configs_file << infeasible_configs.dump(4);
    infeasible_configs_file.close();

    std::cout << "Finished all CBS computations" << std::endl;

    return 0;
}