
#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>

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
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    static int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    static std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});
    static int32_t driving_velocity_level = Config::getInstance().get<int32_t>({"velocity","driving_velocity_level"});

    dynamics::data::PoseByIndex end = {5,25,7,zero_velocity_level};
    dynamics::data::PoseByIndex start = {25,3,5,zero_velocity_level};
    std::vector<uint32_t> times;
    std::cout << "Start AStar!" << std::endl;
    auto tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    std::vector<dynamics::data::PBIConstraint> constraints;
    dynamics::data::PBIConstraint constr = {28,-7,3,driving_velocity_level};
    constraints.push_back(constr);
    dynamics::data::PBIConstraint constr2 = {26,7,6,driving_velocity_level};
    constraints.push_back(constr2);
    //6:9:4:2 index: 8
    // dynamics::data::PBIConstraint constr3 = {6,9,4,0};
    // constraints.push_back(constr3);

    auto res = planner.astar(start,end, constraints);
    std::cout << "Completed Astar" << std::endl;
    std::cout << res.path->size() << std::endl;
    planner.writeCurveToDisk(res, "llres1.json");

    
    for(uint32_t i = 0; i < times.size(); i ++){
        std::cout << "times " << i << " " << times.at(i) << std::endl; 
    }
}



