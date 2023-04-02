
#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

int main() {
    static uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    static float dpc = Config::getInstance().get<float>({"disc","dstep"});
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t driving_velocity_level = Config::getInstance().get<int32_t>({"velocity","driving_velocity_level"});
    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    static int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    static std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

    std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
    std::cout << "Loading graph from disk!" << std::endl;
    planner->mp_comp.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;

    
    dynamics::data::PoseByIndex end = {25,25,7,driving_velocity_level};
    dynamics::data::PoseByIndex start = {25,5,5,driving_velocity_level};
    std::vector<uint32_t> times;
    std::cout << "Start AStar!" << std::endl;
    auto tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto res = planner->astar(start,end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    if(res.found_path){ 
        std::cout << res.path->size() << std::endl;
        planner->writeCurveToDisk(res, "path1.json");
        // planner->writeCurveToDisk(res.spline, "curve.json");
    }else{
        std::cout << "No viable solution found" << std::endl;
    }
    
    auto tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);


    tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    end = {10,25,0,driving_velocity_level};
    start = {10,10,3,driving_velocity_level};
    std::cout << "Start AStar!" << std::endl;
    res = planner->astar(start,end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    
    if(res.found_path){ 
        std::cout << res.path->size() << std::endl;
        planner->writeCurveToDisk(res, "path2.json");
    }else{
        std::cout << "No viable solution found" << std::endl;
    }
    
    tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);

    tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    end = {10,25,0,zero_velocity_level};
    start = {28,25,3,zero_velocity_level};
    std::cout << "Start AStar!" << std::endl;
     res = planner->astar(start,end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    if(res.found_path){ 
        std::cout << res.path->size() << std::endl;
        planner->writeCurveToDisk(res, "path3.json");
    }else{
        std::cout << "No viable solution found" << std::endl;
    }
    
    tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);

    tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    end = {10,25,0,zero_velocity_level};
    start = {25,10,3,zero_velocity_level};
    std::cout << "Start AStar!" << std::endl;
    res = planner->astar(start,end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    if(res.found_path){ 
        std::cout << res.path->size() << std::endl;
        planner->writeCurveToDisk(res, "path4.json");
    }else{
        std::cout << "No viable solution found" << std::endl;
    }
    
    tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);
    
    for(uint32_t i = 0; i < times.size(); i ++){
        std::cout << "times " << i << " " << times.at(i) << std::endl; 
    }
}



