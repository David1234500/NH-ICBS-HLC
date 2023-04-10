
#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

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
    planner->preparePoseLuT();
    std::cout << "Finished loading graph from disk!" << std::endl;
   
    dynamics::data::PoseByIndex start = {30,30,7,zero_velocity_level};

    dynamics::data::PoseByIndex end = {28,20,5,zero_velocity_level};
    std::vector<uint32_t> times;
    
    auto res = planner->astar(start, end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    
    if(res.found_path){ 

        std::cout << res.path->size() << std::endl;
        planner->writeCurveToDisk(res, "path1.json");
    }else{
        std::cout << "No viable solution found" << std::endl;
    }
        

    }




