
#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

int main() {
    static uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    static float xpc = Config::getInstance().get<float>({"disc","xstep"});
    static float ypc = Config::getInstance().get<float>({"disc","ystep"});
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

    if(!res.reachable){
        std::cout << "Unreachable Configuration:" << std::endl;
        uint32_t count = 0;
        for(auto unreach: res.unreachable_configurations){
            std::cout << "Configuration: " << count << std::endl; 
            std::cout << "Start: " << unreach.first.x << ":" << unreach.first.y << ":" << unreach.first.a << ":" << unreach.first.s << std::endl;
            std::cout << "Target: " << unreach.second.x << ":" << unreach.second.y << ":" << unreach.second.a << ":" << unreach.second.s << std::endl;
            count += 1;

            bool found_configuration = false;
            for(int32_t a = 0; a < map_size_angle; a ++ ){
                for(int32_t s = 0; s < map_size_speed; s ++){
                    for(auto prim: planner->mp_comp.m_mpmap[a][s]){
                        if(prim.target.a == unreach.second.a && prim.target.s == unreach.second.s){
                            std::cout << "Found node with target heading and speed config... " << prim.target.x << ":" << prim.target.y << ":" << prim.target.a << ":" << prim.target.s << std::endl;
                            found_configuration = true;
                        }
                    }
                }
            }
            if(!found_configuration){
                std::cout << "There is no node to this configuration " << unreach.second.a << ":" << unreach.second.s << std::endl;
            }
        }


    }
}



