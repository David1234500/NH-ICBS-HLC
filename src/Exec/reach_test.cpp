
#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

int main() {


    std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
    std::cout << "Loading graph from disk!" << std::endl;
    planner->m_proxGraph.loadGraphFromDisk();
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
                    for(auto prim: planner->m_proxGraph.m_proxyEdgeList[a][s]){
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



