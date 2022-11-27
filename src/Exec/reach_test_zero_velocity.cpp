
#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

int main() {


    std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
    std::cout << "Loading graph from disk!" << std::endl;
    planner->m_proxGraph.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;
    
    //Test if we can get going from any location
    // for(){
         
        dynamics::data::PoseByIndex start = {30,30,7,zero_velocity_level};
        dynamics::data::PoseByIndex end = {28,12,1,zero_velocity_level};
        std::vector<uint32_t> times;
        
        auto res = planner->astar(start,end, std::vector<dynamics::data::PBIConstraint>());
        std::cout << "Completed Astar" << std::endl;
        
        if(res.found_path){ 

            std::cout << res.path->size() << std::endl;
            planner->writePathToDisk(*res.path, "path1.json");
        }else{
            std::cout << "No viable solution found" << std::endl;
        }
        
    // }

    }




