
#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

int main() {


    std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
    std::cout << "Loading graph from disk!" << std::endl;
    planner->mp_comp.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;
    
    //Test if we can get going from any location
    // for(){
         
        dynamics::data::PoseByIndex start = {30,30,7,2};
        dynamics::data::PoseByIndex end = {28,12,5,0};
        std::vector<uint32_t> times;
        
        auto res = planner->astar(start,end, std::vector<dynamics::data::PBIConstraint>());
        std::cout << "Completed Astar" << std::endl;
        
        if(res.found_path){ 

            std::cout << res.path->size() << std::endl;
            planner->writeCurveToDisk(res, "path1.json");
        }else{
            std::cout << "No viable solution found" << std::endl;
        }
        
    // }

    }




