
#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

int main() {

    static int32_t driving_velocity_level = Config::getInstance().get<int32_t>({"velocity","driving_velocity_level"});
    std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
    std::cout << "Loading graph from disk!" << std::endl;
    planner->mp_comp.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;
    
    dynamics::data::PoseByIndex start = {30,30,7,driving_velocity_level};
    dynamics::data::PoseByIndex end = {28,12,1,driving_velocity_level};
    std::vector<uint32_t> times;
    
    std::cout << "Start AStar!" << std::endl;
    auto tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto res = planner->astar(start,end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    
    if(res.found_path){ 

        std::cout << res.path->size() << std::endl;
        planner->writeCurveToDisk(res, "path1.json");
    }else{
        std::cout << "No viable solution found" << std::endl;
    }
    
    auto tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);

    for(uint32_t i = 0; i < times.size(); i ++){
        std::cout << "times " << i << " " << times.at(i) << std::endl; 
    }
}



