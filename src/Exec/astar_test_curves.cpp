
#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

int main() {


    CBSPlanner planner;
    std::cout << "Loading graph from disk!" << std::endl;
    planner.m_proxGraph.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;

    
    dynamics::data::PoseByIndex end = {5,25,7,2};
    dynamics::data::PoseByIndex start = {38,2,5,2};
    std::vector<uint32_t> times;
    std::cout << "Start AStar!" << std::endl;
    auto tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto res = planner.astar(start,end, std::unordered_set<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    std::cout << res.path->size() << std::endl;
    planner.writePathToDisk(*res.path, "path1.json");
    planner.writeCurveToDisk(res.spline, "spline1.json");
    
    for(uint32_t i = 0; i < times.size(); i ++){
        std::cout << "times " << i << " " << times.at(i) << std::endl; 
    }
}



