
#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

int main() {


    CBSPlanner planner;
    std::cout << "Loading graph from disk!" << std::endl;
    planner.m_proxGraph.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;

    
    dynamics::data::PoseByIndex end = {55,55,7,2};
    dynamics::data::PoseByIndex start = {45,2,5,2};
    std::cout << "Start AStar!" << std::endl;
    auto res = planner.astar(start,end);
    std::cout << "Completed Astar" << std::endl;
    std::cout << res.size() << std::endl;
    planner.writeCurveToDisk(res, "res2_path");
    // planner.m_proxGraph.writeGraphToDisk();

}



