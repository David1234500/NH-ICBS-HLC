
#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

int main() {


    CBSPlanner planner;
    planner.m_proxGraph.computeProxyEdges();
    std::cout << "Completed computation" << std::endl;

    std::cout << "Generating json graph with all edges..." << std::endl;
    planner.m_proxGraph.writeGraphToDisk();
    std::cout << "Completed generating edges..." << std::endl;

    dynamics::data::PoseByIndex start = {5,5,5,3};
    dynamics::data::PoseByIndex end = {35,35,35,3};

    auto res = planner.astar(start,end);
    std::cout << "Completed Astar" << std::endl;
    std::cout << res.size() << std::endl;
    planner.writePathToDisk(res,end);
    std::cout << "Wrote res to disc" << std::endl;
}



