
#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

int main() {


    ProxyGraph graph;
    std::cout << "Loading graph from disk!" << std::endl;
    graph.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;
    graph.printPositions();
    std::cout << "Finished printing positions!" << std::endl;

}



