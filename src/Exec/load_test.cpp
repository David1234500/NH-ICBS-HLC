
#include <Planner/DirectedSearchProxy.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

int main() {


    DirectedSearchProxy graph;
    std::cout << "Loading graph from disk!" << std::endl;
    graph.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;
    
    std::cout << "Finished printing positions!" << std::endl;

}



