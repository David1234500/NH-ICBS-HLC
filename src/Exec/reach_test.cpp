
#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

int main() {


    std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
    std::cout << "Loading graph from disk!" << std::endl;
    planner->m_proxGraph.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;
    planner->checkForReachability();

}



