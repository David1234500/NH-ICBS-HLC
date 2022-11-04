#include <Planner/DirectedSearchProxy.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <memory>

int main() {


std::shared_ptr<DirectedSearchProxy> graph = std::make_shared<DirectedSearchProxy>();
graph->computeProxyEdges();
std::cout << "Completed computation" << std::endl;


}
