#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <memory>

int main() {


std::shared_ptr<ProxyGraph> graph = std::make_shared<ProxyGraph>();
graph->computeProxyEdges();
std::cout << "Completed computation" << std::endl;

std::cout << "Generating json graph with all edges..." << std::endl;
graph->writeGraphToDisk();
std::cout << "Completed generating edges..." << std::endl;
}
