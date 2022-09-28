#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>

int main() {


auto prxy = std::make_shared<ProxyGraph>();
prxy->computeProxyEdges();
std::cout << "Completed computation" << std::endl;

std::cout << "Generating json graph with all edges..." << std::endl;
prxy->writeGraphToDisk();
std::cout << "Completed generating edges..." << std::endl;
}
