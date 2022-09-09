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
}
