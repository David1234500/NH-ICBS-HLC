#include <Planner/ProxyGraph.hpp>
#include <iostream>

int main() {


ProxyGraph graph;
graph.computeProxyEdges();
std::cout << "Completed computation" << std::endl;

std::cout << "Generating json positions..." << std::endl;
graph.printPositions();
std::cout << "Generating json graph with all edges..." << std::endl;
graph.printEdges();
std::cout << "Completed generating edges..." << std::endl;
}
