#include <MPComputeNL/MPNLOptStagedMultiStart.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <memory>

int main() {


std::shared_ptr<MPNLOptStagedMultiStart> graph = std::make_shared<MPNLOptStagedMultiStart>();
graph->computeMPEdges();
std::cout << "Completed computation" << std::endl;

std::cout << "Generating json graph with all edges..." << std::endl;
graph->writeGraphToDisk();
std::cout << "Completed generating edges..." << std::endl;
}
