#include <Planner/StateGraph.hpp>
#include <iostream>

int main() {


StateGraph graph;
graph.computeProxyEdges();
std::cout << "Completed computation" << std::endl;
graph.printEdges(1);
graph.printEdges(5);
graph.printEdges(10);
}
