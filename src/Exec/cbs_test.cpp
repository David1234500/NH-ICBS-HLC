
#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

int main() {


    CBSPlanner planner;
    std::cout << "Loading graph from disk!" << std::endl;
    planner.m_proxGraph.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;

    std::vector<dynamics::data::PoseByIndex> targets;
    std::vector<dynamics::data::PoseByIndex> starts;

    dynamics::data::PoseByIndex end = {25,25,7,2};
    dynamics::data::PoseByIndex start = {25,2,7,2};
    targets.push_back(end);
    starts.push_back(start);

    dynamics::data::PoseByIndex end2 = {10,25,7,2};
    dynamics::data::PoseByIndex start2 = {25,22,7,2};
    targets.push_back(end2);
    starts.push_back(start2);

    planner.cbs(starts,targets);
}



