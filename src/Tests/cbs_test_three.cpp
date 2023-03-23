
#include <Planner/ProxyGraph.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

int main() {

// START 35:22:9:1
// TARGET 6:6:6:1
// [G2F]Got vehicle: 2 with position 3.534605:1.440412 and heading -1.124394
// Log at time 1671197896879665028, level 1: [G2F]Got vehicle: 2 with position 3.534605:1.440412 and heading -1.124394
// [G2F]Got vehicle: 2 with position 3.534605:1.440412 and heading -1.124394
// Log at time 1671197896882812674, level 1: [G2F]Got vehicle: 2 with position 3.534605:1.440412 and heading -1.124394
// START 71:29:10:1
// TARGET 16:6:6:1
// [G2F]Got vehicle: 3 with position 2.646776:0.953318 and heading -2.911921
// Log at time 1671197896884406504, level 1: [G2F]Got vehicle: 3 with position 2.646776:0.953318 and heading -2.911921
// [G2F]Got vehicle: 3 with position 2.646776:0.953318 and heading -2.911921
// Log at time 1671197896886549368, level 1: [G2F]Got vehicle: 3 with position 2.646776:0.953318 and heading -2.911921
// START 53:19:6:1
// TARGET 26:6:6:1

    CBSPlanner planner;
    std::cout << "Loading graph from disk!" << std::endl;
    planner.m_proxGraph.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;

    std::vector<dynamics::data::PoseByIndex> targets;
    std::vector<dynamics::data::PoseByIndex> starts;

    dynamics::data::PoseByIndex end = { 6,6,6,1};
    dynamics::data::PoseByIndex start = {35,22,9,1};
    targets.push_back(end);
    starts.push_back(start);

    dynamics::data::PoseByIndex end2 = {16,6,6,1};
    dynamics::data::PoseByIndex start2 = {71,29,10,1};
    targets.push_back(end2);
    starts.push_back(start2);

    dynamics::data::PoseByIndex end3 = {26,6,6,1};
    dynamics::data::PoseByIndex start3 = {53,19,6,1};
    targets.push_back(end3);
    starts.push_back(start3);

    std::cout << "Finished a star computation" << std::endl;
    

    auto tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto result = planner.cbs(starts,targets);
    auto tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    
    std::cout << "computation took:" << tstop - tstart << std::endl;
    std::cout << "Finished a cbs computation" << std::endl;
    planner.writeMultiplePathsToDisk(result, "cbs_result.json");


}



