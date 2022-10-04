
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

    dynamics::data::PoseByIndex end = {25,60,7,0};
    dynamics::data::PoseByIndex start = {25,5,5,0};
    targets.push_back(end);
    starts.push_back(start);

    dynamics::data::PoseByIndex end2 = {25,5,7,1};
    dynamics::data::PoseByIndex start2 = {25,40,7,1};
    targets.push_back(end2);
    starts.push_back(start2);

    dynamics::data::PoseByIndex end3 = {10,2,6,1};
    dynamics::data::PoseByIndex start3 = {40,10,7,1};
    targets.push_back(end3);
    starts.push_back(start3);

    planner.astar(start, end, std::vector<dynamics::data::PBIConstraint>());
    planner.astar(start2, end2, std::vector<dynamics::data::PBIConstraint>());


    std::cout << "Finished a star computation" << std::endl;
    auto result = planner.cbs(starts,targets);
    std::cout << "Finished a cbs computation" << std::endl;
    planner.writeMultiplePathsToDisk(result, "cbs_result.json");

    planner.writeCurveToDisk(result.result[0].spline, "cbs_track_car_0.json");
    planner.writeCurveToDisk(result.result[1].spline, "cbs_track_car_1.json");
    planner.writeCurveToDisk(result.result[2].spline, "cbs_track_car_2.json");


}



