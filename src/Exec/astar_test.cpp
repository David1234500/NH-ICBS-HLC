
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

    std::vector<int64_t> times;

    auto tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    dynamics::data::PoseByIndex end = {10,11,0,2};
    dynamics::data::PoseByIndex start = {10,10,3,2};
    std::cout << "Start AStar!" << std::endl;
    auto res = planner.astar(start,end);
    std::cout << "Completed Astar" << std::endl;
    std::cout << res.size() << std::endl;
    planner.writeCurveToDisk(res, "res2_path");
    auto tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);


    tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    end = {10,25,0,2};
    start = {10,10,3,2};
    std::cout << "Start AStar!" << std::endl;
     res = planner.astar(start,end);
    std::cout << "Completed Astar" << std::endl;
    std::cout << res.size() << std::endl;
    planner.writeCurveToDisk(res, "res3_path");
    tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);

    tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    end = {10,25,0,2};
    start = {35,25,3,2};
    std::cout << "Start AStar!" << std::endl;
     res = planner.astar(start,end);
    std::cout << "Completed Astar" << std::endl;
    std::cout << res.size() << std::endl;
    planner.writeCurveToDisk(res, "res4_path");
    tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);

    tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    end = {10,25,0,2};
    start = {25,10,3,2};
    std::cout << "Start AStar!" << std::endl;
     res = planner.astar(start,end);
    std::cout << "Completed Astar" << std::endl;
    std::cout << res.size() << std::endl;
    planner.writeCurveToDisk(res, "res5_path");
    tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);
    
    for(uint32_t i = 0; i < times.size(); i ++){
        std::cout << "times " << i << " " << times.at(i) << std::endl; 
    }
}



