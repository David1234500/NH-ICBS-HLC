
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

    
    dynamics::data::PoseByIndex end = {25,25,7,2};
    dynamics::data::PoseByIndex start = {25,2,5,2};
    std::vector<uint32_t> times;
    std::cout << "Start AStar!" << std::endl;
    auto tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto res = planner.astar(start,end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    std::cout << res->size() << std::endl;
    
    auto tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);


    tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    end = {10,25,0,2};
    start = {10,10,3,2};
    std::cout << "Start AStar!" << std::endl;
     res = planner.astar(start,end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    std::cout << res->size() << std::endl;
    
    tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);

    tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    end = {10,25,0,2};
    start = {28,25,3,2};
    std::cout << "Start AStar!" << std::endl;
     res = planner.astar(start,end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    std::cout << res->size() << std::endl;
    
    tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);

    tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    end = {10,25,0,2};
    start = {25,10,3,2};
    std::cout << "Start AStar!" << std::endl;
     res = planner.astar(start,end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    std::cout << res->size() << std::endl;
    
    tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);
    
    for(uint32_t i = 0; i < times.size(); i ++){
        std::cout << "times " << i << " " << times.at(i) << std::endl; 
    }
}



