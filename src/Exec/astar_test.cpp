
#include <Planner/DirectedSearchProxy.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

int main() {


    std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
    std::cout << "Loading graph from disk!" << std::endl;
    planner->m_proxGraph.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;

    
    dynamics::data::PoseByIndex end = {3,1,3,0};
    dynamics::data::PoseByIndex start = {23,2,7,0};
    std::vector<uint32_t> times;
    std::cout << "Start AStar!" << std::endl;
    auto tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto res = planner->astar(start,end, std::vector<dynamics::data::Constraint>());
    std::cout << "Completed Astar" << std::endl;
    if(res.found_path){ 
        std::cout << res.path->size() << std::endl;
        planner->writePathToDisk(*res.path, "path1.json");
        planner->writeCurveToDisk(res.spline, "curve.json");
    }else{
        std::cout << "No viable solution found" << std::endl;
    }
    
    auto tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);


    tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    end = {10,3,0,0};
    start = {26,26,3,0};
    std::cout << "Start AStar!" << std::endl;
    res = planner->astar(start,end, std::vector<dynamics::data::Constraint>());
    std::cout << "Completed Astar" << std::endl;
    
    if(res.found_path){ 
        std::cout << res.path->size() << std::endl;
        planner->writePathToDisk(*res.path, "path2.json");
        planner->writeCurveToDisk(res.spline, "curve2.json");
    }else{
        std::cout << "No viable solution found" << std::endl;
    }
    
    tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    times.push_back(tstop - tstart);

    // tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    // end = {3,25,0,0};
    // start = {28,3,3,0};
    // std::cout << "Start AStar!" << std::endl;
    //  res = planner->astar(start,end, std::vector<dynamics::data::Constraint>());
    // std::cout << "Completed Astar" << std::endl;
    // if(res.found_path){ 
    //     std::cout << res.path->size() << std::endl;
    //     planner->writePathToDisk(*res.path, "path3.json");
    //     planner->writeCurveToDisk(res.spline, "curve3.json");
    // }else{
    //     std::cout << "No viable solution found" << std::endl;
    // }
    
    // tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    // times.push_back(tstop - tstart);

    // tstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    // end = {10,5,0,0};
    // start = {19,5,3,0};
    // std::cout << "Start AStar!" << std::endl;
    // res = planner->astar(start,end, std::vector<dynamics::data::Constraint>());
    // std::cout << "Completed Astar" << std::endl;
    // if(res.found_path){ 
    //     std::cout << res.path->size() << std::endl;
    //     planner->writePathToDisk(*res.path, "path4.json");
    //     planner->writeCurveToDisk(res.spline, "curve4.json");
    // }else{
    //     std::cout << "No viable solution found" << std::endl;
    // }
    
    // tstop = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    // times.push_back(tstop - tstart);
    
    for(uint32_t i = 0; i < times.size(); i ++){
        std::cout << "times " << i << " " << times.at(i) << std::endl; 
    }
}



