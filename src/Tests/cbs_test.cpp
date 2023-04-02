
#include <MPCompute/MPCompute.hpp>
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
    planner->mp_comp.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;

    static int32_t driving_velocity_level = Config::getInstance().get<int32_t>({"velocity","driving_velocity_level"});

    std::vector<dynamics::data::PoseByIndex> targets;
    std::vector<dynamics::data::PoseByIndex> starts;

    dynamics::data::PoseByIndex end = {6,6,6,driving_velocity_level};
    dynamics::data::PoseByIndex start = {30,35,4,driving_velocity_level};
    targets.push_back(end);
    starts.push_back(start);

    dynamics::data::PoseByIndex end2 = {30,20,4,driving_velocity_level};
    dynamics::data::PoseByIndex start2 = {16,6,6,driving_velocity_level};
    targets.push_back(end2);
    starts.push_back(start2);

   planner->astar(start, end, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "astar 2" << std::endl;
   planner->astar(start2, end2, std::vector<dynamics::data::PBIConstraint>());


    std::cout << "Finished a star computation" << std::endl;
    auto result =planner->cbs(starts,targets);
    std::cout << "Finished a cbs computation" << std::endl;
   planner->writeMultiplePathsToDisk(result, "cbs_result.json");

   planner->writeCurveToDisk(result.result[0], "ll_res_0.json");
   planner->writeCurveToDisk(result.result[1], "ll_res_1.json");

}



