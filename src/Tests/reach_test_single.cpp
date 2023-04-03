
#include <MPCompute/MPCompute.hpp>
#include <Planner/CBSPlanner.hpp>
#include <iostream>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

dynamics::data::PoseByIndex parse_pose(const std::string& pose_str) {
    dynamics::data::PoseByIndex pose;
    std::stringstream ss(pose_str);
    char delimiter;

    ss >> pose.x >> delimiter >> pose.y >> delimiter >> pose.a >> delimiter >> pose.s;

    return pose;
}

int main(int argc, char* argv[]) {

    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t x_steps = Config::getInstance().getXstep();
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});
    static int32_t driving_velocity_level = Config::getInstance().get<int32_t>({"velocity","driving_velocity_level"});
    
    std::shared_ptr<CBSPlanner> planner = std::make_shared<CBSPlanner>();
    std::cout << "Loading graph from disk!" << std::endl;
    planner->mp_comp.loadGraphFromDisk();
    std::cout << "Finished loading graph from disk!" << std::endl;
    
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <start_pose> <target_pose>" << std::endl;
        return 1;
    }

    std::string start_str = argv[1];
    std::string target_str = argv[2];

    dynamics::data::PoseByIndex start = parse_pose(start_str);
    dynamics::data::PoseByIndex target = parse_pose(target_str);

    std::cout << "Start: " << start.x << ":" << start.y << ":" << start.a << ":" << start.s << std::endl;
    std::cout << "Target: " << target.x << ":" << target.y << ":" << target.a << ":" << target.s << std::endl;

    

    // Assumption 2: We can leave each configuration through at least one edge (so can always start moving)
    rlog("ReachCheck", LOG_INFO, "Checking for unleavable start configurations...");

    if(planner->mp_comp.m_mpmap[start.a][start.s].empty()){
        rlog("ReachCheck", LOG_WARNING, "Found unleavable source configuration " + std::to_string(start.a) + ":" + std::to_string(start.s));
    }
      

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Assumption 3: We can reach each configuration through at least one edge (so can always stop moving)
    rlog("ReachCheck", LOG_INFO, "Checking for unreachable end configurations...");
   
    bool found_link = false;
    for(int32_t sh = 0; sh < map_size_angle && !found_link; sh ++ ){
        for(int32_t sv = zero_velocity_level; sv < map_size_speed && !found_link; sv ++){
            for(auto prim: planner->mp_comp.m_mpmap[sh][sv]){

                if(prim.target.a == target.a && prim.target.s == target.s){
                    found_link = true;
                    break;
                }
            
            }
        }
    }

    if(!found_link){
            rlog("ReachCheck", LOG_WARNING, "Found unreachable target configuration " + std::to_string(target.a) + ":" + std::to_string(target.s));
    }
     

    std::cout << "Start AStar!" << std::endl;
    auto res = planner->astar(start,target, std::vector<dynamics::data::PBIConstraint>());
    std::cout << "Completed Astar" << std::endl;
    
    if(res.found_path){ 

        std::cout << res.path->size() << std::endl;
        planner->writeCurveToDisk(res, "path1.json");
    }else{
        std::cout << "No viable solution found" << std::endl;
    }

    std::cout << "Start AStar Reversed!" << std::endl;
    res = planner->astar_reversed(start,target);
    std::cout << "Completed Astar Reversed" << std::endl;
    
    if(res.found_path){ 
        std::cout << res.path->size() << std::endl;
        planner->writeCurveToDisk(res, "path2.json");
    }else{
        std::cout << "No viable solution found" << std::endl;
    }
}
