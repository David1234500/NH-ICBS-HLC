#ifndef PLANNER_BASE_HPP
#define PLANNER_BASE_HPP

#include <unordered_set>
#include <unordered_map>
#include <fstream>
#include <queue>

#include <nlohmann/json.hpp>
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"

#include <util/Pose.hpp>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <MPCompute/MPBruteforce.hpp>
#include <Collision/CollisionDetectBase.hpp>
#include <util/Profiler.hpp>

using json = nlohmann::json;

struct LLJob{
    dynamics::data::PoseByIndex start_positions; 
    dynamics::data::PoseByIndex target_positions;
    std::vector<dynamics::data::PBIConstraint> avoid;
    bool relaxed_start = false;
    uint32_t job_id = 0;
    uint16_t car_id = 0;  
};


struct LLResult{
    bool found_path = false;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> path;
    std::vector<dynamics::data::Pose2WithTime> spline;
    std::vector<dynamics::data::Pose2WithTime> interprimitive;
    uint32_t job_id = 0;
    uint16_t car_id = 0;
};



struct ReachabilityResult{
    bool reachable = false;
    float mean_path_length = 0.f;
    float mean_time_length = 0.f;
    uint32_t edge_count = 0;
    std::vector<std::pair<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>> unreachable_configurations;
    std::vector<std::pair<int32_t, int32_t>> unleavable_start_configurations;
    std::vector<std::pair<int32_t, int32_t>> unreachable_end_configuartions;
};


struct constraint_node{
    uint64_t sic = 0;
    uint32_t node_id = 0;
    uint32_t father = 0;
    bool feasible = false;

    std::vector<dynamics::data::PBIConstraint> avoid;
    std::map<int32_t, LLResult> result;
   
    bool operator < (const constraint_node r) const {
        if(sic < r.sic){
            return false;
        }else{
            return true;
        }
    }
};

class PlannerBase{
public:
PlannerBase(){
    
}


~PlannerBase(){
   
}

std::mutex m_lowLevelSearchJobLock;
std::vector<LLJob> m_lowLevelJobs;
std::vector<std::thread> m_lowLevelWorkers;
std::mutex m_lowLevelSearchResultsLock;
std::vector<LLResult> m_lowLevelResults;
bool m_keepThreadsAlive = true;

absl::flat_hash_map<dynamics::data::PoseByIndex, dynamics::data::Pose2D> pose_lut;
MPBruteforce mp_comp;


void preparePoseLuT(){
    static uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    static float dpc = Config::getInstance().get<float>({"disc","dstep"});
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    static int32_t x_steps = Config::getInstance().getXstep();
    static int32_t y_steps = Config::getInstance().getYstep();

    static int32_t allowed_waiting = Config::getInstance().get<int32_t>({"allowed_waiting"});
    static std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

    for(int32_t t = 0; t <= allowed_waiting; t++){
        for(int32_t x = 0; x < x_steps; x++){
            for(int32_t y = 0; y < y_steps; y ++){
                for(int32_t a = 0; a < map_size_angle; a ++){
                    for(int32_t s = 0; s < map_size_speed; s ++){
                        
                        dynamics::data::PoseByIndex pbi = {x,y,a,s,t};

                        dynamics::data::Pose2D global_pose;
                        global_pose.pos[0] = (dpc * x);
                        global_pose.pos[1] = (dpc * y);
                        global_pose.h = api * static_cast<float>(a);
                        global_pose.vel = vlevels[s] * dynamics::SimpleDynamicsModel::velocity_limit();
        
                        pose_lut[pbi] = global_pose;
                    }
                }
            }
        }
    }
   
}

inline dynamics::data::PoseByIndex toGlobalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex relative){
    return relative + base;
}

std::vector<dynamics::data::Pose2WithTime> interpolatePathSegment(const dynamics::data::Pose2WithTime& start,
                                                                  const dynamics::data::Pose2WithTime& end,
                                                                  float resolution) {
    std::vector<dynamics::data::Pose2WithTime> interpolated_path;
    float total_distance = (end.pos - start.pos).norm();
    int num_steps = static_cast<int>(std::ceil(total_distance / resolution));

    for (int i = 0; i <= num_steps; ++i) {
        float t = static_cast<float>(i) / num_steps;
        dynamics::data::Pose2WithTime interpolated_pose;

        interpolated_pose.pos = start.pos * (1 - t) + end.pos * t;
        interpolated_pose.h = start.h + t * (end.h - start.h);
        interpolated_pose.vel = start.vel + t * (end.vel - start.vel);
        interpolated_pose.time_ms = start.time_ms + t * (end.time_ms - start.time_ms);
        // interpolated_pose.baseNode = start;
        interpolated_pose.path_depth_index = start.path_depth_index + i;

        interpolated_path.push_back(interpolated_pose);
    }

    return interpolated_path;
}


dynamics::data::Pose2D indexToPose(dynamics::data::PoseByIndex global){
    static uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    static float dpc = Config::getInstance().get<float>({"disc","dstep"});
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    static int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    static std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});


    dynamics::data::Pose2D global_pose;
    
    global_pose.pos[0] = (dpc * global.x);
    global_pose.pos[1] = (dpc * global.y);
    global_pose.h = api * static_cast<float>(global.a);
    global_pose.vel = vlevels[global.s] * dynamics::SimpleDynamicsModel::velocity_limit();
    
    return global_pose;
}

dynamics::data::Pose2D indexToPose(dynamics::data::PBIConstraint global){
    static uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    static float dpc = Config::getInstance().get<float>({"disc","dstep"});
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    static int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    static std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

    dynamics::data::Pose2D global_pose;
    
    global_pose.pos[0] = (dpc * global.x);
    global_pose.pos[1] = (dpc * global.y);
    global_pose.vel = 0.f;
    global_pose.h = 0;
    
    return global_pose;
}



void writeVisitedNodesToDisk(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target,  absl::flat_hash_map<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex> cameFrom, std::string name="visited_nodes.json"){
    json visited_nodes;
    
    visited_nodes["target"]["x"] = target.x;
    visited_nodes["target"]["y"] = target.y;
    visited_nodes["target"]["a"] = target.a;
    visited_nodes["target"]["s"] = target.s;

    visited_nodes["start"]["x"] = start.x;
    visited_nodes["start"]["y"] = start.y;
    visited_nodes["start"]["a"] = start.a;
    visited_nodes["start"]["s"] = start.s;

    for(auto it: cameFrom){

        json node;
        node["x"] = it.second.x;
        node["y"] = it.second.y;
        node["a"] = it.second.a;
        node["s"] = it.second.s;

        node["px"] = it.first.x;
        node["py"] = it.first.y;
        node["pa"] = it.first.a;
        node["ps"] = it.first.s;

        visited_nodes["nodes"].push_back(node);
    }

    std::ofstream o(name);
    o << visited_nodes << std::endl;
    o.close();

}


dynamics::data::PoseByIndex findNearestPoseByIndex(dynamics::data::Pose2D pose){
    static float dpc = Config::getInstance().get<float>({"disc","dstep"});
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
   
    float near_x = pose.pos[0] / dpc;
    float near_y = pose.pos[1] / dpc;
    pose.h = fmod(pose.h + 2*PI , 2*PI);
    float near_a = pose.h / api;
    dynamics::data::PoseByIndex result = {(int32_t)round(near_x),(int32_t)round(near_y),(int32_t)std::floor(near_a),zero_velocity_level};
    return result;
}


dynamics::data::PoseByIndex toLocalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex global){
    return global - base;
}

inline bool validatePosition(dynamics::data::Pose2D& cpose, dynamics::data::PoseByIndex& base, MotionPrimitive& mp, std::vector<dynamics::data::Pose2D>& mp_trajectory){
    static int32_t x_steps = Config::getInstance().getXstep();
    static int32_t y_steps = Config::getInstance().getYstep();

    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});

    assert(base.s < map_size_speed);
    assert(base.a < map_size_angle);

    if(base.x < 1 || base.x > x_steps - 1 ||
       base.y < 1 || base.y > y_steps - 1){
        return false;
    }

    // if(base.x + mp.dist_xn < 1 || base.x + mp.dist_xp > x_steps - 1 ||
    //    base.y + mp.dist_yn < 1 || base.y + mp.dist_yp > y_steps - 1){
    //     return false;
    // }

    
    return true;
}

LLResult astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target, std::vector<dynamics::data::PBIConstraint> constraints, bool relaxed = false){
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t allowed_rev_counter = Config::getInstance().get<int32_t>({"allowed_reversing"});
    static int32_t driving_velocity_level = Config::getInstance().get<int32_t>({"velocity","driving_velocity_level"});
    static int32_t allowed_waiting = Config::getInstance().get<int32_t>({"allowed_waiting"});
    static bool astar_debug = Config::getInstance().get<bool>({"debug_modes", "astar"});
    static bool astar_result = Config::getInstance().get<bool>({"debug_modes", "astar_result"});

    static int32_t x_steps = Config::getInstance().getXstep();
    static int32_t y_steps = Config::getInstance().getYstep();

    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});

    uint64_t astar_compl = CBSProfiler::instance().start(AStarComplete);

    std::priority_queue<dynamics::data::LLNode> openQueue;
    absl::flat_hash_set<dynamics::data::PoseByIndex> openSet;

    absl::flat_hash_map<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex> cameFrom;
    absl::flat_hash_map<dynamics::data::PoseByIndex, MotionPrimitive> usedEdge;

    assert(start.s < map_size_speed);
    assert(target.s < map_size_speed);

    auto current_pose = pose_lut[start];
    auto target_pose = pose_lut[target];

    absl::flat_hash_map<dynamics::data::PoseByIndex, float> fScore;
    float manhattan = std::abs(target_pose.pos[0] - current_pose.pos[0]) + std::abs(target_pose.pos[1] - current_pose.pos[1]);
    fScore[start] = manhattan;
    // fScore[start] = (target_pose.pos - current_pose.pos).norm();
    
    absl::flat_hash_map<dynamics::data::PoseByIndex, float> gScore;
    gScore[start] = 0.f;

    dynamics::data::LLNode initial = {start, fScore[start], 0};
    openQueue.push(initial);
    openSet.insert(start);

    if(relaxed){
        initial.pose.s = driving_velocity_level;
        start.s = driving_velocity_level;
        openQueue.push(initial);
        openSet.insert(start);
    }

    uint32_t explored_nodes = 0; 

    while(!openQueue.empty()){
        
        auto current = openQueue.top(); 
        explored_nodes += 1;

        if(current.pose &= target){
            if(astar_result){
                rlog("ASTAR", LOG_INFO, "Found path for " + std::to_string(target.x) + ":" + std::to_string(target.y) + ":" + std::to_string(target.a));
                writeVisitedNodesToDisk(start, target, cameFrom, "FLLT"+ std::to_string(target.x) + "_" + std::to_string(target.y) + "_" + std::to_string(target.a) + "_" + std::to_string(target.s) +"S"+ std::to_string(start.x) + "_" + std::to_string(start.y) + "_" + std::to_string(start.a)+ "_" + std::to_string(start.s) + ".json");
            }

            LLResult res;
            res.path = getPath(cameFrom, current.pose);
            // res.spline = getSplines(cameFrom, usedEdge, current.pose);
            res.interprimitive = getInterPrimitivPositions(cameFrom, usedEdge, current.pose);
            res.found_path = true;
            CBSProfiler::instance().stop(AStarComplete,astar_compl);

            return res;
        }

        openQueue.pop();
        bool infront_of_constraint = false;
        auto current_pose = pose_lut[current.pose];
        for(auto& rel_neighbor: mp_comp.m_mpmap[current.pose.a][current.pose.s]){
            assert(rel_neighbor.target.s < map_size_speed);
            dynamics::data::PoseByIndex gl_neighbor = toGlobalIndex(current.pose, rel_neighbor.target);
            
            
            
            gl_neighbor.t = current.waiting_counter;

            if(rel_neighbor.target.s < zero_velocity_level){
                if(current.rev_counter >= allowed_rev_counter || current.timestep >= allowed_rev_counter){
                    continue;
                }
            }

            if(!validatePosition(current_pose, gl_neighbor, rel_neighbor, rel_neighbor.trajectory)){
                continue;
            }

            assert(gl_neighbor.s < map_size_speed);
            assert(gl_neighbor.x < x_steps);
            assert(gl_neighbor.y < y_steps);
            assert(gl_neighbor.a < map_size_angle);

            bool discard_due_to_obstacle = false;
            for(auto obstacle : constraints){
               if( std::abs(current.timestep - obstacle.t ) <= 2 || obstacle.t == -1 ){
                    if(std::abs(gl_neighbor.x - obstacle.x ) <= 2 && std::abs(gl_neighbor.y - obstacle.y) <= 2){
                    discard_due_to_obstacle = true;
                    infront_of_constraint = true;
                    break;
                    }
                }
            }


            if(!discard_due_to_obstacle){
                // float dist = (neigh_pose.pos - current_pose.pos).norm();
                auto neigh_pose = pose_lut[gl_neighbor];
                float manhattan = std::abs(neigh_pose.pos[0] - current_pose.pos[0]) + std::abs(neigh_pose.pos[1] - current_pose.pos[1]);

                float tentative_score = gScore[current.pose] + manhattan;
                auto gScoreIt = gScore.find(gl_neighbor); 
                if(gScoreIt == gScore.end() || tentative_score < gScoreIt->second){

                    cameFrom[gl_neighbor] = current.pose;
                    usedEdge[gl_neighbor] = rel_neighbor;
                    gScore[gl_neighbor] = tentative_score;
                    
                    float manhattan_h = std::abs(neigh_pose.pos[0] - target_pose.pos[0]) + std::abs(neigh_pose.pos[1] - target_pose.pos[1]);
                    // float h = (neigh_pose.pos - target_pose.pos).norm();
                    fScore[gl_neighbor] = tentative_score + manhattan_h;
                    
                    if(openSet.find(gl_neighbor) == openSet.end()){
                        //openSet.count(gl_neighbor) == 0){
                        dynamics::data::LLNode node = {gl_neighbor, fScore[gl_neighbor], current.timestep + 1, (rel_neighbor.target.s < zero_velocity_level ? current.rev_counter + 1 : current.rev_counter), current.waiting_counter};
                        openQueue.push(node);
                        openSet.insert(gl_neighbor);
                    }
                }
            }
        }

        auto new_pbi = current.pose;
        new_pbi.t += 1;
        if(openSet.count(new_pbi) == 0 && current.pose.s == zero_velocity_level && current.waiting_counter < allowed_waiting){
            dynamics::data::LLNode node = {new_pbi, fScore[current.pose], current.timestep + 1,  current.rev_counter, current.waiting_counter + 1};
            
            MotionPrimitive mp;
            mp.is_waiting_trajectory = true;
            mp.target = new_pbi;
            
            cameFrom[new_pbi] = current.pose;
            usedEdge[new_pbi] = mp;
            gScore[new_pbi] = gScore[current.pose];
            
            openQueue.push(node);
            openSet.insert(current.pose);
        }

    }
    
    if(astar_result){
        writeVisitedNodesToDisk(start, target, cameFrom, "FLLT"+ std::to_string(target.x) + "_" + std::to_string(target.y) + "_" + std::to_string(target.a) + "_" + std::to_string(target.s) +"S"+ std::to_string(start.x) + "_" + std::to_string(start.y) + "_" + std::to_string(start.a)+ "_" + std::to_string(start.s) + ".json");
        rlog("ASTAR", LOG_WARNING, "Found no viable path with #open: " + std::to_string(explored_nodes));
        rlog("ASTAR", LOG_WARNING, "Target: " + std::to_string(target.x) + ":" + std::to_string(target.y) + ":" + std::to_string(target.a) + ":" + std::to_string(target.s));
        rlog("ASTAR", LOG_WARNING, "Start: " + std::to_string(start.x) + ":" + std::to_string(start.y) + ":" + std::to_string(start.a)+ ":" + std::to_string(start.s));
    }

    CBSProfiler::instance().stop(AStarComplete,astar_compl);
    LLResult res;
    res.found_path = false;
    res.spline.clear();
    
    return res;
}



LLResult astar_reversed(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target, std::vector<dynamics::data::PBIConstraint> constraints = {}){
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t allowed_rev_counter = Config::getInstance().get<int32_t>({"allowed_reversing"});

    static int32_t x_steps = Config::getInstance().getXstep(); 
    static int32_t y_steps = Config::getInstance().getYstep();
    static int32_t dstep = Config::getInstance().get<int32_t>({"disc","dstep"});

    std::priority_queue<dynamics::data::LLNode> openQueue;
    absl::flat_hash_set<dynamics::data::PoseByIndex> openSet;

    absl::flat_hash_map<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex> cameFrom;
    absl::flat_hash_map<dynamics::data::PoseByIndex, MotionPrimitive> usedEdge;

    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});

    auto current_pose = pose_lut[target];
    auto target_pose = pose_lut[start];

    absl::flat_hash_map<dynamics::data::PoseByIndex, float> fScore;
    fScore[target] = (target_pose.pos - current_pose.pos).norm();
    
    absl::flat_hash_map<dynamics::data::PoseByIndex, float> gScore;
    gScore[target] = 0.f;

    dynamics::data::LLNode initial = {target, fScore[target], 0};
    openQueue.push(initial);
    openSet.insert(target);
    uint32_t explored_nodes = 0; 

    dynamics::data::LLNode current;

    while(!openQueue.empty()){
        
        current = openQueue.top(); 
        explored_nodes += 1;

        if(current.pose == start){
            rlog("ASTAR", LOG_INFO, "Found path for " + std::to_string(target.x) + ":" + std::to_string(target.y) + ":" + std::to_string(target.a));
            
            LLResult res;
            res.path = getPath(cameFrom, current.pose);
            res.spline = getSplines(cameFrom, usedEdge, current.pose);
            res.interprimitive = getInterPrimitivPositions(cameFrom, usedEdge, current.pose);
            return res;
        }

        openQueue.pop();
        auto current_pose = pose_lut[current.pose];
         for (int32_t a = 0; a < map_size_angle; a++) {
            for (int32_t s = 0; s < map_size_speed; s++) {
                for (auto rel_neighbor : mp_comp.m_mpmap[a][s]) {

                    // Does this MP end at my current state with respect to heading and speed?
                    if (rel_neighbor.target.a != current.pose.a || rel_neighbor.target.s != current.pose.s) {
                        continue;
                    }

                    //MP always start at the origin and end relative to the origin, so subtract MP from current position to get the start
                    auto gl_neighbor = rel_neighbor.target - current.pose;
                    gl_neighbor.a = a;
                    gl_neighbor.s = s;

                    //Check if start of MP is still inside our drivable envelope
                    if(gl_neighbor.x < 0 || gl_neighbor.x > x_steps || gl_neighbor.y < 0 || gl_neighbor.y > y_steps ){
                       continue;
                    }

                    //Update AStar
                    auto neigh_pose = pose_lut[gl_neighbor];
                    float dist = (neigh_pose.pos - current_pose.pos).norm();

                    float tentative_score = gScore[current.pose] + dist; 
                    if(gScore.count(gl_neighbor) == 0 || tentative_score < gScore[gl_neighbor]){

                        cameFrom[gl_neighbor] = current.pose;
                        usedEdge[gl_neighbor] = rel_neighbor;
                        gScore[gl_neighbor] = tentative_score;
                        
                        float h = (neigh_pose.pos - target_pose.pos).norm();
                        fScore[gl_neighbor] = tentative_score + h;
                        
                        if(openSet.count(gl_neighbor) == 0){
                            dynamics::data::LLNode node = {gl_neighbor, fScore[gl_neighbor], current.timestep + 1, (rel_neighbor.target.s < zero_velocity_level ? current.rev_counter + 1 : current.rev_counter)};
                            openQueue.push(node);
                            openSet.insert(gl_neighbor);
                        }
                    }
                }
            }
        }
    }
    
    writeVisitedNodesToDisk(start, target, cameFrom);
    rlog("ASTAR", LOG_WARNING, "Found no viable path with #open: " + std::to_string(explored_nodes));
    rlog("ASTAR", LOG_WARNING, "Target: " + std::to_string(target.x) + ":" + std::to_string(target.y) + ":" + std::to_string(target.a) + ":" + std::to_string(target.s));
    rlog("ASTAR", LOG_WARNING, "Start: " + std::to_string(start.x) + ":" + std::to_string(start.y) + ":" + std::to_string(start.a)+ ":" + std::to_string(start.s));

    LLResult res;
    res.path = getPath(cameFrom, current.pose);
    res.spline.clear();
    
    return res;
}


void writeCollisionInfoToDisc(const CollisionInfo& collision_info,
                                  const std::vector<dynamics::data::Pose2WithTime>& track1,
                                  const std::vector<dynamics::data::Pose2WithTime>& track2,
                                  const constraint_node& constraint,
                                  const std::string& json_file) {
    // Create a JSON object to store data
    json j;

    // Store CollisionInfo struct data
    j["collision_occurred"] = collision_info.collision_occurred;
    j["track_end"] = collision_info.collision_with_veh_at_track_end;
    j["index1"] = collision_info.index1;
    j["index2"] = collision_info.index2;
    j["feasible1"] = collision_info.one_feasible;
    j["feasible2"] = collision_info.two_feasible;
    j["car_index_1"] = collision_info.car_index_1;
    j["car_index_2"] = collision_info.car_index_2;
    j["car_rem_index_1"] = collision_info.car_rem_1;
    j["car_rem_index_2"] = collision_info.car_rem_2;
    j["pose1"] = {{"pos", {collision_info.pose1.pos[0], collision_info.pose1.pos[1]}},
                  {"h", collision_info.pose1.h},
                  {"vel", collision_info.pose1.vel},
                  {"time_ms", collision_info.pose1.time_ms}};
    j["pose2"] = {{"pos", {collision_info.pose2.pos[0], collision_info.pose2.pos[1]}},
                  {"h", collision_info.pose2.h},
                  {"vel", collision_info.pose2.vel},
                  {"time_ms", collision_info.pose2.time_ms}};
    j["pose1bi"] = {{"x", collision_info.pose1bi.x},
                    {"y", collision_info.pose1bi.y},
                    {"a", collision_info.pose1bi.a},
                    {"s", collision_info.pose1bi.s}};
    j["pose2bi"] = {{"x", collision_info.pose2bi.x},
                    {"y", collision_info.pose2bi.y},
                    {"a", collision_info.pose2bi.a},
                    {"s", collision_info.pose2bi.s}};

    // Store track1 and track2 vectors
    json track1_json = json::array();
    json track2_json = json::array();

    for (const auto& pose : track1) {
        track1_json.push_back({
            {"pos", {pose.pos[0], pose.pos[1]}},
            {"h", pose.h},
            {"vel", pose.vel},
            {"time_ms", pose.time_ms},
            {"baseNode", {{"x", pose.baseNode.x},
                          {"y", pose.baseNode.y},
                          {"a", pose.baseNode.a},
                          {"s", pose.baseNode.s}}},
            {"path_depth_index", pose.path_depth_index}
        });
    }

    for (const auto& pose : track2) {
        track2_json.push_back({
            {"pos", {pose.pos[0], pose.pos[1]}},
            {"h", pose.h},
            {"vel", pose.vel},
            {"time_ms", pose.time_ms},
            {"baseNode", {{"x", pose.baseNode.x},
                          {"y", pose.baseNode.y},
                          {"a", pose.baseNode.a},
                          {"s", pose.baseNode.s}}},
            {"path_depth_index", pose.path_depth_index}
        });
    }

    j["track1"] = track1_json;
    j["track2"] = track2_json;

    // Store constraint_node struct data
    j["constraint_node"] = {
        {"sic", constraint.sic},
        {"node_id", constraint.node_id}
    };

    // Store PBIConstraint data
    json avoid_json = json::array();
    for (const auto& pbi_constraint : constraint.avoid) {
        avoid_json.push_back({
            {"x", pbi_constraint.x},
            {"y", pbi_constraint.y},
            {"t", pbi_constraint.t},
            {"id", pbi_constraint.id}
        });
    }
    j["constraint_node"]["avoid"] = avoid_json;

    // json result_json;
    // for (const auto& [key, value] : constraint.result) {
    //     result_json[std::to_string(key)] = value; // Assuming LLResult is serializable to JSON
    // }
    // j["constraint_node"]["result"] = result_json;

    // Write JSON data to file
    std::ofstream json_out(json_file);
    json_out << j.dump(4); // Pretty print with indent of 4 spaces
    json_out.close();
}



std::vector<dynamics::data::Pose2WithTime> getSplines(absl::flat_hash_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, absl::flat_hash_map<dynamics::data::PoseByIndex,MotionPrimitive>& edge_map, dynamics::data::PoseByIndex target){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

    
    auto current = target;
    std::vector<dynamics::data::PoseByIndex> nodes;
    std::vector<MotionPrimitive> edges;

    do{
        nodes.push_back(current);
        edges.push_back(edge_map[current]);
        current = predecessor[current];
    }while(predecessor.find(current) != predecessor.end());

    nodes.push_back(current);
    edges.push_back(edge_map[current]);

    std::vector<dynamics::data::Pose2WithTime> result;
    
    uint32_t path_depth_index = 0;
    dynamics::data::Pose2D veh_pose = pose_lut[nodes.at( nodes.size() - 1)];

    for(int64_t i = nodes.size() - 1; i >= 0; i --){

        dynamics::data::Pose2WithTime next_with_time;
        next_with_time = pose_lut[nodes.at(i)]; 
        next_with_time.time_ms = (path_depth_index * 2 * timestep_ms); 
        result.push_back(next_with_time);

        path_depth_index += 1;
    }
    
    return result;
}

std::vector<dynamics::data::Pose2WithTime> getInterPrimitivPositions(absl::flat_hash_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, absl::flat_hash_map<dynamics::data::PoseByIndex,MotionPrimitive>& edge_map, dynamics::data::PoseByIndex target){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float dpc = Config::getInstance().get<float>({"disc","dstep"});
    
    float api = Config::getInstance().get<float>({"disc","hstep"});
    int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    static bool astar_debug = Config::getInstance().get<bool>({"debug_modes", "astar"});
    std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

    auto current = target;
    std::vector<dynamics::data::PoseByIndex> nodes;
    std::vector<MotionPrimitive> edges;

    do{
        nodes.push_back(current);
        edges.push_back(edge_map[current]);
        current = predecessor[current];
    }while(predecessor.find(current) != predecessor.end());

    nodes.push_back(current);
    edges.push_back(edge_map[current]);

    std::vector<dynamics::data::Pose2WithTime> result;
    uint32_t path_depth_index = 0;
    float half_step =  (timestep_ms / 2);
    
    for(int64_t i = nodes.size() - 1; i > 0; i --){

        dynamics::data::Pose2D veh_pose = pose_lut[nodes.at(i)];

        dynamics::data::Pose2D next_pose;
        if(astar_debug){
            rlog("GetInterprimitive", LOG_INFO, "P: " + std::to_string(nodes.at(i).x) + ":" + std::to_string(nodes.at(i).y) + ":" + std::to_string(nodes.at(i).a) + ":" + std::to_string(nodes.at(i).s) + " pdi: " + std::to_string(path_depth_index) + " acc:" + std::to_string(edges.at(i - 1).link.is_acc_based));
            rlog("GetInterprimitive", LOG_INFO, "-> MPACC: " + std::to_string(edges.at(i - 1).link.s_a) + ":" + std::to_string(edges.at(i - 1).link.s_acc) + ":" + std::to_string(edges.at(i - 1).link.s_a_2) + ":" + std::to_string(edges.at(i - 1).link.s_acc_2) );
            rlog("GetInterprimitive", LOG_INFO, "-> MPV: " + std::to_string(edges.at(i - 1).link.s_a) + ":" + std::to_string(edges.at(i - 1).link.s_v) + ":" + std::to_string(edges.at(i - 1).link.s_a_2) + ":" + std::to_string(edges.at(i - 1).link.s_v_2) );
        }
        if(edges.at(i - 1).link.is_acc_based){
                for(float ts = 0; ts < timestep_ms; ts += 50.f){    
                next_pose = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(veh_pose, edges.at(i - 1).link.s_a, edges.at(i - 1).link.s_acc, ts);
                dynamics::data::Pose2WithTime next_with_time;
                next_with_time.baseNode = nodes.at(i - 1);
                
                next_with_time.path_depth_index = path_depth_index;
                next_with_time.rem_seg_index = nodes.size() - i;
                next_with_time = next_pose;

                next_with_time.time_ms = path_depth_index * 2 * timestep_ms + ts;
                result.push_back(next_with_time);
            }
            
            for(float ts = 0; ts < timestep_ms; ts += 50.f){
                auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPoseUnderAcceleration(next_pose, edges.at(i - 1).link.s_a_2, edges.at(i - 1).link.s_acc_2, ts);
                dynamics::data::Pose2WithTime next_pose2_with_time;
                
                next_pose2_with_time = next_pose2;
                
                next_pose2_with_time.baseNode = nodes.at(i - 1); //Potentially the -1 is required!
                next_pose2_with_time.path_depth_index = path_depth_index;
                next_pose2_with_time.rem_seg_index = nodes.size() - i;
                
                next_pose2_with_time.time_ms = path_depth_index * 2 * timestep_ms + timestep_ms + ts;
                result.push_back(next_pose2_with_time);
            }
        }else{
            for(float ts = 0; ts < timestep_ms; ts += 50.f){    
                next_pose = dynamics::SimpleDynamicsModel::computeNextPose(veh_pose, edges.at(i - 1).link.s_a, edges.at(i - 1).link.s_v, ts);
                dynamics::data::Pose2WithTime next_with_time;
                next_with_time.baseNode = nodes.at(i - 1);
                
                next_with_time.path_depth_index = path_depth_index;
                next_with_time.rem_seg_index = nodes.size() - i;
                next_with_time = next_pose;

                next_with_time.time_ms = path_depth_index * 2 * timestep_ms + ts;
                result.push_back(next_with_time);
            }
            
            for(float ts = 0; ts < timestep_ms; ts += 50.f){
                auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPose(next_pose, edges.at(i - 1).link.s_a_2, edges.at(i - 1).link.s_v_2, ts);
                dynamics::data::Pose2WithTime next_pose2_with_time;
                
                next_pose2_with_time = next_pose2;
                
                next_pose2_with_time.baseNode = nodes.at(i - 1); //Potentially the -1 is required!
                next_pose2_with_time.path_depth_index = path_depth_index;
                next_pose2_with_time.rem_seg_index = nodes.size() - i;
                
                next_pose2_with_time.time_ms = path_depth_index * 2 * timestep_ms + timestep_ms + ts;
                result.push_back(next_pose2_with_time);
            }
        }
        

        path_depth_index += 1;
    }
    
    // dynamics::data::Pose2D veh_pose = pose_lut[pbi](nodes.at(0));
    // dynamics::data::Pose2WithTime next_with_time;
    // next_with_time = veh_pose;
    // next_with_time.baseNode = nodes.at(0);
    // next_with_time.path_depth_index = 0;
    // next_with_time.time_ms = path_depth_index * 2 * timestep_ms;
    // result.push_back(next_with_time);

    return result;
}


void low_level_astar_worker(uint32_t threadid){
    while(m_keepThreadsAlive){
        
        bool hasJob = false;
        LLJob job;

        m_lowLevelSearchJobLock.lock();
        // rlog("low_level_astar_worker", LOG_WARNING, "Remaining queue size: " + std::to_string(m_lowLevelJobs.size()));
        if(!m_lowLevelJobs.empty()){
            hasJob = true;
            job = m_lowLevelJobs.back();
            m_lowLevelJobs.pop_back();
            m_lowLevelSearchJobLock.unlock();
        }else{
            m_lowLevelSearchJobLock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        LLResult res = astar(job.start_positions, job.target_positions, job.avoid, job.relaxed_start);
        res.job_id = job.job_id;
        res.car_id = job.car_id;
        
        m_lowLevelSearchResultsLock.lock();
        m_lowLevelResults.push_back(res);
        m_lowLevelSearchResultsLock.unlock();
    }
}

void enqueue_astar(dynamics::data::PoseByIndex& start, dynamics::data::PoseByIndex& target, constraint_node& constraint, uint32_t& id, bool relax = false){
    LLJob job;
    
    job.job_id = id;
    
    job.avoid.clear();
    for(auto constr: constraint.avoid){
        if(constr.id == id){
            job.avoid.push_back(constr);
        }
    }
    
    job.relaxed_start = relax;
    job.start_positions = start;
    job.target_positions = target;
    job.car_id = id;

    m_lowLevelSearchJobLock.lock();
    m_lowLevelJobs.push_back(job);
    m_lowLevelSearchJobLock.unlock();
}

void await_astar_result(uint32_t count){
     while(true){
        m_lowLevelSearchResultsLock.lock();
        // std::cout << "target " << std::to_string(count) << " current " << std::to_string(m_lowLevelResults.size()) << std::endl; 
        if(m_lowLevelResults.size() == count){
            m_lowLevelSearchResultsLock.unlock();
            break;
        }
        m_lowLevelSearchResultsLock.unlock();
       std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
}

std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> getPath(absl::flat_hash_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, dynamics::data::PoseByIndex& target){
    auto current = target;
    static bool astar_debug = Config::getInstance().get<bool>({"debug_modes", "astar"});
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> result = std::make_shared<std::vector<dynamics::data::PoseByIndex>>();
    uint32_t index = 0;
    do{

        result->push_back(current);
        current = predecessor[current];
        if(astar_debug){
            rlog("GetPath", LOG_INFO, "P: " + std::to_string(current.x) + ":" + std::to_string(current.y) + ":" + std::to_string(current.a) + ":" + std::to_string(current.s) + " index: " + std::to_string(index));
        }
        index ++;
    } while(predecessor.find(current) != predecessor.end());
    
    result->push_back(current);

    return result;
}


void writeCurveToDisk(LLResult res, std::string name){
    
    
    json llres;

//    for(auto current: *res.path){
//         json pnode;
//         pnode["x"] = current.x;
//         pnode["y"] = current.y;
//         pnode["s"] = current.s;
//         pnode["a"] = current.a;
//         llres["path"].push_back(pnode);
//     }

    //secondly by precise positions
    for(auto current: res.interprimitive){
        json pnode;
        pnode["x"] = current.pos[0];
        pnode["y"] = current.pos[1];
        pnode["s"] = current.vel;
        pnode["a"] = current.h;
        pnode["t"] = current.time_ms;
        pnode["ti"] = current.path_depth_index;

        pnode["bnode"]["x"] =  pose_lut[current.baseNode].pos[0];
        pnode["bnode"]["y"] =  pose_lut[current.baseNode].pos[1];
        pnode["bnode"]["a"] =  pose_lut[current.baseNode].vel;
        pnode["bnode"]["s"] =  pose_lut[current.baseNode].h;
        llres["interprimitive"].push_back(pnode);
    }
    //dump file to disc
    std::ofstream o(name);
    o << llres << std::endl;
    o.close();
}


};
#endif