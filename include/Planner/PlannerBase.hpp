#ifndef PLANNER_BASE_HPP
#define PLANNER_BASE_HPP

#include <unordered_set>
#include <unordered_map>
#include <fstream>
#include <queue>
#include <DynamicsModel/SingleTrackModel.hpp>
#include <MPCompute/MPBruteforce.hpp>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct LLJob{
    dynamics::data::PoseByIndex start_positions; 
    dynamics::data::PoseByIndex target_positions;
    std::vector<dynamics::data::PBIConstraint> avoid;
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
    m_keepThreadsAlive = false;
    for(auto& s: m_lowLevelWorkers){
        s.join();
    }
}

std::mutex m_lowLevelSearchJobLock;
std::vector<LLJob> m_lowLevelJobs;
std::vector<std::thread> m_lowLevelWorkers;
std::mutex m_lowLevelSearchResultsLock;
std::vector<LLResult> m_lowLevelResults;
bool m_keepThreadsAlive = true;

MPBruteforce mp_comp;


dynamics::data::PoseByIndex toGlobalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex relative){
    return  (relative + base) - mp_comp.m_mpMapCarOffset;
}

dynamics::data::Pose2D indexToPose(dynamics::data::PoseByIndex global){
    static uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    static float xpc = Config::getInstance().get<float>({"disc","xstep"});
    static float ypc = Config::getInstance().get<float>({"disc","ystep"});
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    static int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    static std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});


    dynamics::data::Pose2D global_pose;
    
    global_pose.pos[0] = (xpc * global.x);
    global_pose.pos[1] = (ypc * global.y);
    global_pose.h = api * static_cast<float>(global.a);
    global_pose.vel = vlevels[global.s] * dynamics::SimpleDynamicsModel::velocity_limit();
    
    return global_pose;
}

dynamics::data::Pose2D indexToPose(dynamics::data::PBIConstraint global){
    static uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    static float xpc = Config::getInstance().get<float>({"disc","xstep"});
    static float ypc = Config::getInstance().get<float>({"disc","ystep"});
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
    static int32_t map_size_speed = Config::getInstance().get<int32_t>({"map","speed_steps"});
    static int32_t map_size_angle = Config::getInstance().get<int32_t>({"map","angle_steps"});
    static int32_t worker_count = Config::getInstance().get<int32_t>({"compute","worker_count"});
    static std::vector<float> vlevels = Config::getInstance().get<std::vector<float>>({"velocity","vlevels"});

    dynamics::data::Pose2D global_pose;
    
    global_pose.pos[0] = (xpc * global.x);
    global_pose.pos[1] = (ypc * global.y);
    global_pose.vel = 0.f;
    global_pose.h = 0;
    
    return global_pose;
}



void writeVisitedNodesToDisk(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target,  std::unordered_map<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex> cameFrom){
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

    std::ofstream o("visited_nodes.json");
    o << visited_nodes << std::endl;
    o.close();

}


dynamics::data::PoseByIndex findNearestPoseByIndex(dynamics::data::Pose2D pose){
    static float xpc = Config::getInstance().get<float>({"disc","xstep"});
    static float ypc = Config::getInstance().get<float>({"disc","ystep"});
    static float api = Config::getInstance().get<float>({"disc","hstep"});
    static int32_t zero_velocity_level = Config::getInstance().get<int32_t>({"velocity","zero_velocity_level"});
   
    float near_x = pose.pos[0] / xpc;
    float near_y = pose.pos[1] / ypc;
    pose.h = fmod(pose.h + 2*PI , 2*PI);
    float near_a = pose.h / api;
    dynamics::data::PoseByIndex result = {(int32_t)round(near_x),(int32_t)round(near_y),(int32_t)std::floor(near_a),zero_velocity_level};
    return result;
}


dynamics::data::PoseByIndex toLocalIndex(dynamics::data::PoseByIndex base, dynamics::data::PoseByIndex global){
    return (global - base) +  mp_comp.m_mpMapCarOffset;
}

bool validatePosition(dynamics::data::PoseByIndex base, dynamics::data::Pose2DWithError edge){
    int32_t x_steps = Config::getInstance().get<int32_t>({"map","x_steps"});
    if(base.x < 0 || base.x > x_steps){
        return false;
    }
    if(base.y < 0 || base.y > x_steps){
        return false;
    }

    return true;
}

LLResult astar(dynamics::data::PoseByIndex start, dynamics::data::PoseByIndex target, std::vector<dynamics::data::PBIConstraint> obstacles){
    
    std::priority_queue<dynamics::data::LLNode> openQueue;
    std::unordered_set<dynamics::data::PoseByIndex> openSet;

    std::unordered_map<dynamics::data::PoseByIndex, dynamics::data::PoseByIndex> cameFrom;
    std::unordered_map<dynamics::data::PoseByIndex, MotionPrimitive> usedEdge;

    for(auto obstacle: obstacles){
        rlog("ASTAR", LOG_INFO, "A*S Obstacle: " + std::to_string(obstacle.x) + ":" + std::to_string(obstacle.y) + ":" + std::to_string(obstacle.t));
    }

    auto current_pose = indexToPose(start);
    auto target_pose = indexToPose(target);

    std::unordered_map<dynamics::data::PoseByIndex, float> fScore;
    fScore[start] = (target_pose.pos - current_pose.pos).norm();
    
    std::unordered_map<dynamics::data::PoseByIndex, float> gScore;
    gScore[start] = 0.f;

    dynamics::data::LLNode initial = {start, fScore[start], 0};
    openQueue.push(initial);
    openSet.insert(start);
    uint32_t explored_nodes = 0; 

    while(!openQueue.empty()){
        
        auto current = openQueue.top(); 
        explored_nodes += 1;

        if(current.pose == target){
            rlog("ASTAR", LOG_INFO, "Found path for " + std::to_string(target.x) + ":" + std::to_string(target.y) + ":" + std::to_string(target.a));
            
            LLResult res;
            res.path = getPath(cameFrom, current.pose);
            res.spline = getSplines(cameFrom, usedEdge, current.pose);
            res.interprimitive = getInterPrimitivPositions(cameFrom, usedEdge, current.pose);

            res.found_path = true;

            return res;
        }

        bool discard = false;
        for(auto obstacle : obstacles){
            //std::abs(current.timestep - obstacle.t) <= 1  &&
            if(  current.pose.x == obstacle.x && current.pose.y == obstacle.y){
                rlog("ASTAR", LOG_INFO, "2. Discarding neighbor due to conflict t: " + std::to_string(current.timestep) + " l: " + std::to_string(current.pose.x) + ":" + std::to_string(current.pose.y));
                discard = true;
                break;
            }
        }
        if(discard){
            continue;
        }


        openQueue.pop();
        for(auto rel_neighbor: mp_comp.m_mpmap[current.pose.a][current.pose.s]){

            dynamics::data::PoseByIndex gl_neighbor = toGlobalIndex(current.pose, rel_neighbor.target);
            auto neigh_pose = indexToPose(gl_neighbor);

            // Invalid position, so we can skip this
            if(!validatePosition(gl_neighbor, rel_neighbor.link)){
                continue;
            }
            
            // TODO POSSIBLY DO A LOOKUP USING UNORDERED MAP or KD Tree to avoid this computation every time
            bool discard_due_to_obstacle = false;
            for(auto obstacle : obstacles){
                
                //std::abs((current.timestep + 1) - obstacle.t) <= 1 &&
                if(  gl_neighbor.x == obstacle.x && gl_neighbor.y == obstacle.y){
                    discard_due_to_obstacle = true;
                    rlog("ASTAR", LOG_INFO, "1. Discarding neighbor due to conflict t: " + std::to_string(current.timestep) + "-" + std::to_string(std::abs(current.timestep - obstacle.t)) + " l: " + std::to_string(gl_neighbor.x) + ":" + std::to_string(gl_neighbor.y));
                    break;
                }

            }

            if(!discard_due_to_obstacle){

                auto current_pose = indexToPose(current.pose); 
                float dist = (neigh_pose.pos - current_pose.pos).norm();
                
                if(current.pose.s != rel_neighbor.target.s){
                    dist *= 1.4f;
                }

                float tentative_score = gScore[current.pose] + dist; 
                if(gScore.count(gl_neighbor) == 0 || tentative_score < gScore[gl_neighbor]){

                    cameFrom[gl_neighbor] = current.pose;
                    usedEdge[gl_neighbor] = rel_neighbor;
                    gScore[gl_neighbor] = tentative_score;
                    
                    float h = (neigh_pose.pos - target_pose.pos).norm();
                    fScore[gl_neighbor] = tentative_score + h;
                    
                    if(openSet.count(gl_neighbor) == 0){
                        dynamics::data::LLNode node = {gl_neighbor, fScore[gl_neighbor], current.timestep + 1};
                        openQueue.push(node);
                        openSet.insert(gl_neighbor);
                    }
                }
            }
        }
    }
    
    writeVisitedNodesToDisk(start, target, cameFrom);
    rlog("ASTAR", LOG_WARNING, "Found no viable path with #open: " + std::to_string(explored_nodes));
    rlog("ASTAR", LOG_WARNING, "Target: " + std::to_string(target.x) + ":" + std::to_string(target.y) + ":" + std::to_string(target.a));
    rlog("ASTAR", LOG_WARNING, "Start: " + std::to_string(start.x) + ":" + std::to_string(start.y) + ":" + std::to_string(start.a));
    
    // writeVisitedNodesToDisk(start,target,cameFrom);
    LLResult res;
    res.found_path = false;
    res.spline.clear();
    
    return res;
}


std::vector<dynamics::data::Pose2WithTime> getSplines(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, std::unordered_map<dynamics::data::PoseByIndex,MotionPrimitive>& edge_map, dynamics::data::PoseByIndex target){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float xpc = Config::getInstance().get<float>({"disc","xstep"});
    float ypc = Config::getInstance().get<float>({"disc","ystep"});
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
    
    uint32_t time_index = 0;
    dynamics::data::Pose2D veh_pose = indexToPose(nodes.at( nodes.size() - 1));

    for(int64_t i = nodes.size() - 1; i >= 0; i --){

        dynamics::data::Pose2WithTime next_with_time;
        next_with_time = indexToPose(nodes.at(i)); 
        next_with_time.time_ms = (time_index * 2 * timestep_ms); 
        result.push_back(next_with_time);

        time_index += 1;
    }
    
    return result;
}

std::vector<dynamics::data::Pose2WithTime> getInterPrimitivPositions(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, std::unordered_map<dynamics::data::PoseByIndex,MotionPrimitive>& edge_map, dynamics::data::PoseByIndex target){
    uint32_t timestep_ms = Config::getInstance().get<uint32_t>({"timestep_ms"});
    float xpc = Config::getInstance().get<float>({"disc","xstep"});
    float ypc = Config::getInstance().get<float>({"disc","ystep"});
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
    uint32_t time_index = 0;
    float half_step =  (timestep_ms / 2);
    
    for(int64_t i = nodes.size() - 1; i > 0; i --){

        dynamics::data::Pose2D veh_pose = indexToPose(nodes.at(i));
        dynamics::data::Pose2D next_pose;

        for(float ts = 0; ts < timestep_ms; ts += 50.f){    
            next_pose = dynamics::SimpleDynamicsModel::computeNextPose(veh_pose, edges.at(i - 1).link.s_a, edges.at(i - 1).link.s_v, ts);
            dynamics::data::Pose2WithTime next_with_time;
            next_with_time.baseNode = nodes.at(i - 1);
            
            next_with_time.time_index = i;
            next_with_time = next_pose;

            next_with_time.time_ms = time_index * 2 * timestep_ms + ts;
            result.push_back(next_with_time);
        }
        
        for(float ts = 0; ts < timestep_ms; ts += 50.f){
            auto next_pose2 = dynamics::SimpleDynamicsModel::computeNextPose(next_pose, edges.at(i - 1).link.s_a_2, edges.at(i - 1).link.s_v_2, ts);
            dynamics::data::Pose2WithTime next_pose2_with_time;
            
            next_pose2_with_time = next_pose2;
            
            next_pose2_with_time.baseNode = nodes.at(i - 1);
            next_pose2_with_time.time_index = i;
            
            next_pose2_with_time.time_ms = time_index * 2 * timestep_ms + timestep_ms + ts;
            result.push_back(next_pose2_with_time);
        }


        time_index += 1;
    }
    
    dynamics::data::Pose2D veh_pose = indexToPose(nodes.at(0));
    dynamics::data::Pose2WithTime next_with_time;
    next_with_time = veh_pose;
    next_with_time.baseNode = nodes.at(0);
    next_with_time.time_index = 0;
    next_with_time.time_ms = time_index * 2 * timestep_ms;
    result.push_back(next_with_time);

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
        
        LLResult res = astar(job.start_positions, job.target_positions, job.avoid);
        res.job_id = job.job_id;
        res.car_id = job.car_id;
        
        m_lowLevelSearchResultsLock.lock();
        m_lowLevelResults.push_back(res);
        m_lowLevelSearchResultsLock.unlock();
    }
}

void enqueue_astar(dynamics::data::PoseByIndex& start, dynamics::data::PoseByIndex& target, constraint_node& constraint, uint32_t& id){
    LLJob job;
    
    job.job_id = id;
    job.avoid.clear();
    job.avoid = constraint.avoid; // this should just copy the constraints with correct ids
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
        if(m_lowLevelResults.size() == count){
            m_lowLevelSearchResultsLock.unlock();
            break;
        }
        m_lowLevelSearchResultsLock.unlock();
       std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> getPath(std::unordered_map<dynamics::data::PoseByIndex,dynamics::data::PoseByIndex>& predecessor, dynamics::data::PoseByIndex& target){
    auto current = target;
    std::shared_ptr<std::vector<dynamics::data::PoseByIndex>> result = std::make_shared<std::vector<dynamics::data::PoseByIndex>>();
    uint32_t index = 0;
    do{

        result->push_back(current);
        current = predecessor[current];
        rlog("GetPath", LOG_INFO, "P: " + std::to_string(current.x) + ":" + std::to_string(current.y) + ":" + std::to_string(current.a) + ":" + std::to_string(current.s) + " index: " + std::to_string(index));

        index ++;
    } while(predecessor.find(current) != predecessor.end());
    
    result->push_back(current);

    return result;
}


void writePathToDisk( std::vector<dynamics::data::PoseByIndex> path, std::string name){
    json astar_path;
    
    for(auto current: path){
        json node;
        node["x"] = current.x;
        node["y"] = current.y;
        node["s"] = current.s;
        node["a"] = current.a;
        astar_path["path"].push_back(node);
    }

    //dump file to disc
    std::ofstream o(name);
    o << astar_path << std::endl;
    o.close();
}

void writeCurveToDisk(std::vector<dynamics::data::Pose2WithTime> path, std::string name){
    json astar_path;
    
    for(auto current: path){
        json node;
        node["x"] = current.pos[0];
        node["y"] = current.pos[1];

        astar_path["path"].push_back(node);
    }
    //dump file to disc
    std::ofstream o(name);
    o << astar_path << std::endl;
    o.close();
}


};
#endif