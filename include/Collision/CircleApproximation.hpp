#ifndef CIRCLE_APPROXIMATION_HPP
#define CIRCLE_APPROXIMATION_HPP

#include <Collision/CollisionDetectBase.hpp>
#include <Config.hpp>

class CircleApproximation: public CollisionDetectBase{
    public:
    CircleApproximation(){

    }

    struct CircleApproxDebug {
        std::vector<std::tuple<size_t, dynamics::data::Pose2WithTime, dynamics::data::Pose2WithTime, float>> coarse_checks;
        std::vector<std::tuple<size_t, dynamics::data::Pose2WithTime, dynamics::data::Pose2WithTime, float>> precise_checks;
    };

    void saveDebugInfoToJson(const CircleApproxDebug& debug_info, const CollisionInfo& collision_info,
                         const std::vector<dynamics::data::Pose2WithTime>& track1,
                         const std::vector<dynamics::data::Pose2WithTime>& track2) {
        json j;
        j["collision_occurred"] = collision_info.collision_occurred;
        j["index1"] = collision_info.index1;
        j["index2"] = collision_info.index2;

        auto save_pose = [](const dynamics::data::Pose2WithTime& pose) {
            json p;
            p["x"] = pose.pos.x();
            p["y"] = pose.pos.y();
            p["a"] = pose.h;
            p["t"] = pose.time_ms;
            p["ti"] = pose.path_depth_index;
            return p;
        };

        j["coarse_checks"] = json::array();
        for (const auto& [i, pose1, pose2, distance] : debug_info.coarse_checks) {
            json c;
            c["idx"] = i;
            c["pose1"] = save_pose(pose1);
            c["pose2"] = save_pose(pose2);
            c["distance"] = distance;
            j["coarse_checks"].push_back(c);
        }

        j["precise_checks"] = json::array();
        for (const auto& [i, pose1, pose2, distance] : debug_info.precise_checks) {
            json c;
            c["idx"] = i;
                c["pose1"] = save_pose(pose1);
            c["pose2"] = save_pose(pose2);
            c["distance"] = distance;
            j["precise_checks"].push_back(c);
        }

        // Generate a unique file name based on car indexes and times
        size_t car1_index = track1.front().path_depth_index;
        size_t car2_index = track2.front().path_depth_index;
        uint64_t time1 = track1.front().time_ms;
        uint64_t time2 = track2.front().time_ms;

        std::stringstream file_name;
        file_name << "debug_collision_" << car1_index << "_" << car2_index << "_" << time1 << "_" << time2 << ".json";

        // Save JSON to file
        std::ofstream o(file_name.str());
        o << j << std::endl;
        o.close();
    }


   

    CollisionInfo checkForCollision(const std::vector<dynamics::data::Pose2WithTime>& track1,
                                    const std::vector<dynamics::data::Pose2WithTime>& track2
                                   ) override {
        CollisionInfo collision_info;
        CircleApproxDebug debug_info;

        static auto& config = Config::getInstance();
        static float over_threshold = config.get<float>({"collision_detect","ov_appr_threshold"});
        static float close_threshold = config.get<float>({"collision_detect","cl_appr_threshold"}); 
        static uint32_t step_overapproximation = config.get<uint32_t>({"collision_detect","ov_appr_step"}); 

        static bool collision_to_disc_prec = config.get<float>({"debug_modes","collision_to_disc_prec"}); 

        size_t track1_size = track1.size();
        size_t track2_size = track2.size();
        size_t max_track_size = std::max(track1_size, track2_size);

        // First step: coarse collision check
        for (size_t i = 0; i < max_track_size; i += step_overapproximation) {
            dynamics::data::Pose2WithTime pose1 = (i < track1_size) ? track1[i] : track1.back();
            dynamics::data::Pose2WithTime pose2 = (i < track2_size) ? track2[i] : track2.back();

            float distance = (pose1.pos - pose2.pos).norm();
            
            if (collision_to_disc_prec) {
                debug_info.coarse_checks.emplace_back(i, pose1, pose2, distance);
            }

            if (distance <= over_threshold) {
                // Second step: precise collision check around detected possible collision
                for (int j = -step_overapproximation; j <= static_cast<int>(step_overapproximation); ++j) {
                    size_t idx = static_cast<size_t>(std::max(0, static_cast<int>(i) + j));

                    dynamics::data::Pose2WithTime precise_pose1 = (idx < track1_size) ? track1[idx] : track1.back();
                    dynamics::data::Pose2WithTime precise_pose2 = (idx < track2_size) ? track2[idx] : track2.back();

                    float precise_distance = (precise_pose1.pos - precise_pose2.pos).norm();

                    if (precise_distance <= close_threshold) {
                        collision_info.collision_occurred = true;
                        collision_info.index1 = precise_pose1.path_depth_index;
                        collision_info.index2 = precise_pose2.path_depth_index;
                        
                        collision_info.pose1 = precise_pose1;
                        collision_info.pose2 = precise_pose2;

                        if (collision_to_disc_prec) {
                            debug_info.precise_checks.emplace_back(idx, precise_pose1, precise_pose2, precise_distance);
                        }

                        if(idx >= track1_size){
                            collision_info.collision_with_veh_at_track_end = 1;
                        }
                        if(idx >= track2_size){
                            collision_info.collision_with_veh_at_track_end = 2;
                        }

                        collision_info.car_rem_1 = precise_pose1.rem_seg_index;
                        collision_info.car_rem_2 = precise_pose2.rem_seg_index;
                        
                        collision_info.pose1bi = precise_pose1.baseNode;
                        collision_info.pose2bi = precise_pose2.baseNode;

                        return collision_info;
                    }
                }
            }
        }

        if (collision_to_disc_prec) {
            saveDebugInfoToJson(debug_info, collision_info, track1, track2);
        }

        return collision_info;

    }


   static CollisionInfo checkForCollision( MotionPrimitive p1,  dynamics::data::Pose2D pbi1,
                                           MotionPrimitive p2,  dynamics::data::Pose2D pbi2) {
    CollisionInfo collision_info;
    CircleApproxDebug debug_info;

    static auto& config = Config::getInstance();
    static float over_threshold = config.get<float>({"collision_detect","ov_appr_threshold"});
    static float close_threshold = config.get<float>({"collision_detect","cl_appr_threshold"}); 
    static uint32_t step_overapproximation = config.get<uint32_t>({"collision_detect","ov_appr_step"}); 

    size_t track1_size = p1.trajectory.size();
    size_t track2_size = p2.trajectory.size();
    size_t max_track_size = std::max(track1_size, track2_size);

    // First step: coarse collision check
    for (size_t i = 0; i < max_track_size; i += 1) {
        dynamics::data::Vector2Df pose1 = (i < track1_size) ? p1.trajectory.at(i).pos + pbi1.pos : pbi1.pos + p1.trajectory.back().pos;
        dynamics::data::Vector2Df pose2 = (i < track2_size) ? pbi2.pos + p2.trajectory.at(i).pos : pbi2.pos + p2.trajectory.back().pos;

        float distance = (pose1 - pose2).norm();
        if (distance <= close_threshold) {
            collision_info.collision_occurred = true;
            collision_info.pose1.pos = pose1;
            collision_info.pose2.pos = pose2;
            return collision_info;
        }
            
    }
    return collision_info;
    }

    
};


#endif