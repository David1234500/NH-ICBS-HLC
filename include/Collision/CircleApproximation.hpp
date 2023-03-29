#ifndef CIRCLE_APPROXIMATION_HPP
#define CIRCLE_APPROXIMATION_HPP

#include <Collision/CollisionDetectBase.hpp>
#include <Config.hpp>

class CircleApproximation: public CollisionDetectBase{
    public:
    CircleApproximation(){

    }

    CollisionInfo checkForCollision(const std::vector<dynamics::data::Pose2WithTime>& track1,
                                    const std::vector<dynamics::data::Pose2WithTime>& track2
                                   ) override {
        CollisionInfo collision_info;

       auto& config = Config::getInstance();
        float over_threshold = config.get<float>({"collision_detect","ov_appr_threshold"});
        float close_threshold = config.get<float>({"collision_detect","cl_appr_threshold"}); 
        uint32_t step_overapproximation = config.get<uint32_t>({"collision_detect","ov_appr_step"}); 

        size_t track1_size = track1.size();
        size_t track2_size = track2.size();
        size_t max_track_size = std::max(track1_size, track2_size);

        // First step: coarse collision check
        for (size_t i = 0; i < max_track_size; i += step_overapproximation) {
            dynamics::data::Pose2WithTime pose1 = (i < track1_size) ? track1[i] : track1.back();
            dynamics::data::Pose2WithTime pose2 = (i < track2_size) ? track2[i] : track2.back();

            float distance = (pose1.pos - pose2.pos).norm();

            if (distance <= over_threshold) {
                // Second step: precise collision check around detected possible collision
                for (int j = -step_overapproximation; j <= static_cast<int>(step_overapproximation); ++j) {
                    size_t idx = static_cast<size_t>(std::max(0, static_cast<int>(i) + j));

                    dynamics::data::Pose2WithTime precise_pose1 = (idx < track1_size) ? track1[idx] : track1.back();
                    dynamics::data::Pose2WithTime precise_pose2 = (idx < track2_size) ? track2[idx] : track2.back();

                    float precise_distance = (precise_pose1.pos - precise_pose2.pos).norm();

                    if (precise_distance <= close_threshold) {
                        collision_info.collision_occurred = true;
                        collision_info.index1 = idx;
                        collision_info.index2 = idx;
                        collision_info.car_index_1 = precise_pose1.path_depth_index;
                        collision_info.car_index_2 = precise_pose2.path_depth_index;
                        collision_info.pose1 = precise_pose1;
                        collision_info.pose2 = precise_pose2;
                        collision_info.pose1bi = precise_pose1.baseNode;
                        collision_info.pose2bi = precise_pose2.baseNode;

                        return collision_info;
                    }
                }
            }
        }

        return collision_info;

    }

};


#endif