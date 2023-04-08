#ifndef BOUNDING_BOXES_HPP
#define BOUNDING_BOXES_HPP

#include <Collision/CollisionDetectBase.hpp>
#include <Config.hpp>

class BoundingBoxes: public CollisionDetectBase {
public:
    BoundingBoxes() {

    }

    CollisionInfo checkForCollision(const std::vector<dynamics::data::Pose2WithTime>& track1,
                                    const std::vector<dynamics::data::Pose2WithTime>& track2
                                   ) override {
        CollisionInfo collision_info;

        auto& config = Config::getInstance();
        float over_threshold = config.get<float>({"collision_detect", "ov_appr_threshold"});
        float close_threshold = config.get<float>({"collision_detect", "cl_appr_threshold"});
        uint32_t step_overapproximation = config.get<uint32_t>({"collision_detect", "ov_appr_step"});

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

                    if (bounding_boxes_overlap(precise_pose1, precise_pose2, close_threshold)) {
                        collision_info.collision_occurred = true;
                        collision_info.index1 = precise_pose1.path_depth_index;
                        collision_info.index2 = precise_pose2.path_depth_index;

                        collision_info.pose1 = precise_pose1;
                        collision_info.pose2 = precise_pose2;

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

        return collision_info;
    }

private:
    bool bounding_boxes_overlap(const dynamics::data::Pose2WithTime& pose1, const dynamics::data::Pose2WithTime& pose2, float half_threshold) {
        // Define the bounding boxes for pose1 and pose2
        Eigen::Vector2f(half_threshold, half_threshold);
        auto box1_max = pose1.pos + Eigen::Vector2f(half_threshold, half_threshold);
        auto box2_min = pose2.pos - Eigen::Vector2f(half_threshold, half_threshold);
        auto box2_max = pose2.pos + Eigen::Vector2f(half_threshold, half_threshold);
        
        
        bool overlap_x = (box1_min.x() <= box2_max.x()) && (box1_max.x() >= box2_min.x());
        bool overlap_y = (box1_min.y() <= box2_max.y()) && (box1_max.y() >= box2_min.y());

        return overlap_x && overlap_y;
    }
