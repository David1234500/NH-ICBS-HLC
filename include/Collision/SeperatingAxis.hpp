#ifndef SEPERATING_AXIS_HPP
#define SEPERATING_AXIS_HPP

#include <Collision/CollisionDetectBase.hpp>
#include <Config.hpp>

class SeperatingAxis: public CollisionDetectBase {
public:
    SeperatingAxis() {

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
    float project_point_on_axis(const Eigen::Vector2f& point, const Eigen::Vector2f& axis) {
        return point.dot(axis) / axis.squaredNorm();
    }

    // Helper function to check if two ranges overlap
    bool ranges_overlap(float min_a, float max_a, float min_b, float max_b) {
        return (min_a <= max_b) && (max_a >= min_b);
    }


    bool bounding_boxes_overlap(const dynamics::data::Pose2WithTime& pose1, const dynamics::data::Pose2WithTime& pose2, float half_length, float half_width) {
        // Define the corners of the bounding boxes for pose1 and pose2
        Eigen::Vector2f axes[2] = {
            {std::cos(pose1.h), std::sin(pose1.h)},
            {-std::sin(pose1.h), std::cos(pose1.h)}
        };
        
        Eigen::Vector2f box1[4] = {
            pose1.pos + axes[0] * half_length + axes[1] * half_width,
            pose1.pos - axes[0] * half_length + axes[1] * half_width,
            pose1.pos - axes[0] * half_length - axes[1] * half_width,
            pose1.pos + axes[0] * half_length - axes[1] * half_width
        };

        axes[0] = {std::cos(pose2.h), std::sin(pose2.h)};
        axes[1] = {-std::sin(pose2.h), std::cos(pose2.h)};
        
        Eigen::Vector2f box2[4] = {
            pose2.pos + axes[0] * half_length + axes[1] * half_width,
            pose2.pos - axes[0] * half_length + axes[1] * half_width,
            pose2.pos - axes[0] * half_length - axes[1] * half_width,
            pose2.pos + axes[0] * half_length - axes[1] * half_width
        };

        // Project the corners onto the axes and check for overlaps
        for (int i = 0; i < 2; ++i) {
            float min_a = std::numeric_limits<float>::max();
            float max_a = std::numeric_limits<float>::lowest();
            float min_b = std::numeric_limits<float>::max();
            float max_b = std::numeric_limits<float>::lowest();

            for (int j = 0; j < 4; ++j) {
                float projection_a = project_point_on_axis(box1[j], axes[i]);
                float projection_b = project_point_on_axis(box2[j], axes[i]);

                min_a = std::min(min_a, projection_a);
                max_a = std::max(max_a, projection_a);
                min_b = std::min(min_b, projection_b);
                max_b = std::max(max_b, projection_b);
            }

            if (!ranges_overlap(min_a, max_a, min_b, max_b)) {
                return false;
            }
        }

        return true;
    }
};

#endif