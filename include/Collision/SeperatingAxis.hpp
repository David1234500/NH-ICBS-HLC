#ifndef SEPERATING_AXIS_HPP
#define SEPERATING_AXIS_HPP

#include <Collision/CollisionDetectBase.hpp>
#include <Config.hpp>

struct OrientedBoundingBox {
    Eigen::Vector2f center;
    Eigen::Vector2f extents;
    Eigen::Matrix2f rotation;
};

class SeparatingAxisTheorem : public CollisionDetectBase {
public:
    SeparatingAxisTheorem() {}

    CollisionInfo checkForCollision(const std::vector<dynamics::data::Pose2WithTime>& track1,
                                    const std::vector<dynamics::data::Pose2WithTime>& track2) override {
        CollisionInfo collision_info;

        auto& config = Config::getInstance();
        float over_threshold = config.get<float>({"collision_detect","ov_appr_threshold"});
        float close_threshold = config.get<float>({"collision_detect","ov_appr_threshold"}); 
        uint32_t step_overapproximation = config.get<uint32_t>({"collision_detect","ov_appr_step"}); 

        // Iterate over each pair of Pose2WithTime objects from track1 and track2
        for (size_t i = 0; i < track1.size(); ++i) {
            for (size_t j = 0; j < track2.size(); ++j) {
                // Create OBBs for each Pose2WithTime object
                OrientedBoundingBox obb1 = createOBB(track1[i], close_threshold);
                OrientedBoundingBox obb2 = createOBB(track2[j], close_threshold);

                // Check for collision using SAT
                if (obbCollision(obb1, obb2)) {
                    collision_info.collision_occurred = true;
                    collision_info.index1 = i;
                    collision_info.index2 = j;
                    collision_info.pose1 = track1[i];
                    collision_info.pose2 = track2[j];
                    return collision_info;
                }
            }
        }
        return collision_info;
    }

private:
    OrientedBoundingBox createOBB(const dynamics::data::Pose2WithTime& pose, float threshold) {
        OrientedBoundingBox obb;
        obb.center = pose.pos;
        obb.extents = Eigen::Vector2f(threshold / 2, threshold / 2);
        obb.rotation = Eigen::Rotation2D<float>(pose.h).toRotationMatrix();
        return obb;
    }

    bool obbCollision(const OrientedBoundingBox& obb1, const OrientedBoundingBox& obb2) {
        Eigen::Vector2f t = obb2.center - obb1.center;
        Eigen::Matrix2f R = obb1.rotation.transpose() * obb2.rotation;

        for (int i = 0; i < 2; ++i) {
            float ra = obb1.extents[i];
            float rb = obb2.extents[0] * std::abs(R(i, 0)) + obb2.extents[1] * std::abs(R(i, 1));

            if (std::abs(t.dot(obb1.rotation.col(i))) > ra + rb) {
                return false;
            }
        }

        for (int i = 0; i < 2; ++i) {
            float ra = obb1.extents[0] * std::abs(R(0, i)) + obb1.extents[1] * std::abs(R(1, i));
            float rb = obb2.extents[i];

            if (std::abs((obb1.rotation.transpose() * t).dot(obb2.rotation.col(i))) > ra + rb) {
                return false;
            }
        }

        if (std::abs(t.dot(obb1.rotation.col(0)) * R(1, 0) - t.dot(obb1.rotation.col(1)) * R(0, 0)) >
            obb1.extents[1] * std::abs(R(0, 0)) + obb2.extents[1] * std::abs(R(1, 0))) {
            return false;
        }

        if (std::abs(t.dot(obb1.rotation.col(0)) * R(1, 1) - t.dot(obb1.rotation.col(1)) * R(0, 1)) >
            obb1.extents[1] * std::abs(R(0, 1)) + obb2.extents[1] * std::abs(R(1, 1))) {
            return false;
        }

        return true;
    }

};

#endif