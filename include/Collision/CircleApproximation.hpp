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
        float close_threshold = config.get<float>({"collision_detect","ov_appr_threshold"}); 
        uint32_t step_overapproximation = config.get<uint32_t>({"collision_detect","ov_appr_step"}); 

        for (size_t i = 0; i < track1.size(); i += step_overapproximation) {
            for (size_t j = 0; j < track2.size(); j += step_overapproximation) {
                if (tracksTooClose(track1[i], track2[j], over_threshold)) {
                    return checkForCollisionDetailed(track1, track2, close_threshold);
                }
            }
        }
        return collision_info;
    }

private:
    bool tracksTooClose(const dynamics::data::Pose2WithTime& pose1,
                        const dynamics::data::Pose2WithTime& pose2, float threshold) {
        float distance = (pose1.pos - pose2.pos).norm();
        return distance < threshold;
    }

    CollisionInfo checkForCollisionDetailed(const std::vector<dynamics::data::Pose2WithTime>& track1,
                                            const std::vector<dynamics::data::Pose2WithTime>& track2,
                                            float threshold) {
        CollisionInfo collision_info;
        for (size_t i = 0; i < track1.size(); ++i) {
            for (size_t j = 0; j < track2.size(); ++j) {
                if (tracksTooClose(track1[i], track2[j], threshold)) {
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

};


#endif