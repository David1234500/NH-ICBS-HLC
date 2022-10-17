#ifndef DYNAMICS_MODEL_HPP
#define DYNAMICS_MODEL_HPP
 
#include <util/Pose.hpp>
#include <Config.hpp>

namespace dynamics{

using namespace data;

class SimpleDynamicsModel 
{

public:
    SimpleDynamicsModel();

    static Pose2D computeNextPose(Pose2D& current_pose, float& steering_angle, float& velocity, float& time);
    static dynamics::data::Pose2D computeNextPoseWithVelocityInterpolation(dynamics::data::Pose2D& start_pose, float& angle_step_a, float& angle_step_b,
                                                                            float& start_vel, float& target_vel, uint32_t& simulation_velocity_interpolation_count,
                                                                            float& ts_ms);
   
    static dynamics::data::Pose2DWithMotionData computeBestFit(Pose2D& current_pose, Pose2D& target_pose, float& timestep_min_ms, float& timestep_max_ms);
    static std::shared_ptr<std::vector<dynamics::data::Pose2DWithMotionData>> computeReachableSet(Pose2D& current_pose, float& timestep_min_ms, float& timestep_max_ms);
    static float velocity_limit();
    static float angle_limit();

};

}
#endif