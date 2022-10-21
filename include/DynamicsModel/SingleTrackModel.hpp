#ifndef DYNAMICS_MODEL_HPP
#define DYNAMICS_MODEL_HPP
 
#include <util/Pose.hpp>
#include <Config.hpp>
#include <vector>

namespace dynamics{

using namespace data;

class SimpleDynamicsModel 
{

public:
    SimpleDynamicsModel();

    static Pose2D computeNextPose(Pose2D& current_pose, double& steering_angle, double& velocity, double& time);
    static dynamics::data::Pose2D computeNextPoseWithVelocityInterpolation(dynamics::data::Pose2D& start_pose, double& angle_step_a, double& angle_step_b,
                                                                            double& start_vel, double& target_vel, uint32_t& simulation_velocity_interpolation_count,
                                                                            double& ts_ms);
    static std::vector<dynamics::data::Pose2D> computePoseSeries(dynamics::data::Pose2D& start_pose, double& angle_step_a, double& angle_step_b,
                                                                                     double& start_vel, double& target_vel, uint32_t& simulation_velocity_interpolation_count,
                                                                                     double& ts_ms);
    static dynamics::data::Pose2DWithMotionData computeBestFit(Pose2D& current_pose, Pose2D& target_pose, double& timestep_min_ms, double& timestep_max_ms);
    static std::shared_ptr<std::vector<dynamics::data::Pose2DWithMotionData>> computeReachableSet(Pose2D& current_pose, double& timestep_min_ms, double& timestep_max_ms, std::vector<double> velocities);
    static double velocity_limit();
    static double angle_limit();

};

}
#endif