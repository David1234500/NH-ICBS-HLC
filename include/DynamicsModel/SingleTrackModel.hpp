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

    static Pose2D computeNextPose(Pose2D current_pose, float steering_angle, float velocity, float time);
    static dynamics::data::Pose2DWithError computeBestFit(Pose2D current_pose, PoseByIndex tpi, Pose2D target_pose, float timestep);
    static dynamics::data::Pose2DWithError computeBestFitSingleStep(Pose2D current_pose, PoseByIndex tpi, Pose2D target_pose, float timestep);

    static float velocity_limit();
    static float angle_limit();

};

}
#endif