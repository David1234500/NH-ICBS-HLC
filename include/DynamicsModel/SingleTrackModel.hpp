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

    static Pose2D computeNextPose(Pose2D& current_pose, float steering_angle, float velocity, float time);
    static dynamics::data::Pose2DWithError forceBestFit(Pose2D current_pose, PoseByIndex tpi, Pose2D target_pose, float timestep, float allowed_speed_deviation);
    static dynamics::data::Pose2DWithError forceBestFitSingleStep(Pose2D current_pose, PoseByIndex tpi, Pose2D target_pose, float timestep);

    static std::vector<dynamics::data::Vector2Df> vector_limits(float h, float vm);
    static float velocity_limit();
    static float angle_limit();

};

}
#endif