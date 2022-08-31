#ifndef DYNAMICS_MODEL_HPP
#define DYNAMICS_MODEL_HPP
 
#include <util/Pose.hpp>


#define PI 3.14159265

namespace dynamics{

using namespace data;

class SimpleDynamicsModel 
{

public:
    SimpleDynamicsModel();

    static Pose2D computeNextPose(Pose2D current_pose, float steering_angle, float velocity, float time);
    static bool computeBestFit(Pose2D current_pose, Pose2D target_pose, float& steering_angle, float& velocity, float time, float error = 0.05f);

    static float velocity_limit();
    static float angle_limit();

};

}
#endif