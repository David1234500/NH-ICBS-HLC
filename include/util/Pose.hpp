#ifndef UTIL_POSE_HPP
#define UTIL_POSE_HPP
 
#include <Eigen/Dense>

namespace dynamics{

namespace data{

typedef Eigen::Matrix<float, 2, 1> Position2Df;
typedef Eigen::Matrix<float, 2, 1> Vector2Df;

struct PoseByIndex{
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t a = 0;
    uint32_t s = 0;
};

struct Pose2D{
    /* 2D Position in cm */
    Position2Df pos = {0.f,0.f}; 
    
    /* Heading in radians */
    float h = 0.f; 

    /* Velocity in m/s */
    float vel = 0.f;
};

struct Pose2DWithError{
    /* 2D Position in cm */
    Position2Df pos = {0.f,0.f};
    
    /* Heading in radians and index */
    float h = 0.f; 

    /* Velocity in m/s and node index */
    float vel = 0.f;

    PoseByIndex bi_pose;

    float a_error = 0.f;
    float p_error = 0.f;

    /* settings values for the associated fit */
    float s_a = 0.f;
    float s_v = 0.f;
};

}

}
#endif //UTIL_POSE_HPP