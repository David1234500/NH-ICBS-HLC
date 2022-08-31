#ifndef UTIL_POSE_HPP
#define UTIL_POSE_HPP
 
#include <Eigen/Dense>

namespace dynamics{

namespace data{

typedef Eigen::Matrix<float, 2, 1> Position2Df;
typedef Eigen::Matrix<float, 2, 1> Vector2Df;

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
    
    /* Heading in radians */
    float h = 0.f; 

    /* Velocity in m/s */
    float vel = 0.f;

    float error = 0.f;
};

}

}
#endif //UTIL_POSE_HPP