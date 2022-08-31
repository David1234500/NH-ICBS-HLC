#include <DynamicsModel/SingleTrackModel.hpp>

#include <math.h> 

using namespace dynamics;
using namespace dynamics::data;

Pose2D SimpleDynamicsModel::computeNextPose(Pose2D current_pose, float steering_angle, float velocity, float time){

// Compute driven distance 
float time_sec = time / 1000; 
float dist = velocity * time_sec;

//Case of no steering angle
if(steering_angle == 0.f){
Vector2Df dp = {dist , 0.f};
Eigen::Rotation2Df rotation(current_pose.h);
auto dph = rotation * dp;
auto new_pos = current_pose.pos + dph;
return Pose2D{new_pos, current_pose.h};
}

// Compute radius of driven curve
float radius = 15 / std::sin(steering_angle);  /* TODO: Correct Wheelbase here! */

// Compute circumference of driven circle
float circ = 2 * PI * radius;

// Compute drive angle inside circle
float circ_comp = dist / circ * 2 * PI;

// Compute new x_diff offset
float dx = std::sin(circ_comp) * radius;
float dpy = std::cos(circ_comp) * radius;
float dy = radius - dpy;

// Position offset assuming car points perfectly along x axis
Vector2Df dp = {dx,dy};
Eigen::Rotation2Df rotation(current_pose.h);

// Compute new valid position
auto dph = rotation * dp;
auto new_pos = current_pose.pos + dph;
auto new_head = current_pose.h + circ_comp;

return Pose2D{new_pos, new_head};
}

bool SimpleDynamicsModel::computeBestFit(Pose2D current_pose, Pose2D target_pose, float& steering_angle, float& velocity, float timestep, float allowable_error){
    float error = 1.f;

    float current_angle = 0.f;
    float current_velocity = 0.f;
    float current_angle_span = SimpleDynamicsModel::angle_limit() / 4;
    float current_velocity_span = SimpleDynamicsModel::velocity_limit() / 4;
    float current_error = 1000.f;

    
    // Stage 1 
    uint32_t angle_count = 30;
    uint32_t vel_count = 5;
    for(uint32_t i = 1; i < angle_count; i ++){
        float next_angle = -SimpleDynamicsModel::angle_limit() + SimpleDynamicsModel::angle_limit() * 2 * (i/(float)angle_count);
        for(uint32_t j = 1; j < vel_count; j ++){
            
            float next_speed = SimpleDynamicsModel::velocity_limit() * 0.5f;
            auto next_pose = SimpleDynamicsModel::computeNextPose(current_pose, next_angle, next_speed, timestep);
            float pose_error = (next_pose.pos - target_pose.pos).norm() +  std::abs(next_pose.h - target_pose.h);
            
            if(pose_error < current_error){
                current_error = pose_error;
                current_angle = next_angle;
                current_velocity = next_speed;
            }
        }
    }

    // Stage 2 
    for(uint32_t it = 0; it < 5 && current_error > allowable_error; it ++){
        auto result_pose_upper = SimpleDynamicsModel::computeNextPose(current_pose, current_angle + current_angle_span, current_velocity, timestep);
        float upper_error = (result_pose_upper.pos - target_pose.pos).norm() +  std::abs(result_pose_upper.h - target_pose.h);
        auto result_pose_lower = SimpleDynamicsModel::computeNextPose(current_pose, current_angle - current_angle_span, current_velocity, timestep);
        float lower_error = (result_pose_lower.pos - target_pose.pos).norm() +  std::abs(result_pose_lower.h - target_pose.h);

        if(upper_error < lower_error){
            current_error = upper_error;
            current_angle = current_angle + current_angle_span;
            current_velocity = current_velocity + current_velocity_span;
            current_angle_span = current_angle_span / 2.f;
            current_velocity_span = current_velocity_span / 2.f;
        }else{
            current_error = lower_error;
            current_angle = current_angle - current_angle_span;
            current_velocity = current_velocity - current_velocity_span;
            current_angle_span = current_angle_span / 2.f;
            current_velocity_span = current_velocity_span / 2.f;
        }
    }

    if(current_error < allowable_error){
         steering_angle = current_angle;
        velocity = current_velocity;
        return true;
    }
    return false;
}   

float SimpleDynamicsModel::velocity_limit(){ 
    return 100.0; // cm/s
}

float SimpleDynamicsModel::angle_limit(){
    return PI / 5;
}