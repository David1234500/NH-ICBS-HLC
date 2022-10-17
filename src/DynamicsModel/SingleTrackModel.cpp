#include <DynamicsModel/SingleTrackModel.hpp>

#include <iostream>
#include <math.h> 
#include <memory>

using namespace dynamics;
using namespace dynamics::data;

Pose2D SimpleDynamicsModel::computeNextPose(Pose2D& current_pose, float& steering_angle, float& velocity, float& time){

// Compute driven distance 
float time_sec = time / 1000; //millisec
float dist = velocity * time_sec;

steering_angle = fmod(steering_angle + 2*PI , 2*PI);

//Case of no steering angle
if(steering_angle <= 0.05f){
    Vector2Df dp = {dist , 0.f};
    Eigen::Rotation2Df rotation(current_pose.h);
    auto dph = rotation * dp;
    auto new_pos = current_pose.pos + dph;
    return Pose2D{new_pos, current_pose.h, velocity};
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

return Pose2D{new_pos, new_head, velocity};
}

dynamics::data::Pose2D SimpleDynamicsModel::computeNextPoseWithVelocityInterpolation(dynamics::data::Pose2D& start_pose, float& angle_step_a, float& angle_step_b,
                                                                                     float& start_vel, float& target_vel, uint32_t& simulation_velocity_interpolation_count,
                                                                                     float& ts_ms){

    float itp_velocity_step_cms = std::abs(start_vel - target_vel) / static_cast<float>(simulation_velocity_interpolation_count);
    float itp_velocity_base_cms = std::min(start_vel, target_vel);
    dynamics::data::Pose2D pose = start_pose;

    for(uint32_t svic = 1; svic < simulation_velocity_interpolation_count; svic ++){
        float velocity = itp_velocity_base_cms + (itp_velocity_step_cms * svic);
        pose = SimpleDynamicsModel::computeNextPose(pose, angle_step_a, velocity, ts_ms);

    }
    for(uint32_t svic = 1; svic < simulation_velocity_interpolation_count; svic ++){
        float velocity = itp_velocity_base_cms + (itp_velocity_step_cms * svic);
        pose = SimpleDynamicsModel::computeNextPose(pose, angle_step_b, velocity, ts_ms);

    }

    return pose;
}

dynamics::data::Pose2DWithMotionData SimpleDynamicsModel::computeBestFit(Pose2D& current_pose, Pose2D& target_pose, float& timestep_min_ms, float& timestep_max_ms){
   
    dynamics::data::Pose2DWithMotionData current_best_pose;

    Pose2D pose = current_pose;
    float current_angle_span = SimpleDynamicsModel::angle_limit();
    float current_error = 10000.f;

    uint32_t solver_angle_count = 100;
    uint32_t solver_vel_count = 100;
    uint32_t simulation_velocity_interpolation_count = 4;
    uint32_t solver_timestep_count = 20;

    float timestep_size_ms = (timestep_max_ms - timestep_min_ms)/ solver_timestep_count;
    

    for(float ts_ms = timestep_min_ms; ts_ms < timestep_max_ms; ts_ms += timestep_size_ms){

        for(uint32_t i = 0; i < solver_angle_count; i ++){
            float next_angle = (-0.5f * current_angle_span) + current_angle_span * ((float)i / (float)solver_angle_count);

            for(uint32_t i2 = 0; i2 < solver_angle_count; i2 ++){
                float next_angle2 = (-0.5f * current_angle_span) + current_angle_span * ((float)i2 / (float)solver_angle_count);
                
                auto result_pose = computeNextPoseWithVelocityInterpolation(current_pose, next_angle, next_angle2, current_pose.vel, target_pose.vel, simulation_velocity_interpolation_count, ts_ms);

                // Compute error from target position 
                float position_pose_error = (result_pose.pos - target_pose.pos).norm(); 
                float angle_pose_error =  std::abs(result_pose.h - target_pose.h);
                float combined_pose_error = position_pose_error + angle_pose_error;

                if(position_pose_error < current_error){

                    current_best_pose = result_pose;

                    current_best_pose.a_error = angle_pose_error;
                    current_best_pose.p_error = position_pose_error;
                
                    current_best_pose.s_a = next_angle;
                    current_best_pose.s_a_2 = next_angle2;

                    current_best_pose.ts_ms = ts_ms;
                }
            }
        }
    }
    return current_best_pose;
}

std::shared_ptr<std::vector<dynamics::data::Pose2DWithMotionData>> SimpleDynamicsModel::computeReachableSet(Pose2D& current_pose, float& timestep_min_ms, float& timestep_max_ms){
   
    auto reachable_pose_set = std::make_shared<std::vector<dynamics::data::Pose2DWithMotionData>>();

    Pose2D pose = current_pose;
    float current_angle_span = SimpleDynamicsModel::angle_limit();
    float current_error = 10000.f;

    uint32_t solver_angle_count = 100;
    uint32_t solver_vel_count = 100;
    uint32_t simulation_velocity_interpolation_count = 4;
    uint32_t solver_timestep_count = 20;

    float timestep_size_ms = (timestep_max_ms - timestep_min_ms)/ solver_timestep_count;
    

    for(float ts_ms = timestep_min_ms; ts_ms < timestep_max_ms; ts_ms += timestep_size_ms){

        for(uint32_t i = 0; i < solver_angle_count; i ++){
            float next_angle = (-0.5f * current_angle_span) + current_angle_span * ((float)i / (float)solver_angle_count);

            for(uint32_t i2 = 0; i2 < solver_angle_count; i2 ++){
                float next_angle2 = (-0.5f * current_angle_span) + current_angle_span * ((float)i2 / (float)solver_angle_count);
                
                auto result_pose = computeNextPoseWithVelocityInterpolation(current_pose, next_angle, next_angle2, current_pose.vel, target_pose.vel, simulation_velocity_interpolation_count, ts_ms);

                dynamics::data::Pose2DWithMotionData reachable_pose;
                reachable_pose = result_pose;
                reachable_pose.ts_ms = ts_ms;
                reachable_pose.s_a = next_angle;
                reachable_pose.s_a_2 = next_angle2;

                reachable_pose_set->push_back(reachable_pose);
            }
        }
    }
    return reachable_pose_set;
}

 

float SimpleDynamicsModel::velocity_limit(){ 
    return 150.0; // cm/s
}

float SimpleDynamicsModel::angle_limit(){
    return PI / 4;
}