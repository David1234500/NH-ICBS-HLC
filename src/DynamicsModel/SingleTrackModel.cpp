#include <DynamicsModel/SingleTrackModel.hpp>

#include <iostream>
#include <math.h> 

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

// dynamics::data::Pose2D SimpleDynamicsModel::computeNextPoseWithVelocityInterpolation(dynamics::data::Pose2D& start_pose, double& angle_step_a, double& angle_step_b,
//                                                                                      double& start_vel, double& target_vel, uint32_t& simulation_velocity_interpolation_count,
//                                                                                      double& ts_ms){
    
//     double itp_velocity_step_cms = std::abs(start_vel - target_vel) / static_cast<double>(simulation_velocity_interpolation_count);
//     double itp_velocity_base_cms = std::min(start_vel, target_vel);
//     double itp_time_base_ms = ts_ms / static_cast<double>(simulation_velocity_interpolation_count);
//     dynamics::data::Pose2D pose = start_pose;

//     double itp_vel_step = 0.f;
//     for(uint32_t svic = 0; svic <= simulation_velocity_interpolation_count/2; svic ++){
//         itp_vel_step = itp_velocity_base_cms + (itp_velocity_step_cms * svic);
//         pose = SimpleDynamicsModel::computeNextPose(pose, angle_step_a, itp_vel_step, itp_time_base_ms);
//     }
//     for(uint32_t svic = 0; svic <= simulation_velocity_interpolation_count/2; svic ++){
//         double velocity = itp_vel_step + (itp_velocity_step_cms * svic);
//         pose = SimpleDynamicsModel::computeNextPose(pose, angle_step_b, velocity, itp_time_base_ms);
//     }
//     return pose;
// }

dynamics::data::Pose2DWithError SimpleDynamicsModel::computeBestFit(Pose2D current_pose, PoseByIndex tpi, Pose2D target_pose, float timestep, float allowed_speed_deviation){
   
    dynamics::data::Pose2DWithError current_best_pose;
    current_best_pose.bi_pose = tpi;

    float current_angle_span = SimpleDynamicsModel::angle_limit();
    float center_average_speed = ((current_pose.vel + target_pose.vel) / 2.f);
    
    if(center_average_speed < 0.f){
        current_angle_span = 0.2f * current_angle_span;
    }
    
    float current_error = 1000.f;

    // Iterate over all possible combinations of speeds and steering angles to find best configuration for reaching the target node
    uint32_t angle_count = 20;
    uint32_t vel_count = 20;

    for(uint32_t i = 0; i <= angle_count; i ++){
        float next_angle = (-(1.f/2.f) * current_angle_span) + current_angle_span * ((float)i / (float)angle_count);
        
        for(uint32_t j = 0; j <= vel_count; j ++){ 

            
            float allowed_deviation_from_target_speed = (-(allowed_speed_deviation/2.f) + (allowed_speed_deviation/ static_cast<float>(vel_count)) * static_cast<float>(j)) * SimpleDynamicsModel::velocity_limit();
            float next_speed =  center_average_speed + allowed_deviation_from_target_speed;

            // Project Vehicle for one timestep with these settings
            auto next_pose_step_1 = SimpleDynamicsModel::computeNextPose(current_pose, next_angle, next_speed, timestep);

            for(uint32_t i2 = 0; i2 <= angle_count; i2 ++){
                float next_angle2 = (-(1.f/2.f)*current_angle_span) + current_angle_span * ((float)i2 / (float)angle_count);
                
                for(uint32_t j2 = 0; j2 <= vel_count; j2 ++){    
                    
                    //Compute next speed for second step of planning for the vehicle
                    float allowed_deviation_from_target_speed2 = (-(allowed_speed_deviation/2.f) + (allowed_speed_deviation/static_cast<float>(vel_count)) * static_cast<float>(j2)) * SimpleDynamicsModel::velocity_limit();
                    float next_speed2 =  center_average_speed + allowed_deviation_from_target_speed2;

                    // Project Vehicle for one timestep with these settings
                    auto next_pose_step_2 = SimpleDynamicsModel::computeNextPose(next_pose_step_1, next_angle2, next_speed2, timestep);

                    // Compute error from target position 
                    float position_pose_error = (next_pose_step_2.pos - target_pose.pos).norm(); 
                    float angle_pose_error =  std::abs(next_pose_step_2.h - target_pose.h);
                    float combined_pose_error = position_pose_error + angle_pose_error;

                    if(position_pose_error < current_error){

                        current_error = position_pose_error;

                        current_best_pose.pos = next_pose_step_2.pos;
                        current_best_pose.h = next_pose_step_2.h;
                        current_best_pose.vel = next_pose_step_2.vel;

                        current_best_pose.a_error = angle_pose_error;
                        current_best_pose.p_error = position_pose_error;
                    
                        current_best_pose.s_a = next_angle;
                        current_best_pose.s_v = next_speed;

                        current_best_pose.s_a_2 = next_angle2;
                        current_best_pose.s_v_2 = next_speed2;

                    }
                }
            }
        }
            
    }
    return current_best_pose;
}



 

float SimpleDynamicsModel::velocity_limit(){ 
    return 200.0; // cm/s
}

float SimpleDynamicsModel::angle_limit(){
    return PI / 8; 
}