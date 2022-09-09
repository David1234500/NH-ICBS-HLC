#include <DynamicsModel/SingleTrackModel.hpp>

#include <iostream>
#include <math.h> 

using namespace dynamics;
using namespace dynamics::data;

Pose2D SimpleDynamicsModel::computeNextPose(Pose2D current_pose, float steering_angle, float velocity, float time){

// Compute driven distance 
float time_sec = time / 1000; //millisec
float dist = velocity * time_sec;

//Case of no steering angle
if(steering_angle <= 0.05f){
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

dynamics::data::Pose2DWithError SimpleDynamicsModel::computeBestFit(Pose2D current_pose, PoseByIndex tpi, Pose2D target_pose, float timestep){
   
    dynamics::data::Pose2DWithError current_best_pose;
    current_best_pose.bi_pose = tpi;

    float current_angle_span = SimpleDynamicsModel::angle_limit();
    float current_velocity_span = SimpleDynamicsModel::velocity_limit() / 3; //TODO REMOVE HARDCODING HERE, CALC LIMIT TO ACCELERATION AND CONSEQ LIMIT TO SPEED SPAN
    float current_error = 1000.f;

    // Iterate over all possible combinations of speeds and steering angles to find best configuration for reaching the target node
    uint32_t angle_count = 30;
    uint32_t vel_count = 20; // TODO REMOVE THESE HARDCODED VALUES

    for(uint32_t i = 0; i <= angle_count; i ++){
        float next_angle = (-(1.f/2.f)*SimpleDynamicsModel::angle_limit()) + SimpleDynamicsModel::angle_limit() * ((float)i / (float)angle_count);
        
        for(uint32_t j = 0; j <= vel_count; j ++){    
            float next_speed = current_pose.vel -(1.f/2.f)*current_velocity_span + (current_velocity_span * ((float)j / (float)vel_count)); //TODO AFJUST SPEED VAL HERE

            // Project Vehicle for one timestep with these settings
            auto next_pose_step_1 = SimpleDynamicsModel::computeNextPose(current_pose, next_angle, next_speed, timestep/2);

            for(uint32_t i2 = 0; i2 <= angle_count; i2 ++){
                float next_angle2 = (-(1.f/2.f)*SimpleDynamicsModel::angle_limit()) + SimpleDynamicsModel::angle_limit() * ((float)i2 / (float)angle_count);
                
                for(uint32_t j2 = 0; j2 <= vel_count; j2 ++){    
                    float next_speed2 = current_pose.vel -(1.f/2.f)*current_velocity_span + (current_velocity_span * ((float)j2 / (float)vel_count)); //TODO AFJUST SPEED VAL HERE

                    // Project Vehicle for one timestep with these settings
                    auto next_pose_step_2 = SimpleDynamicsModel::computeNextPose(next_pose_step_1, next_angle2, next_speed2, timestep/2);

                    // Compute error from target position 
                    float position_pose_error = (next_pose_step_2.pos - target_pose.pos).norm() ; //TODO: GUESTIMATE USEFULL ERRORS BOUNDARIES FOR THESE VALUES
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



dynamics::data::Pose2DWithError SimpleDynamicsModel::computeBestFitSingleStep(Pose2D current_pose, PoseByIndex tpi, Pose2D target_pose, float timestep){
    float error = 1.f;

    dynamics::data::Pose2DWithError current_best_pose;
    current_best_pose.bi_pose = tpi;

    float current_angle_span = SimpleDynamicsModel::angle_limit();
    float current_velocity_span = SimpleDynamicsModel::velocity_limit() / 3; //TODO REMOVE HARDCODING HERE, CALC LIMIT TO ACCELERATION AND CONSEQ LIMIT TO SPEED SPAN
    float current_error = 1000.f;

    // Iterate over all possible combinations of speeds and steering angles to find best configuration for reaching the target node
    uint32_t angle_count = 40;
    uint32_t vel_count = 30; // TODO REMOVE THESE HARDCODED VALUES

    for(uint32_t i = 0; i < angle_count; i ++){
        float next_angle = (-(1.f/2.f)*SimpleDynamicsModel::angle_limit()) + SimpleDynamicsModel::angle_limit() * ((float)i / (float)angle_count);
        
        for(uint32_t j = 0; j < vel_count; j ++){    
            float next_speed = current_pose.vel -(1.f/2.f)*current_velocity_span + (current_velocity_span * ((float)j / (float)vel_count)); //TODO AFJUST SPEED VAL HERE

            // Project Vehicle for one timestep with these settings
            auto next_pose_step_1 = SimpleDynamicsModel::computeNextPose(current_pose, next_angle, next_speed, timestep);

            // Compute error from target position 
            float position_pose_error = (next_pose_step_1.pos - target_pose.pos).norm() ; //TODO: GUESTIMATE USEFULL ERRORS BOUNDARIES FOR THESE VALUES
            float angle_pose_error =  std::abs(next_pose_step_1.h - target_pose.h);
            float combined_pose_error = position_pose_error + angle_pose_error;

            if(position_pose_error < current_error){

                current_error = position_pose_error;

                current_best_pose.pos = next_pose_step_1.pos;
                current_best_pose.h = next_pose_step_1.h;
                current_best_pose.vel = next_pose_step_1.vel;

                current_best_pose.a_error = angle_pose_error;
                current_best_pose.p_error = position_pose_error;
            
                current_best_pose.s_a = next_angle;
                current_best_pose.s_v= next_speed;

            }
        }
            
    }
    return current_best_pose;
}   

float SimpleDynamicsModel::velocity_limit(){ 
    return 100.0; // cm/s
}

float SimpleDynamicsModel::angle_limit(){
    return PI / 2;
}