#include <DynamicsModel/SingleTrackModel.hpp>

#include <iostream>
#include <math.h> 
#include <memory>

using namespace dynamics;
using namespace dynamics::data;

Pose2D SimpleDynamicsModel::computeNextPose(Pose2D& current_pose, double& steering_angle, double& velocity, double& time){

// Compute driven distance 
double time_sec = time / 1000.f; //millisec
double dist = velocity * time_sec;

// steering_angle = fmod(steering_angle + 2.f * PI , 2.f * PI);

//Case of no steering angle
if(std::fabs(steering_angle) <= 0.05f){
    Vector2Df dp = {dist , 0.f};
    Eigen::Rotation2Df rotation(current_pose.h);
    auto dph = rotation * dp;
    auto new_pos = current_pose.pos + dph;
    return Pose2D{new_pos, current_pose.h, velocity};
}

// Compute radius of driven curve
double radius = 15.f / std::sin(steering_angle);  /* TODO: Correct Wheelbase here! */

// Compute circumference of driven circle
double circ = 2.f * PI * radius;

// Compute drive angle inside circle
double circ_comp = dist / circ * 2.f * PI;

// Compute new x_diff offset
double dx = std::sin(circ_comp) * radius;
double dpy = std::cos(circ_comp) * radius;
double dy = radius - dpy;

// Position offset assuming car points perfectly along x axis
Vector2Df dp = {dx,dy};
Eigen::Rotation2Df rotation(current_pose.h);

// Compute new valid position
auto dph = rotation * dp;
auto new_pos = current_pose.pos + dph;
auto new_head = current_pose.h + circ_comp * 2.f * PI;

return Pose2D{new_pos, new_head, velocity};
}

dynamics::data::Pose2D SimpleDynamicsModel::computeNextPoseWithVelocityInterpolation(dynamics::data::Pose2D& start_pose, double& angle_step_a, double& angle_step_b,
                                                                                     double& start_vel, double& target_vel, uint32_t& simulation_velocity_interpolation_count,
                                                                                     double& ts_ms){
    
    double itp_velocity_step_cms = std::abs(start_vel - target_vel) / static_cast<double>(simulation_velocity_interpolation_count);
    double itp_velocity_base_cms = std::min(start_vel, target_vel);
    double itp_time_base_ms = ts_ms / static_cast<double>(simulation_velocity_interpolation_count);
    dynamics::data::Pose2D pose = start_pose;

    double itp_vel_step = 0.f;
    for(uint32_t svic = 0; svic <= simulation_velocity_interpolation_count/2; svic ++){
        itp_vel_step = itp_velocity_base_cms + (itp_velocity_step_cms * svic);
        pose = SimpleDynamicsModel::computeNextPose(pose, angle_step_a, itp_vel_step, itp_time_base_ms);
    }
    for(uint32_t svic = 0; svic <= simulation_velocity_interpolation_count/2; svic ++){
        double velocity = itp_vel_step + (itp_velocity_step_cms * svic);
        pose = SimpleDynamicsModel::computeNextPose(pose, angle_step_b, velocity, itp_time_base_ms);
    }
    return pose;
}

std::vector<dynamics::data::Pose2D> SimpleDynamicsModel::computePoseSeries(dynamics::data::Pose2D& start_pose, double& angle_step_a, double& angle_step_b,
                                                                                     double& start_vel, double& target_vel, uint32_t& simulation_velocity_interpolation_count,
                                                                                     double& ts_ms){
    std::vector<dynamics::data::Pose2D> pose_vector;
    double itp_velocity_step_cms = std::abs(start_vel - target_vel) / static_cast<double>(simulation_velocity_interpolation_count);
    double itp_velocity_base_cms = std::min(start_vel, target_vel);
    double itp_time_base_ms = ts_ms / static_cast<double>(simulation_velocity_interpolation_count);
    dynamics::data::Pose2D pose = start_pose;

    double itp_vel_step = 0.f;
    for(uint32_t svic = 0; svic <= simulation_velocity_interpolation_count/2; svic ++){
        itp_vel_step = itp_velocity_base_cms + (itp_velocity_step_cms * svic);
        pose = SimpleDynamicsModel::computeNextPose(pose, angle_step_a, itp_vel_step, itp_time_base_ms);
        pose_vector.push_back(pose);
    }
    for(uint32_t svic = 0; svic <= simulation_velocity_interpolation_count/2; svic ++){
        double velocity = itp_vel_step + (itp_velocity_step_cms * svic);
        pose = SimpleDynamicsModel::computeNextPose(pose, angle_step_b, velocity, itp_time_base_ms);
        pose_vector.push_back(pose);
    }

    return pose_vector;
}


dynamics::data::Pose2DWithMotionData SimpleDynamicsModel::computeBestFit(Pose2D& current_pose, Pose2D& target_pose, double& timestep_min_ms, double& timestep_max_ms){
   
    dynamics::data::Pose2DWithMotionData current_best_pose;

    Pose2D pose = current_pose;
    double current_angle_span = SimpleDynamicsModel::angle_limit();
    double current_error = 10000.f;

    uint32_t solver_angle_count = 100;
    uint32_t solver_vel_count = 100;
    uint32_t simulation_velocity_interpolation_count = 4;
    uint32_t solver_timestep_count = 20;

    double timestep_size_ms = (timestep_max_ms - timestep_min_ms)/ solver_timestep_count;
    

    for(double ts_ms = timestep_min_ms; ts_ms < timestep_max_ms; ts_ms += timestep_size_ms){

        for(uint32_t i = 0; i < solver_angle_count; i ++){
            double next_angle = (-current_angle_span) + 2.f * current_angle_span * ((double)i / (double)solver_angle_count);

            for(uint32_t i2 = 0; i2 < solver_angle_count; i2 ++){
                double next_angle2 = (-current_angle_span) + 2.f *  current_angle_span * ((double)i2 / (double)solver_angle_count);
                
                auto result_pose = computeNextPoseWithVelocityInterpolation(current_pose, next_angle, next_angle2, current_pose.vel, target_pose.vel, simulation_velocity_interpolation_count, ts_ms);

                // Compute error from target position 
                double position_pose_error = (result_pose.pos - target_pose.pos).norm(); 
                double angle_pose_error =  std::abs(result_pose.h - target_pose.h);
                double combined_pose_error = position_pose_error + angle_pose_error;

                if(position_pose_error < current_error){

                    current_best_pose = result_pose;

                    // current_best_pose.a_error = angle_pose_error;
                    // current_best_pose.p_error = position_pose_error;
                
                    current_best_pose.s_a = next_angle;
                    current_best_pose.s_a_2 = next_angle2;

                    current_best_pose.ts_ms = ts_ms;
                }
            }
        }
    }
    return current_best_pose;
}

std::shared_ptr<std::vector<dynamics::data::Pose2DWithMotionData>> SimpleDynamicsModel::computeReachableSet(Pose2D& current_pose, double& timestep_min_ms, double& timestep_max_ms, std::vector<double> velocities){
   
    auto reachable_pose_set = std::make_shared<std::vector<dynamics::data::Pose2DWithMotionData>>();

    Pose2D pose = current_pose;

    uint32_t solver_angle_count = 200;
    uint32_t solver_vel_count = 200;
    uint32_t simulation_velocity_interpolation_count = 8;
    uint32_t solver_timestep_count = 100;

    double timestep_size_ms = (timestep_max_ms - timestep_min_ms)/ solver_timestep_count;
    double current_angle_base =  (-SimpleDynamicsModel::angle_limit());

    for(uint32_t vel_index = 0; vel_index < velocities.size(); vel_index ++){
        double target_vel = velocities.at(vel_index);
        for(double ts_ms = timestep_min_ms; ts_ms < timestep_max_ms; ts_ms += timestep_size_ms){

            for(uint32_t i = 1; i < solver_angle_count; i ++){
                double next_angle = current_angle_base + (2.f * SimpleDynamicsModel::angle_limit() * (static_cast<double>(i) / static_cast<double>(solver_angle_count)));

                for(uint32_t i2 = 1; i2 < solver_angle_count; i2 ++){
                    double next_angle2 = current_angle_base + (2.f * SimpleDynamicsModel::angle_limit() * (static_cast<double>(i2) / static_cast<double>(solver_angle_count)));
                    
                    auto result_pose = computeNextPoseWithVelocityInterpolation(current_pose, next_angle, next_angle2 , current_pose.vel, target_vel, simulation_velocity_interpolation_count, ts_ms);

                    dynamics::data::Pose2DWithMotionData reachable_pose;
                    reachable_pose = result_pose;
                    reachable_pose.ts_ms = ts_ms;

                    reachable_pose.start_vel = current_pose.vel;
                    reachable_pose.target_vel = target_vel;
                    reachable_pose.target_vel_index = vel_index;

                    reachable_pose.s_a = next_angle;
                    reachable_pose.s_a_2 = next_angle2;

                    reachable_pose_set->push_back(reachable_pose);
                }
            }
        }
   }
    std::cout << "reachable_set size: " << reachable_pose_set->size() << std::endl;
    return reachable_pose_set;
}

 

double SimpleDynamicsModel::velocity_limit(){ 
    return 150.0; // cm/s
}

double SimpleDynamicsModel::angle_limit(){
    return PI / 8;
}