#include <DynamicsModel/SingleTrackModel.hpp>

#include <iostream>
#include <math.h> 
#include <memory>

using namespace dynamics;
using namespace dynamics::data;

Pose2D SimpleDynamicsModel::computeNextPose(Pose2D& current_pose, double& steering_angle, double& velocity, double& time){
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

std::vector<dynamics::data::Pose2WithTime> SimpleDynamicsModel::computePoseSeries(dynamics::data::Pose2D& start_pose, double& angle_step_a, double& angle_step_b,
                                                                                     double& start_vel, double& target_vel, uint32_t& simulation_velocity_interpolation_count,
                                                                                     double& ts_ms, double base_time){
    std::vector<dynamics::data::Pose2WithTime> pose_vector;
    auto vel = 0.5f * (start_vel + target_vel);
    dynamics::data::Pose2D pose_after_step_1;
    dynamics::data::Pose2D pose_after_step_2;

    for(double timeadvance = 0; timeadvance < ts_ms; timeadvance += 50.f){
       
        pose_after_step_1 = SimpleDynamicsModel::computeNextPose(start_pose, angle_step_a, vel, timeadvance);

        dynamics::data::Pose2WithTime timepose;
        timepose = pose_after_step_1;
        timepose.time_ms = timeadvance;

        pose_vector.push_back(timepose);
    }
    for(double timeadvance = 0; timeadvance < ts_ms; timeadvance += 50.f){
        pose_after_step_2 = SimpleDynamicsModel::computeNextPose(pose_after_step_1, angle_step_b, vel, timeadvance);

        dynamics::data::Pose2WithTime timepose;
        timepose = pose_after_step_2;
        timepose.time_ms = ts_ms + timeadvance;

        pose_vector.push_back(timepose);
    }
    
    return pose_vector;
}

std::shared_ptr<std::vector<dynamics::data::Pose2DWithMotionData>> SimpleDynamicsModel::computeReachableSet(Pose2D& current_pose, double& timestep_min_ms, double& timestep_max_ms, std::vector<double> velocities, std::vector<int32_t> vel_index ){
   
    auto reachable_pose_set = std::make_shared<std::vector<dynamics::data::Pose2DWithMotionData>>();

    Pose2D pose = current_pose;

    uint32_t solver_angle_count = 100;
    uint32_t solver_timestep_count = 100;

    double timestep_size_ms = (timestep_max_ms - timestep_min_ms)/ solver_timestep_count;

    for(uint32_t vi = 0; vi < velocities.size(); vi ++){
        double target_vel = velocities.at(vi);
        int32_t vel_index_global_ref = vel_index.at(vi);
        
        for(double ts_ms = timestep_min_ms; ts_ms < timestep_max_ms; ts_ms += timestep_size_ms){

            std::cout << "ts" << ts_ms << "sv" << current_pose.vel << "tv" << target_vel << std::endl;
            
            for(uint32_t i = 1; i < solver_angle_count; i ++){
                double next_angle = -SimpleDynamicsModel::angle_limit() + (2.f * SimpleDynamicsModel::angle_limit() * (static_cast<double>(i) / static_cast<double>(solver_angle_count)));

                for(uint32_t i2 = 1; i2 < solver_angle_count; i2 ++){

                    double next_angle2 = -SimpleDynamicsModel::angle_limit() + (2.f * SimpleDynamicsModel::angle_limit() * (static_cast<double>(i2) / static_cast<double>(solver_angle_count)));
                    
                    auto vel = 0.5f * (current_pose.vel + target_vel);
                    auto result_pose_1 = computeNextPose(current_pose, next_angle, vel, ts_ms);
                    auto result_pose_2 = computeNextPose(result_pose_1, next_angle2, vel, ts_ms);

                    dynamics::data::Pose2DWithMotionData reachable_pose;
                    
                    reachable_pose = result_pose_2;
                    reachable_pose.ts_ms = ts_ms;

                    reachable_pose.start_vel = current_pose.vel;
                    reachable_pose.target_vel = target_vel;
                    reachable_pose.target_vel_index = vel_index_global_ref;

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
    return 350.0; // cm/s
}

double SimpleDynamicsModel::angle_limit(){
    return PI / 7; // a
}

double SimpleDynamicsModel::acceleration_limit(){
    return 0.7f; // 0.7 m/s2
}