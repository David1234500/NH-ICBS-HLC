#include <Config.hpp>

 int map_size_x = 50;
 int map_size_y = 50;
 int map_size_angle = 8;
 int map_size_speed = 3;

 float timestep_ms = 500.f;

 int map_size_x_cm = 500;
 int map_size_y_cm = 500;

 float xpc = (float)map_size_x_cm / (float)map_size_x;
 float ypc = (float)map_size_y_cm / (float)map_size_y;
 float api = (2.f * PI) / (float)map_size_angle;

 float safe_radius = 20.f;
 float safe_level2_rad = 10.f;
 float heuristic_factor_backwards = 3.f;

 int zero_velocity_level = 1;
 
 float m_speedsFactor[3] = {-0.1f, 0.f, 0.2f}; 
 bool m_speedFactorIntermediate[3] = {false, true, false}; 

/* Constraints what the allow angle range is for the edge */
 float state_change_fit_quality_angle = api / 2;
 float state_change_fit_quality_position =  xpc / 6;
 float state_change_fit_allowed_speed_difference = 0.15f;


 int worker_counter = 1;

 void recompute_inferred_values(){
    xpc = (float)map_size_x_cm / (float)map_size_x;
    ypc = (float)map_size_y_cm / (float)map_size_y;
    api = (2.f * PI) / (float)map_size_angle;   

    state_change_fit_quality_angle = api / 2;
    state_change_fit_quality_position =  xpc / 2;
    state_change_fit_allowed_speed_difference = 0.15f; 
 }