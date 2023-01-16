#include <Config.hpp>

 int map_size_x = 80;
 int map_size_y = 80;
 int map_size_angle = 12;
 int map_size_speed = 3;

 float timestep_ms = 500.f;

 int map_size_x_cm = 400;
 int map_size_y_cm = 450;

 float xpc = (float)map_size_x_cm / (float)map_size_x;
 float ypc = (float)map_size_y_cm / (float)map_size_y;
 float api = (2.f * PI) / (float)map_size_angle;

 float safe_radius = 70.f;
 float safe_level2_rad = 15.f;
 float heuristic_factor_backwards = 3.f;

 int zero_velocity_level = 1;
 
 float m_speedsFactor[3] = {-0.4f, 0.f, 0.4f}; 
 bool m_speedFactorIntermediate[3] = {false, false, false}; 

/* Constraints what the allow angle range is for the edge */
 float state_change_fit_quality_angle = api / 5;
 float state_change_fit_quality_position =  xpc / 8;
 float state_change_fit_allowed_speed_difference = 0.075f;


 int worker_counter = 27;
 
 void recompute_inferred_values(){
    xpc = (float)map_size_x_cm / (float)map_size_x;
    ypc = (float)map_size_y_cm / (float)map_size_y;
    api = (2.f * PI) / (float)map_size_angle;   

    state_change_fit_quality_angle = api / 4;
    state_change_fit_quality_position =  xpc / 4;
    state_change_fit_allowed_speed_difference = 0.05f; 
 }