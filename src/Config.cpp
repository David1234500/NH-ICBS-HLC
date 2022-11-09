#include <Config.hpp>

 int map_size_x = 60;
 int map_size_y = 60;
 int map_size_angle = 8;
 int map_size_speed = 4;

 float timestep_ms = 400.f;

 int map_size_x_cm = 450;
 int map_size_y_cm = 400;

 float xpc = (float)map_size_x_cm / (float)map_size_x;
 float ypc = (float)map_size_y_cm / (float)map_size_y;
 float api = (2.f * PI) / (float)map_size_angle;

 float safe_radius = 15.f;
 float heuristic_factor_backwards = 3.f;

 int zero_velocity_level = 1;
 float m_speedsFactor[4] = { -0.2f, 0.f, 0.2f, 0.4f}; 

/* Constraints what the allow angle range is for the edge */
 float state_change_fit_quality_angle = api / 2;
 float state_change_fit_quality_position =  xpc / 2;
 float state_change_fit_allowed_speed_difference = 0.15f;


 int worker_counter = 12;

 void recompute_inferred_values(){
    xpc = (float)map_size_x_cm / (float)map_size_x;
    ypc = (float)map_size_y_cm / (float)map_size_y;
    api = (2.f * PI) / (float)map_size_angle;   

    state_change_fit_quality_angle = api / 2;
    state_change_fit_quality_position =  xpc / 2;
    state_change_fit_allowed_speed_difference = 0.15f;
 }